#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUDP.h>
#include <Wire.h>
#include "MPU9250.h"

#include "ekf.h"
#include "motor_control.h"
#include <esp_now.h>

// ================= CONFIGURATION =================
#define WIFI_SSID "SwarmNet"
#define WIFI_PASSWORD "swarmrobotics"
#define AGENT_ID 1   // <-- BOT 1

// Network
const char* agent_ip = "192.168.137.1"; // Laptop Server IP address
const int agent_port = 8888;
const int local_port = 8888;

// ---- MUX (CD4051B) Pinout ----
#define MUX_ECHO_OUT  34
#define MUX_A         18
#define MUX_B         19

// ---- 4 Ultrasonic Trigger Pins (BOT 1) ----
#define TRIG_FRONT  2
#define TRIG_LEFT   4
#define TRIG_BACK   5
#define TRIG_RIGHT  13

const int TRIG_PINS[4]   = {TRIG_FRONT, TRIG_LEFT, TRIG_BACK, TRIG_RIGHT};
const int MUX_A_BITS[4]  = {0, 1, 0, 1};
const int MUX_B_BITS[4]  = {0, 0, 1, 1};

// ---- MPU6050 I2C ----
// SDA -> GPIO 21, SCL -> GPIO 22

// ---- Encoder ----
#define ENCODER_PIN 15
const float CM_PER_GROOVE = 30.0 / 28.0;

// ---- Sound Speed ----
#define SOUND_SPEED 0.0343f  // cm/us

// Navigation Parameters
const float OBSTACLE_THRESHOLD = 0.30;  // 30cm
const float SAFE_DISTANCE      = 0.50;  // 50cm
const int   MOTOR_SPEED        = 190;
const int   TURN_SPEED         = 200;
const int   STARTUP_DELAY_SEC  = 5;

// Left motor speed compensation
const float LEFT_SPEED_BOOST = 1.00;

// Wall-following parameters
const float WALL_TARGET_CM     = 25.0;
const float WALL_TOO_CLOSE_CM  = 8.0;
const float WALL_TOO_FAR_CM    = 40.0;
const float WALL_LOST_CM       = 80.0;

// ================= TERRITORY AVOIDANCE =================
// The server sends us a bounding box of the other bot's mapped area
// Packet: "ZONE" + float min_x, min_y, max_x, max_y
struct __attribute__((packed)) ZonePacket {
    char   magic[4];         // "ZONE"
    float  min_x, min_y;
    float  max_x, max_y;
};
float forbidden_min_x = 999, forbidden_min_y = 999;
float forbidden_max_x = -999, forbidden_max_y = -999;
bool  has_forbidden_zone = false;

bool isInForbiddenZone(float x, float y) {
    if (!has_forbidden_zone) return false;
    float margin = 0.20; // 20cm safety margin
    return (x > forbidden_min_x - margin && x < forbidden_max_x + margin &&
            y > forbidden_min_y - margin && y < forbidden_max_y + margin);
}

// ================= FRONTIER TARGET =================
// The server assigns frontier waypoints for exploration
// Packet: "TARG" + float target_x, target_y
struct __attribute__((packed)) TargetPacket {
    char  magic[4];    // "TARG"
    float target_x;
    float target_y;
};

float  target_x = 0, target_y = 0;
bool   has_target = false;
unsigned long last_target_time = 0;
const unsigned long TARGET_TIMEOUT_MS = 10000;  // 10s — fall back to wall-following
const float TARGET_REACHED_RADIUS = 0.30;       // 30cm

// ================= NAVIGATION STATE (forward declaration) =================
enum NavState { FOLLOW, CORNER_ROUND, TURN_TO_WALL, AVOID_FRONT, GO_TO_TARGET };
NavState nav_state = FOLLOW;

// ================= SERVER PACKET HANDLER =================
// Handles both ZONE (territory) and TARG (frontier) packets from the server
void checkForServerPackets(WiFiUDP& udp) {
    int packetSize = udp.parsePacket();
    if (packetSize <= 0) return;

    uint8_t buf[32];
    int len = udp.read(buf, min((int)sizeof(buf), packetSize));

    if (len == sizeof(ZonePacket) && memcmp(buf, "ZONE", 4) == 0) {
        ZonePacket z;
        memcpy(&z, buf, sizeof(z));
        forbidden_min_x = z.min_x;
        forbidden_min_y = z.min_y;
        forbidden_max_x = z.max_x;
        forbidden_max_y = z.max_y;
        // Zone is valid only if min < max (server sends 999/-999 to lift)
        has_forbidden_zone = (z.min_x < z.max_x && z.min_y < z.max_y);
        if (has_forbidden_zone) {
            Serial.printf("[ZONE] Forbidden area: (%.2f,%.2f)-(%.2f,%.2f)\n",
                z.min_x, z.min_y, z.max_x, z.max_y);
        } else {
            Serial.println("[ZONE] Forbidden zone LIFTED");
        }
    }
    /*
    else if (len >= (int)sizeof(TargetPacket) && memcmp(buf, "TARG", 4) == 0) {
        TargetPacket t;
        memcpy(&t, buf, sizeof(t));
        target_x = t.target_x;
        target_y = t.target_y;
        has_target = true;
        last_target_time = millis();
        if (nav_state != AVOID_FRONT) {  // Don't interrupt obstacle avoidance
            nav_state = GO_TO_TARGET;
        }
        Serial.printf("[TARGET] Assigned frontier: (%.2f, %.2f)\n", target_x, target_y);
    }
    */
}

// ================= LANDMARK DETECTION =================
// Geometric signature from 4 sensor readings for server-side loop closure.
// Landmark types (must match server definitions):
#define LM_NONE     0
#define LM_CORNER_L 1   // Front + Left walls close
#define LM_CORNER_R 2   // Front + Right walls close
#define LM_CORRIDOR 3   // Left + Right walls close, front open
#define LM_DEAD_END 4   // Front + Left + Right walls close
#define LM_OPEN     5   // All directions open

uint8_t detectLandmark(float front_cm, float left_cm, float back_cm, float right_cm) {
    const float close_thresh = 40.0;  // cm
    const float open_thresh  = 80.0;  // cm

    bool f_close = front_cm < close_thresh;
    bool l_close = left_cm  < close_thresh;
    bool r_close = right_cm < close_thresh;
    bool f_open  = front_cm > open_thresh;
    bool l_open  = left_cm  > open_thresh;
    bool r_open  = right_cm > open_thresh;

    if (f_close && l_close && r_close) return LM_DEAD_END;
    if (f_close && l_close)            return LM_CORNER_L;
    if (f_close && r_close)            return LM_CORNER_R;
    if (l_close && r_close && f_open)  return LM_CORRIDOR;
    if (f_open  && l_open  && r_open)  return LM_OPEN;
    return LM_NONE;
}

// ================= PROTOCOL (QuasarPacket v2) =================
struct __attribute__((packed)) QuasarPacket {
    char     magic[4];          // "QSRL"
    uint8_t  agent_id;
    float    odom_x;
    float    odom_y;
    float    odom_yaw;
    int32_t  encoder_total;
    uint32_t v2v_count;
    float    dist_front;
    float    dist_left;
    float    dist_back;
    float    dist_right;
    uint8_t  landmark_type;     // Geometric signature for SLAM loop closure
};

// ================= GLOBALS =================
WiFiUDP           udp;
MotorController   motor;
MPU9250 IMU(Wire, 0x68);
EKF               ekf;

sensor_msgs__msg__Imu current_imu;
volatile long    encoder_count       = 0;
volatile int32_t global_encoder_total = 0;

float robot_x   = 0.0;
float robot_y   = 0.0;
float robot_yaw = 0.0;
float total_distance_traveled = 0.0;

const float MIN_TRAVEL_DISTANCE = 1.6;
const float RETURN_THRESHOLD    = 0.50;
bool mission_complete = false;

float gyroZ_offset = 0.0;
float accX_offset  = 0.0;
unsigned long startup_time = 0;

// V2V
volatile uint32_t v2v_packet_received_total = 0;
typedef struct { float distance; } struct_message;
void OnDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len) {
    v2v_packet_received_total++;
}

// ================= INTERRUPTS =================
void IRAM_ATTR encoderISR() {
    encoder_count++;
    global_encoder_total++;
}

// ================= SENSORS =================
void selectMuxChannel(int ch) {
    digitalWrite(MUX_A, MUX_A_BITS[ch]);
    digitalWrite(MUX_B, MUX_B_BITS[ch]);
    delayMicroseconds(50);
}

float readSensorRaw(int ch) {
    selectMuxChannel(ch);
    digitalWrite(TRIG_PINS[ch], LOW);
    delayMicroseconds(5);
    digitalWrite(TRIG_PINS[ch], HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PINS[ch], LOW);

    long duration = pulseIn(MUX_ECHO_OUT, HIGH, 30000);
    if (duration == 0) return 4.0;
    return (duration * SOUND_SPEED) / 2.0f / 100.0f;
}

float readSensor(int ch) {
    float r[3];
    for (int i = 0; i < 3; i++) {
        r[i] = readSensorRaw(ch);
        delayMicroseconds(500);
    }
    if (r[0] > r[1]) { float t = r[0]; r[0] = r[1]; r[1] = t; }
    if (r[1] > r[2]) { float t = r[1]; r[1] = r[2]; r[2] = t; }
    if (r[0] > r[1]) { float t = r[0]; r[0] = r[1]; r[1] = t; }
    return r[1];
}

float readFront() { return readSensor(0); }
float readLeft()  { return readSensor(1); }
float readBack()  { return readSensor(2); }
float readRight() { return readSensor(3); }

void readIMU() {
    IMU.readSensor();
    current_imu.angular_velocity.z    = IMU.getGyroZ_rads() - gyroZ_offset;
    current_imu.linear_acceleration.x = IMU.getAccelX_mss() - accX_offset;
}

// ================= ODOMETRY =================
// Update position from encoder counts accumulated since last call.
// Uses commanded robot_yaw (not IMU) for heading to avoid drift.
void updateOdometry() {
    noInterrupts();
    long counts = encoder_count;
    encoder_count = 0;
    interrupts();

    if (counts > 0) {
        float m = (counts * CM_PER_GROOVE) / 100.0;
        robot_x += m * cos(robot_yaw);
        robot_y += m * sin(robot_yaw);
        total_distance_traveled += m;
    }
}

// ================= TELEMETRY =================
void sendPacket(float front, float left, float back, float right, uint8_t landmark) {
    QuasarPacket p;
    memcpy(p.magic, "QSRL", 4);
    p.agent_id      = AGENT_ID;
    p.odom_x        = robot_x;
    p.odom_y        = robot_y;
    p.odom_yaw      = robot_yaw;
    p.encoder_total = global_encoder_total;
    p.v2v_count     = v2v_packet_received_total;
    p.dist_front    = front;
    p.dist_left     = left;
    p.dist_back     = back;
    p.dist_right    = right;
    p.landmark_type = landmark;

    IPAddress bIp; bIp.fromString(agent_ip);
    int begin_ok = udp.beginPacket(bIp, agent_port);
    int write_len = udp.write((const uint8_t*)&p, sizeof(p));
    int end_ok = udp.endPacket();

    if (!begin_ok || write_len != sizeof(p) || !end_ok) {
        Serial.printf("[PKT-ERR] Send failed! Target:%s:%d | begin=%d write=%d/%d end=%d\n", 
                      agent_ip, agent_port, begin_ok, write_len, (int)sizeof(p), end_ok);
    } else {
        Serial.printf("[PKT] To:%s:%d | Pose:(%.2f,%.2f,%.0f°) | F:%.0fcm L:%.0fcm B:%.0fcm R:%.0fcm | LM:%d\n",
            agent_ip, agent_port,
            robot_x, robot_y, degrees(robot_yaw),
            front*100, left*100, back*100, right*100, landmark);
    }
}

// ================= MOVEMENT =================
void turn(int deg, bool left) {
    Serial.printf("[TURN] %s %d° ...\n", left ? "LEFT" : "RIGHT", deg);

    // Capture any forward movement before turning
    updateOdometry();

    int left_spd  = left ? (int)(TURN_SPEED * LEFT_SPEED_BOOST) : -(int)(TURN_SPEED * LEFT_SPEED_BOOST);
    int right_spd = left ? -TURN_SPEED : TURN_SPEED;
    left_spd = constrain(left_spd, -255, 255);

    double target_rad = radians(deg);
    double integrated_yaw = 0.0;
    unsigned long last_time = micros();
    unsigned long start_time = millis();
    const unsigned long TURN_TIMEOUT_MS = 5000; // 5 seconds safety timeout

    motor.drive(left_spd, right_spd);
    Serial.printf("[TURN] Motors L=%d R=%d (Target: %.3f rad)\n", left_spd, right_spd, target_rad);

    while (fabs(integrated_yaw) < target_rad && (millis() - start_time) < TURN_TIMEOUT_MS) {
        delay(2);
        unsigned long now = micros();
        double dt = (double)(now - last_time) / 1000000.0;
        last_time = now;

        IMU.readSensor();
        double gyro_z = IMU.getGyroZ_rads() - gyroZ_offset;
        integrated_yaw += gyro_z * dt;
    }

    motor.stop();
    Serial.printf("[TURN] Stopped. Target: %d deg, Integrated: %.2f deg, Time: %lu ms\n",
                  deg, degrees(integrated_yaw), millis() - start_time);

    // Update yaw using the COMMANDED angle (not IMU — avoids drift).
    // The server knows exact turn commands so it can reconstruct heading.
    robot_yaw += radians(left ? deg : -deg);
    while (robot_yaw >  PI) robot_yaw -= 2 * PI;
    while (robot_yaw < -PI) robot_yaw += 2 * PI;

    // Clear encoder counts from the turn (rotation, not translation)
    noInterrupts(); encoder_count = 0; interrupts();

    delay(200);
}

// ================= NAVIGATION STATE MACHINE =================
//
// Left-Wall-Following + Server-Directed Go-To-Target
//
// STATES:
//   FOLLOW       - Front clear + left wall present: drive straight w/ proportional steer
//   CORNER_ROUND - Left wall just vanished: drive straight to clear the corner
//   TURN_TO_WALL - After corner cleared: turn left in 15° bites to re-acquire wall
//   AVOID_FRONT  - Front blocked: turn right until front clears
//   GO_TO_TARGET - Server assigned frontier waypoint: drive toward (target_x, target_y)
//
// Wall-following is the FALLBACK when no frontier target is assigned.
// The server dynamically assigns targets based on frontier detection.

unsigned long  corner_start_ms  = 0;
const unsigned long CORNER_ROUND_MS = 600;

bool checkMissionComplete() {
    float d = sqrt(robot_x*robot_x + robot_y*robot_y);
    return (total_distance_traveled > MIN_TRAVEL_DISTANCE && d < RETURN_THRESHOLD);
}

void navigate() {
    if (checkMissionComplete()) {
        mission_complete = true;
        Serial.println("[DONE] Mission complete!");
        motor.stop(); return;
    }

    checkForServerPackets(udp);

    // Update position from any encoder ticks since last call
    updateOdometry();

    // Read sensors every cycle
    float front = readFront();
    float left  = readLeft();
    float back  = readBack();
    float right = readRight();
    float front_cm = front * 100.0f;
    float left_cm  = left  * 100.0f;
    float right_cm = right * 100.0f;

    // Detect landmark signature from the 4 sensors
    uint8_t landmark = detectLandmark(front_cm, left_cm, back*100, right_cm);

    Serial.printf("[NAV] %-12s | F:%.0f L:%.0f B:%.0f R:%.0f cm | LM:%d\n",
        nav_state==FOLLOW?"FOLLOW":nav_state==CORNER_ROUND?"CORNER_ROUND":
        nav_state==TURN_TO_WALL?"TURN_TO_WALL":nav_state==AVOID_FRONT?"AVOID_FRONT":"GO_TO_TARGET",
        front_cm, left_cm, back*100, right_cm, landmark);

    sendPacket(front, left, back, right, landmark);

    // ── Territory override (highest priority) ────────────────────────────────
    float lx = robot_x + 0.30f * cos(robot_yaw);
    float ly = robot_y + 0.30f * sin(robot_yaw);
    if (isInForbiddenZone(lx, ly)) {
        Serial.println("[ZONE] Other bot territory → turn RIGHT 30 deg");
        motor.stop(); delay(150);
        turn(30, false);
        nav_state = FOLLOW;
        return;
    }

    int bL = (int)(MOTOR_SPEED * LEFT_SPEED_BOOST);
    int bR = MOTOR_SPEED;

    switch (nav_state) {

      // ──────────────────────────────────────────────────────────────────────
      case FOLLOW:
        if (front_cm < 30.0f) {
            motor.stop(); delay(150);
            Serial.println("[NAV] Front blocked -> AVOID_FRONT");
            nav_state = AVOID_FRONT;
            return;
        }
        if (left_cm > WALL_LOST_CM) {
            motor.stop(); delay(100);
            Serial.println("[NAV] Wall lost -> CORNER_ROUND");
            corner_start_ms = millis();
            nav_state = CORNER_ROUND;
            return;
        }
        if (left_cm < WALL_TOO_CLOSE_CM) {
            Serial.printf("[NAV] Too close %.0fcm -> pivot RIGHT\n", left_cm);
            motor.drive(max(60, bL-50), min(255, bR+50));
        } else if (left_cm > WALL_TOO_FAR_CM) {
            Serial.printf("[NAV] Too far %.0fcm -> steer left\n", left_cm);
            motor.drive(min(255, bL+50), max(60, bR-50));
        } else {
            Serial.printf("[NAV] Tracking %.0fcm -> straight\n", left_cm);
            motor.drive(bL, bR);
        }
        delay(300);
        motor.stop();
        delay(100);
        break;

      // ──────────────────────────────────────────────────────────────────────
      case CORNER_ROUND:
        if (front_cm < 30.0f) {
            motor.stop(); delay(150);
            Serial.println("[NAV] Corner aborted by front wall -> AVOID_FRONT");
            nav_state = AVOID_FRONT;
            return;
        }
        if (left_cm <= WALL_LOST_CM) {
            motor.stop(); delay(100);
            Serial.println("[NAV] Wall re-acquired during corner burst -> FOLLOW");
            nav_state = FOLLOW;
            return;
        }
        if (millis() - corner_start_ms < CORNER_ROUND_MS) {
            motor.drive(bL, bR);
            delay(50);
        } else {
            motor.stop(); delay(150);
            Serial.println("[NAV] Corner burst done -> TURN_TO_WALL");
            nav_state = TURN_TO_WALL;
        }
        break;

      // ──────────────────────────────────────────────────────────────────────
      case TURN_TO_WALL:
        if (left_cm <= WALL_LOST_CM) {
            Serial.printf("[NAV] Wall found %.0fcm -> FOLLOW\n", left_cm);
            nav_state = FOLLOW;
            return;
        }
        if (front_cm < 30.0f) {
            Serial.println("[NAV] Front blocked while seeking -> AVOID_FRONT");
            nav_state = AVOID_FRONT;
            return;
        }
        Serial.println("[NAV] Seeking wall -> turn LEFT 15 deg");
        turn(15, true);
        break;

      // ──────────────────────────────────────────────────────────────────────
      case AVOID_FRONT:
        if (front_cm >= 35.0f) {
            // Front clear — resume GO_TO_TARGET if a target is active, else wall-follow
            if (has_target && millis() - last_target_time < TARGET_TIMEOUT_MS) {
                Serial.println("[NAV] Front clear -> GO_TO_TARGET");
                nav_state = GO_TO_TARGET;
            } else {
                Serial.println("[NAV] Front clear -> FOLLOW");
                nav_state = FOLLOW;
            }
            return;
        }
        // Turn right (away from left wall) until the front opens
        Serial.println("[NAV] Front blocked -> turn RIGHT 15 deg");
        turn(15, false);
        break;

      // ──────────────────────────────────────────────────────────────────────
      case GO_TO_TARGET: {
        // Check if target is still valid (server refreshes every 3s)
        if (!has_target || millis() - last_target_time > TARGET_TIMEOUT_MS) {
            Serial.println("[NAV] Target expired -> FOLLOW");
            has_target = false;
            nav_state = FOLLOW;
            return;
        }

        // Front obstacle takes priority
        if (front_cm < 30.0f) {
            motor.stop(); delay(150);
            Serial.println("[NAV] Front blocked during go-to-target -> AVOID_FRONT");
            nav_state = AVOID_FRONT;
            return;
        }

        // Check if we reached the target
        float dist_to_target = sqrt(pow(target_x - robot_x, 2) + pow(target_y - robot_y, 2));
        if (dist_to_target < TARGET_REACHED_RADIUS) {
            Serial.printf("[NAV] Target reached (%.2fm away) -> FOLLOW\n", dist_to_target);
            has_target = false;
            nav_state = FOLLOW;
            return;
        }

        // Compute heading error to target
        float desired_yaw = atan2(target_y - robot_y, target_x - robot_x);
        float heading_error = desired_yaw - robot_yaw;
        while (heading_error >  PI) heading_error -= 2 * PI;
        while (heading_error < -PI) heading_error += 2 * PI;

        if (fabs(heading_error) > radians(15)) {
            // Turn toward target
            int deg = min(30, (int)(fabs(heading_error) * 180.0 / PI));
            if (deg < 5) deg = 5;
            Serial.printf("[NAV] Heading to target: err=%.0f° -> turn %s %d°\n",
                degrees(heading_error), heading_error > 0 ? "LEFT" : "RIGHT", deg);
            turn(deg, heading_error > 0);
        } else {
            // Drive forward toward target
            Serial.printf("[NAV] Driving to target (%.2fm away)\n", dist_to_target);
            motor.drive(bL, bR);
            delay(300);
            motor.stop();
            delay(100);
        }
        break;
      }
    }
}

// ================= CORE =================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n[BOOT] BOT 1 Starting...");

    Wire.begin(21, 22);
    Wire.setClock(100000);

    Serial.println("[BOOT] MPU9250...");
    int status = IMU.begin();
    if (status < 0) {
        Serial.println("[ERROR] MPU9250 not found!");
        while(1) delay(1000);
    }
    Serial.println("[BOOT] MPU9250 OK. Calibrating...");
    float gz_sum = 0, ax_sum = 0;
    for (int i = 0; i < 50; i++) {
        IMU.readSensor();
        gz_sum += IMU.getGyroZ_rads();
        ax_sum += IMU.getAccelX_mss();
        delay(20);
    }
    gyroZ_offset = gz_sum / 50;
    accX_offset  = ax_sum / 50;

    // MUX pins
    pinMode(MUX_A, OUTPUT);
    pinMode(MUX_B, OUTPUT);
    pinMode(MUX_ECHO_OUT, INPUT);

    // Trigger pins
    Serial.printf("[BOOT] Trig pins (F:%d, L:%d, B:%d, R:%d)...\n",
        TRIG_FRONT, TRIG_LEFT, TRIG_BACK, TRIG_RIGHT);
    for (int i = 0; i < 4; i++) {
        pinMode(TRIG_PINS[i], OUTPUT);
        digitalWrite(TRIG_PINS[i], LOW);
    }

    // Encoder
    pinMode(ENCODER_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderISR, RISING);

    // Motors
    motor.init();
    motor.testMotors();

    // Sensor self-test
    Serial.println("[BOOT] Sensor self-test...");
    Serial.printf("  Front: %.0f cm\n", readFront()*100);
    Serial.printf("  Left:  %.0f cm\n", readLeft()*100);
    Serial.printf("  Back:  %.0f cm\n", readBack()*100);
    Serial.printf("  Right: %.0f cm\n", readRight()*100);

    // WiFi
    Serial.printf("[WIFI] Connecting to %s...\n", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    unsigned long wt = millis();
    while (WiFi.status() != WL_CONNECTED && millis()-wt < 10000) {
        delay(500); Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\n[WIFI] Connected! IP: %s\n", WiFi.localIP().toString().c_str());
        if (esp_now_init() == ESP_OK) {
            Serial.println("[SWARM] V2V Initialized.");
            esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
        }
    } else {
        Serial.println("\n[WIFI] Failed. Running offline.");
    }
    udp.begin(local_port);

    // EKF (initialized for internal state estimation)
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(6);
    ekf.init(millis()/1000.0, x0);

    startup_time = millis();
    Serial.printf("[BOOT] BOT 1 READY. Starting in %ds...\n", STARTUP_DELAY_SEC);
}

void loop() {
    if (mission_complete) {
        digitalWrite(2, (millis()/500) % 2);
        return;
    }

    unsigned long now = millis();

    // Read IMU and run EKF predict (for internal state estimation)
    readIMU();
    sensor_msgs__msg__Imu msg;
    msg.angular_velocity.z    = current_imu.angular_velocity.z;
    msg.linear_acceleration.x = current_imu.linear_acceleration.x;
    ekf.predict(msg, now/1000.0);

    // NOTE: robot_yaw is NOT overwritten from EKF output.
    // Yaw is maintained purely by commanded turn angles in turn().
    // This avoids gyro drift accumulation — the server knows exact turn
    // commands and can reconstruct heading accurately for mapping.

    if (now - startup_time > (unsigned long)STARTUP_DELAY_SEC * 1000) {
        navigate();
    }
}
