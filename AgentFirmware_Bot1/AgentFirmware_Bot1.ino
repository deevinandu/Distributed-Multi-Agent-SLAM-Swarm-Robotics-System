#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUDP.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include "ekf.h"
#include "motor_control.h"
#include <esp_now.h>

// ================= CONFIGURATION =================
#define WIFI_SSID "ONEPLUS_7T"
#define WIFI_PASSWORD "dvhy03555"
#define AGENT_ID 1   // <-- BOT 1

// Network
const char* agent_ip = "10.218.44.255"; // Broadcast address
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

void checkForZoneUpdates(WiFiUDP& udp) {
    int packetSize = udp.parsePacket();
    if (packetSize == sizeof(ZonePacket)) {
        ZonePacket z;
        udp.read((uint8_t*)&z, sizeof(z));
        if (memcmp(z.magic, "ZONE", 4) == 0) {
            forbidden_min_x = z.min_x;
            forbidden_min_y = z.min_y;
            forbidden_max_x = z.max_x;
            forbidden_max_y = z.max_y;
            has_forbidden_zone = true;
            Serial.printf("[ZONE] Forbidden area: (%.2f,%.2f)-(%.2f,%.2f)\n",
                z.min_x, z.min_y, z.max_x, z.max_y);
        }
    }
}

// ================= PROTOCOL =================
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
};

// ================= GLOBALS =================
WiFiUDP           udp;
MotorController   motor;
Adafruit_MPU6050  mpu;
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
    sensors_event_t a, g, temp;
    if (mpu.getEvent(&a, &g, &temp)) {
        current_imu.angular_velocity.z    = g.gyro.z - gyroZ_offset;
        current_imu.linear_acceleration.x = a.acceleration.x - accX_offset;
    }
}

// ================= TELEMETRY =================
void sendPacket(float front, float left, float back, float right) {
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

    IPAddress bIp; bIp.fromString(agent_ip);
    udp.beginPacket(bIp, agent_port);
    udp.write((const uint8_t*)&p, sizeof(p));
    udp.endPacket();

    Serial.printf("[PKT] Pose:(%.2f,%.2f,%.0f°) | F:%.0fcm L:%.0fcm B:%.0fcm R:%.0fcm\n",
        robot_x, robot_y, degrees(robot_yaw),
        front*100, left*100, back*100, right*100);
}

// ================= MOVEMENT =================
void moveForwardReactive() {
    noInterrupts(); encoder_count = 0; interrupts();

    int left_fwd = min(255, (int)(MOTOR_SPEED * LEFT_SPEED_BOOST));
    motor.drive(left_fwd, MOTOR_SPEED);
    Serial.printf("[MOVE] Forward (L:%d R:%d) until 30cm...\n", left_fwd, MOTOR_SPEED);

    unsigned long start = millis();
    while (millis() - start < 15000) {
        float d = readFront();
        if (encoder_count % 5 == 0)
            Serial.printf("  Enc:%d | Front:%.2fm\n", encoder_count, d);
        if (d < OBSTACLE_THRESHOLD) {
            Serial.println("[HALT] 30cm front trigger!");
            break;
        }
        delay(30);
    }
    motor.stop();

    float m = (encoder_count * CM_PER_GROOVE) / 100.0;
    robot_x += m * cos(robot_yaw);
    robot_y += m * sin(robot_yaw);
    total_distance_traveled += m;
}

void turn(int deg, bool left) {
    Serial.printf("[TURN] %s %d° ...\n", left ? "LEFT" : "RIGHT", deg);

    unsigned long turn_ms = (unsigned long)(deg * 10);

    int left_spd  = left ? (int)(TURN_SPEED * LEFT_SPEED_BOOST) : -(int)(TURN_SPEED * LEFT_SPEED_BOOST);
    int right_spd = left ? -TURN_SPEED : TURN_SPEED;
    left_spd = constrain(left_spd, -255, 255);

    motor.drive(left_spd, right_spd);
    Serial.printf("[TURN] Motors L=%d R=%d for %lums\n", left_spd, right_spd, turn_ms);

    delay(turn_ms);
    motor.stop();

    robot_yaw += radians(left ? deg : -deg);
    while (robot_yaw >  PI) robot_yaw -= 2 * PI;
    while (robot_yaw < -PI) robot_yaw += 2 * PI;

    delay(200);
}

// ================= NAVIGATION STATE MACHINE (Left-Wall-Following) =================
//
// STATES:
//   FOLLOW       - Front clear + left wall present: drive straight w/ proportional steer
//   CORNER_ROUND - Left wall just vanished: drive straight to physically clear the corner
//   TURN_TO_WALL - After corner cleared: turn left in 15deg bites to re-acquire wall
//   AVOID_FRONT  - Front blocked: turn right away from wall until front clears
//
// WHY: Without CORNER_ROUND the bot would immediately turn left when the wall vanishes,
// then overshoot, see the wall again, steer right, lose the wall, turn left ... infinite loop.
// Driving straight through the corner gap first breaks the cycle completely.

enum NavState { FOLLOW, CORNER_ROUND, TURN_TO_WALL, AVOID_FRONT };
NavState       nav_state        = FOLLOW;
unsigned long  corner_start_ms  = 0;
const unsigned long CORNER_ROUND_MS = 600; // ms to coast straight to clear an open corner

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

    checkForZoneUpdates(udp);

    // Read sensors every cycle
    float front = readFront();
    float left  = readLeft();
    float back  = readBack();
    float right = readRight();
    float front_cm = front * 100.0f;
    float left_cm  = left  * 100.0f;

    Serial.printf("[NAV] %-12s | F:%.0f L:%.0f B:%.0f R:%.0f cm\n",
        nav_state==FOLLOW?"FOLLOW":nav_state==CORNER_ROUND?"CORNER_ROUND":
        nav_state==TURN_TO_WALL?"TURN_TO_WALL":"AVOID_FRONT",
        front_cm, left_cm, back*100, right*100);

    sendPacket(front, left, back, right);

    // ── Territory override (highest priority, hard-coded right turn) ──────────
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
            // Front blocked → stop and switch to avoidance
            motor.stop(); delay(150);
            Serial.println("[NAV] Front blocked -> AVOID_FRONT");
            nav_state = AVOID_FRONT;
            return;
        }
        if (left_cm > WALL_LOST_CM) {
            // Wall disappeared → coast straight to clear the corner gap
            motor.stop(); delay(100);
            Serial.println("[NAV] Wall lost -> CORNER_ROUND");
            corner_start_ms = millis();
            nav_state = CORNER_ROUND;
            return;
        }
        // Physical mapping: drive(first, second) = drive(RIGHT motor, LEFT motor)
        // RIGHT faster → robot turns LEFT. LEFT faster → robot turns RIGHT.
        if (left_cm < WALL_TOO_CLOSE_CM) {
            // Wall dangerously close: pivot RIGHT by reversing the right motor
            Serial.printf("[NAV] Too close %.0fcm -> pivot RIGHT (right motor reverse)\n", left_cm);
            motor.drive(-100, min(255, bR+50));  // right motor REVERSE, left motor forward
        } else if (left_cm > WALL_TOO_FAR_CM) {
            Serial.printf("[NAV] Too far %.0fcm -> steer left\n", left_cm);
            motor.drive(min(255, bL+50), max(60, bR-50));  // right motor faster -> turns left toward wall
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
        // Abort if front is suddenly blocked
        if (front_cm < 30.0f) {
            motor.stop(); delay(150);
            Serial.println("[NAV] Corner aborted by front wall -> AVOID_FRONT");
            nav_state = AVOID_FRONT;
            return;
        }
        // Wall re-appeared during burst? No need to keep going
        if (left_cm <= WALL_LOST_CM) {
            motor.stop(); delay(100);
            Serial.println("[NAV] Wall re-acquired during corner burst -> FOLLOW");
            nav_state = FOLLOW;
            return;
        }
        if (millis() - corner_start_ms < CORNER_ROUND_MS) {
            // Still coasting straight
            motor.drive(bL, bR);
            delay(50);
        } else {
            // Burst done, now seek the wall
            motor.stop(); delay(150);
            Serial.println("[NAV] Corner burst done -> TURN_TO_WALL");
            nav_state = TURN_TO_WALL;
        }
        break;

      // ──────────────────────────────────────────────────────────────────────
      case TURN_TO_WALL:
        if (left_cm <= WALL_LOST_CM) {
            // Wall found again
            Serial.printf("[NAV] Wall found %.0fcm -> FOLLOW\n", left_cm);
            nav_state = FOLLOW;
            return;
        }
        if (front_cm < 30.0f) {
            // Ran into wall while seeking
            Serial.println("[NAV] Front blocked while seeking -> AVOID_FRONT");
            nav_state = AVOID_FRONT;
            return;
        }
        // Small left turn, re-check next call
        Serial.println("[NAV] Seeking wall -> turn LEFT 15 deg");
        turn(15, true);
        break;

      // ──────────────────────────────────────────────────────────────────────
      case AVOID_FRONT:
        if (front_cm >= 35.0f) {
            Serial.println("[NAV] Front clear -> FOLLOW");
            nav_state = FOLLOW;
            return;
        }
        // Turn right until the front opens
        Serial.println("[NAV] Front blocked -> turn RIGHT 15 deg");
        turn(15, false);
        break;
    }
}

// ================= CORE =================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n[BOOT] BOT 1 Starting...");

    Wire.begin(21, 22);
    Wire.setClock(100000);

    Serial.println("[BOOT] MPU6050...");
    if (!mpu.begin()) {
        Serial.println("[ERROR] MPU6050 not found!");
        while(1) delay(1000);
    }
    Serial.println("[BOOT] MPU6050 OK. Calibrating...");
    float gz_sum = 0, ax_sum = 0;
    for (int i = 0; i < 50; i++) {
        sensors_event_t a, g, t;
        if (mpu.getEvent(&a, &g, &t)) {
            gz_sum += g.gyro.z;
            ax_sum += a.acceleration.x;
        }
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

    // EKF
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

    readIMU();
    sensor_msgs__msg__Imu msg;
    msg.angular_velocity.z    = current_imu.angular_velocity.z;
    msg.linear_acceleration.x = current_imu.linear_acceleration.x;
    ekf.predict(msg, now/1000.0);
    nav_msgs__msg__Odometry o = ekf.getOdom();
    robot_yaw = 2.0 * atan2(o.pose.pose.orientation.z, o.pose.pose.orientation.w);

    if (now - startup_time > (unsigned long)STARTUP_DELAY_SEC * 1000) {
        navigate();
    }
}
