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
#define AGENT_ID 1 

// Network
const char* agent_ip = "10.218.44.255"; // Broadcast address
const int agent_port = 8888;
const int local_port = 8888;

// ---- MUX (CD4051B) Pinout ----
#define MUX_ECHO_OUT  34   // MUX pin 3 (common output) -> ESP32 GPIO 34
#define MUX_A         18   // MUX address bit A
#define MUX_B         19   // MUX address bit B

// ---- 4 Ultrasonic Trigger Pins (individual) ----
// Sensor Index: 0=Front, 1=Left, 2=Back, 3=Right
// MUX Channel:  Y0=Front, Y1=Left, Y2=Back, Y3=Right
// A=0,B=0 -> Y0  A=1,B=0 -> Y1  A=0,B=1 -> Y2  A=1,B=1 -> Y3
#define TRIG_FRONT  2
#define TRIG_LEFT   4
#define TRIG_BACK    5
#define TRIG_RIGHT  13

const int TRIG_PINS[4]   = {TRIG_FRONT, TRIG_LEFT, TRIG_BACK, TRIG_RIGHT};
const int MUX_A_BITS[4]  = {0, 1, 0, 1}; // A bit for each channel
const int MUX_B_BITS[4]  = {0, 0, 1, 1}; // B bit for each channel

// ---- MPU6050 I2C ----
// SDA -> GPIO 21, SCL -> GPIO 22 (already default for Wire.begin)

// ---- Encoder ----
#define ENCODER_PIN 15
const float CM_PER_GROOVE = 30.0 / 28.0;

// ---- Sound Speed ----
#define SOUND_SPEED 0.0343f  // cm/us

// Navigation Parameters
const float OBSTACLE_THRESHOLD = 0.30;  // 40cm — stop if front < this
const float SAFE_DISTANCE      = 0.50;  // 50cm
const int   MOTOR_SPEED        = 190;
const int   TURN_SPEED         = 200;
const int   STARTUP_DELAY_SEC  = 5;

// Left motor speed compensation (left is slower — tune in 0.05 steps)
// 1.00 = no boost | 1.15 = 15% boost | increase if still drifting left
const float LEFT_SPEED_BOOST = 1.00;

// Wall-following parameters
const float WALL_TARGET_CM     = 25.0;  // Ideal distance from left wall (cm)
const float WALL_TOO_CLOSE_CM  = 8.0;  // Steer right if closer than this
const float WALL_TOO_FAR_CM    = 40.0;  // Steer left if farther than this
const float WALL_LOST_CM       = 80.0;  // Wall lost → turn left to find it

// ================= PROTOCOL =================
// Packet now carries 4 fixed distances instead of 181-element sweep
struct __attribute__((packed)) QuasarPacket {
    char     magic[4];          // "QSRL"
    uint8_t  agent_id;
    float    odom_x;
    float    odom_y;
    float    odom_yaw;
    int32_t  encoder_total;
    uint32_t v2v_count;
    float    dist_front;        // meters
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

// V2V (kept for future multi-bot expansion)
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

// Select MUX channel (0=Front, 1=Left, 2=Back, 3=Right)
void selectMuxChannel(int ch) {
    digitalWrite(MUX_A, MUX_A_BITS[ch]);
    digitalWrite(MUX_B, MUX_B_BITS[ch]);
    delayMicroseconds(50); // CD4051B switch settling
}

// Raw single pulse reading for one sensor (channel 0-3)
float readSensorRaw(int ch) {
    selectMuxChannel(ch);

    // Trigger
    digitalWrite(TRIG_PINS[ch], LOW);
    delayMicroseconds(5);
    digitalWrite(TRIG_PINS[ch], HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PINS[ch], LOW);

    // Listen (30ms = ~5m max)
    long duration = pulseIn(MUX_ECHO_OUT, HIGH, 30000);
    if (duration == 0) return 4.0; // timeout -> 4m

    return (duration * SOUND_SPEED) / 2.0f / 100.0f; // cm/us -> meters
}

// Median-of-3 filter for one sensor
float readSensor(int ch) {
    float r[3];
    for (int i = 0; i < 3; i++) {
        r[i] = readSensorRaw(ch);
        delayMicroseconds(500);
    }
    // Sort
    if (r[0] > r[1]) { float t = r[0]; r[0] = r[1]; r[1] = t; }
    if (r[1] > r[2]) { float t = r[1]; r[1] = r[2]; r[2] = t; }
    if (r[0] > r[1]) { float t = r[0]; r[0] = r[1]; r[1] = t; }
    return r[1]; // median
}

// Convenience names
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
    Serial.printf("[MOVE] Forward (L:%d R:%d) until 40cm...\n", left_fwd, MOTOR_SPEED);

    unsigned long start = millis();
    while (millis() - start < 15000) {
        float d = readFront();
        if (encoder_count % 5 == 0)
            Serial.printf("  Enc:%d | Front:%.2fm\n", encoder_count, d);
        if (d < OBSTACLE_THRESHOLD) {
            Serial.println("[HALT] 40cm front trigger!");
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

    // Time-based: ~10ms per degree at TURN_SPEED=200 differential
    // (was 30ms which caused 180° overshooting)
    unsigned long turn_ms = (unsigned long)(deg * 10);

    // Start motors
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

    delay(200);  // Settle
}


// ================= NAVIGATION (Left-Wall-Following) =================
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

    // ===== STOP first — always halt before deciding =====
    motor.stop();
    delay(200);  // Let momentum settle

    // Read all 4 sensors (in meters)
    float front = readFront();
    float left  = readLeft();
    float back  = readBack();
    float right = readRight();

    // Convert to cm for wall-following logic
    float front_cm = front * 100.0;
    float left_cm  = left  * 100.0;
    float right_cm = right * 100.0;

    Serial.printf("[NAV] F:%.0fcm L:%.0fcm B:%.0fcm R:%.0fcm\n",
        front_cm, left_cm, back*100, right_cm);

    // Send telemetry
    sendPacket(front, left, back, right);

    // ===== PRIORITY 1: Front obstacle — prefer LEFT if available =====
    if (front_cm < 30.0) {
        if (left_cm > 40.0) {
            Serial.println("[NAV] FRONT BLOCKED, LEFT OPEN → Turn LEFT 15°");
            turn(15, true);
        } else {
            Serial.println("[NAV] FRONT BLOCKED, LEFT CLOSED → Turn RIGHT 15°");
            turn(15, false);
        }
        return;
    }

    // ===== PRIORITY 2: Left wall following =====
    int base_left  = (int)(MOTOR_SPEED * LEFT_SPEED_BOOST);
    int base_right = MOTOR_SPEED;

    if (left_cm > WALL_LOST_CM) {
        // Left wall disappeared → turn left to find it
        Serial.println("[NAV] LEFT WALL LOST → Turn LEFT 15°");
        turn(15, true);
        return;

    } else if (left_cm < WALL_TOO_CLOSE_CM) {
        // Too close to left wall → steer right
        int correction = 40;
        Serial.printf("[NAV] TOO CLOSE (%.0fcm) → Steer right\n", left_cm);
        motor.drive(min(255, base_left + correction), max(80, base_right - correction));

    } else if (left_cm > WALL_TOO_FAR_CM) {
        // Too far from left wall → steer left
        int correction = 40;
        Serial.printf("[NAV] TOO FAR (%.0fcm) → Steer left\n", left_cm);
        motor.drive(max(80, base_left - correction), min(255, base_right + correction));

    } else {
        // Sweet spot → drive straight
        Serial.printf("[NAV] TRACKING (%.0fcm) → Straight\n", left_cm);
        motor.drive(base_left, base_right);
    }

    delay(300);  // Drive for 300ms then stop and re-evaluate
    motor.stop();
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n[BOOT] System Starting...");

    // I2C: SDA=21, SCL=22
    Serial.println("[BOOT] I2C (SDA:21, SCL:22)");
    Wire.begin(21, 22);
    Wire.setClock(100000);

    // MPU6050
    Serial.println("[BOOT] MPU6050...");
    if (!mpu.begin()) {
        Serial.println("[ERROR] MPU6050 not found! Check wiring (SDA:21, SCL:22)");
        while(1) delay(1000);
    }
    Serial.println("[BOOT] MPU6050 OK. Calibrating (50 samples)...");
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
    Serial.printf("[BOOT] Gyro offset: %.4f | Acc offset: %.4f\n", gyroZ_offset, accX_offset);

    // MUX pins
    Serial.println("[BOOT] MUX (A:18, B:19, ECHO:34)...");
    pinMode(MUX_A, OUTPUT);
    pinMode(MUX_B, OUTPUT);
    pinMode(MUX_ECHO_OUT, INPUT);

    // 4 Trigger pins
    Serial.println("[BOOT] Trig pins (F:16, L:17, B:5, R:13)...");
    for (int i = 0; i < 4; i++) {
        pinMode(TRIG_PINS[i], OUTPUT);
        digitalWrite(TRIG_PINS[i], LOW);
    }

    // Encoder (GPIO 15)
    Serial.println("[BOOT] Encoder (GPIO 15)...");
    pinMode(ENCODER_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderISR, RISING);

    // Motors
    Serial.println("[BOOT] Motors...");
    motor.init();

    // === MOTOR DIAGNOSTIC - runs once on boot ===
    // Watch which wheel spins for each step and note it in Serial Monitor
    motor.testMotors();


    // Quick sensor self-test
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
        Serial.println("\n[WIFI] Failed (10s timeout). Running offline.");
    }
    udp.begin(local_port);

    // EKF
    Serial.println("[BOOT] EKF...");
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(6);
    ekf.init(millis()/1000.0, x0);

    startup_time = millis();
    Serial.printf("[BOOT] DONE. Starting in %ds...\n", STARTUP_DELAY_SEC);
}

void loop() {
    if (mission_complete) {
        digitalWrite(2, (millis()/500) % 2);
        return;
    }

    unsigned long now = millis();

    // EKF prediction
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
