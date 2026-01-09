#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUDP.h>

#include "ekf.h"
#include "motor_control.h"

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ================= CONFIGURATION =================
#define WIFI_SSID "ONEPLUS_7T"
#define WIFI_PASSWORD "dvhy03555"
#define AGENT_ID 1 

// Network
const char* agent_ip = "10.132.45.255";
const int agent_port = 8888;
const int local_port = 8888;

// Servo (14-bit PWM)
#define SERVO_PIN 13
#define SERVO_FREQ 50
#define SERVO_RES 14
const int minDuty = 410;  
const int maxDuty = 1966;
const int stepDelay = 50;

// Ultrasonic (HC-SR04)
#define TRIG_PIN 5
#define ECHO_PIN 18
#define SOUND_SPEED 0.0343

// Encoder
#define ENCODER_PIN 34
const float CM_PER_GROOVE = 30.0 / 28.0;  // 1.07 cm per groove
const int GROOVES_PER_30CM = 28;          // 28 grooves = 30 cm

// Navigation
const int STARTUP_DELAY_SEC = 20;
const float MOVE_DISTANCE_CM = 30.0;
const int MOTOR_SPEED = 150;  // PWM value (0-255)

// ================= PROTOCOL =================
struct __attribute__((packed)) QuasarPacket {
    char magic[4];
    uint8_t agent_id;
    float odom_x;
    float odom_y;
    float odom_yaw;
    uint16_t scan_count;
    float ranges[181];
};

struct __attribute__((packed)) CommandPacket {
    char magic[4];
    float linear_x;
    float angular_z;
};

// ================= GLOBALS =================
WiFiUDP udp;
MotorController motor;
EKF ekf;
Adafruit_MPU6050 mpu;

sensor_msgs__msg__Imu current_imu;
nav_msgs__msg__Odometry current_encoder_odom;

// Encoder
volatile long encoder_count = 0;
long last_encoder_count = 0;

// State Machine
enum RobotState {
    STATE_STARTUP,
    STATE_SCANNING,
    STATE_MOVING,
    STATE_STOPPED
};
RobotState current_state = STATE_STARTUP;

// Position tracking
float robot_x = 0.0;
float robot_y = 0.0;
float robot_yaw = 0.0;

// Timers
unsigned long last_imu_time = 0;
unsigned long startup_time = 0;
const long IMU_INTERVAL = 10;

// ================= INTERRUPTS =================
void IRAM_ATTR encoderISR() {
    encoder_count++;
}

// ================= HELPERS =================
void setServoAngle(int angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    int duty = map(angle, 0, 180, minDuty, maxDuty);
    ledcWrite(SERVO_PIN, duty);
}

float readUltrasonic() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    long duration = pulseIn(ECHO_PIN, HIGH, 30000);
    if (duration == 0) return 0.0;
    
    float distanceM = (duration * SOUND_SPEED / 2.0) / 100.0;
    if (distanceM > 4.0 || distanceM < 0.02) return 0.0;
    return distanceM;
}

void readIMU(double t) {
    sensors_event_t a, g, temp;
    if (mpu.getEvent(&a, &g, &temp)) {
        current_imu.angular_velocity.z = g.gyro.z;
        current_imu.linear_acceleration.z = a.acceleration.z;
    }
}

void performScan(QuasarPacket& packet) {
    packet.scan_count = 181;
    Serial.println("[SCAN] Starting 180-degree sweep...");
    
    for (int angle = 0; angle <= 180; angle++) {
        setServoAngle(angle);
        delay(stepDelay);
        packet.ranges[angle] = readUltrasonic();
        
        if (angle % 45 == 0) {
            Serial.printf("  Angle %d: %.2f m\n", angle, packet.ranges[angle]);
        }
    }
    setServoAngle(90); // Center position
    Serial.println("[SCAN] Complete!");
}

void sendPacket(QuasarPacket& packet) {
    memcpy(packet.magic, "QSRL", 4);
    packet.agent_id = AGENT_ID;
    packet.odom_x = robot_x;
    packet.odom_y = robot_y;
    packet.odom_yaw = robot_yaw;
    
    IPAddress broadcastIp;
    broadcastIp.fromString(agent_ip);
    udp.beginPacket(broadcastIp, agent_port);
    udp.write((const uint8_t*)&packet, sizeof(packet));
    udp.endPacket();
    
    Serial.println("[UDP] Packet sent!");
}

void moveForward(float distance_cm) {
    Serial.printf("[MOVE] Moving forward %.1f cm...\n", distance_cm);
    
    int target_grooves = (int)(distance_cm / CM_PER_GROOVE);
    encoder_count = 0;
    
    motor.drive(MOTOR_SPEED, MOTOR_SPEED);
    
    while (encoder_count < target_grooves) {
        // Update IMU while moving
        unsigned long now = millis();
        if (now - last_imu_time >= IMU_INTERVAL) {
            last_imu_time = now;
            readIMU(now / 1000.0);
            ekf.predict(current_imu, now / 1000.0);
        }
        delay(10);
    }
    
    motor.stop();
    
    // Update position (simple dead reckoning)
    float moved_cm = encoder_count * CM_PER_GROOVE;
    robot_x += (moved_cm / 100.0) * cos(robot_yaw);
    robot_y += (moved_cm / 100.0) * sin(robot_yaw);
    
    Serial.printf("[MOVE] Done! Grooves: %ld, Distance: %.1f cm\n", encoder_count, moved_cm);
    Serial.printf("[POS] X: %.2f m, Y: %.2f m\n", robot_x, robot_y);
}

// ================= SETUP =================
void setup() {
    Serial.begin(115200);
    Serial.println("\n========================================");
    Serial.println("Distributed SLAM Agent - Starting Up");
    Serial.println("========================================\n");
    
    Wire.begin(21, 22);

    // MPU6050
    if (!mpu.begin()) {
        Serial.println("[ERROR] MPU6050 not found!");
    } else {
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
        Serial.println("[OK] MPU6050 initialized");
    }
    
    // Servo
    if(!ledcAttach(SERVO_PIN, SERVO_FREQ, SERVO_RES)) {
        Serial.println("[ERROR] Servo attach failed!");
    } else {
        setServoAngle(90);
        Serial.println("[OK] Servo initialized (centered)");
    }
    
    // Ultrasonic
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    float testDist = readUltrasonic();
    Serial.printf("[OK] Ultrasonic initialized (test: %.2f m)\n", testDist);
    
    // Encoder
    pinMode(ENCODER_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderISR, RISING);
    Serial.println("[OK] Encoder initialized");
    
    // EKF
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(6);
    ekf.init(millis()/1000.0, x0);
    Serial.println("[OK] EKF initialized");
    
    // Motors
    motor.init();
    Serial.println("[OK] Motors initialized");

    // WiFi
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("[WIFI] Connecting");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.printf("\n[WIFI] Connected! IP: %s\n", WiFi.localIP().toString().c_str());
    udp.begin(local_port);
    
    // Startup delay
    Serial.printf("\n[STARTUP] Waiting %d seconds...\n", STARTUP_DELAY_SEC);
    Serial.println("Check all sensors are working!\n");
    startup_time = millis();
    current_state = STATE_STARTUP;
}

// ================= MAIN LOOP =================
void loop() {
    unsigned long now = millis();
    
    // IMU always runs
    if (now - last_imu_time >= IMU_INTERVAL) {
        last_imu_time = now;
        readIMU(now / 1000.0);
        ekf.predict(current_imu, now / 1000.0);
        
        // Update yaw from EKF
        nav_msgs__msg__Odometry odom = ekf.getOdom();
        double qz = odom.pose.pose.orientation.z;
        double qw = odom.pose.pose.orientation.w;
        robot_yaw = 2.0 * atan2(qz, qw);
    }
    
    // State Machine
    switch (current_state) {
        case STATE_STARTUP: {
            int elapsed = (now - startup_time) / 1000;
            static int last_printed = -1;
            if (elapsed != last_printed && elapsed <= STARTUP_DELAY_SEC) {
                Serial.printf("[COUNTDOWN] %d seconds remaining...\n", STARTUP_DELAY_SEC - elapsed);
                last_printed = elapsed;
            }
            if (elapsed >= STARTUP_DELAY_SEC) {
                Serial.println("\n[START] Beginning mapping mission!\n");
                current_state = STATE_SCANNING;
            }
            break;
        }
        
        case STATE_SCANNING: {
            QuasarPacket packet;
            performScan(packet);
            sendPacket(packet);
            
            Serial.println("[STATE] Transitioning to MOVING...\n");
            current_state = STATE_MOVING;
            break;
        }
        
        case STATE_MOVING: {
            moveForward(MOVE_DISTANCE_CM);
            
            Serial.println("[STATE] Transitioning to SCANNING...\n");
            delay(500); // Brief pause before scanning
            current_state = STATE_SCANNING;
            break;
        }
        
        case STATE_STOPPED: {
            motor.stop();
            break;
        }
    }
}
