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
const char* agent_ip = "10.57.0.30";
const int agent_port = 8888;
const int local_port = 8888;

// Servo (14-bit PWM)
#define SERVO_PIN 13
#define SERVO_FREQ 50
#define SERVO_RES 14
const int minDuty = 410;  
const int maxDuty = 1966;
const int stepDelay = 30; // Faster scan for reactive nav

// Ultrasonic (HC-SR04)
#define TRIG_PIN 5
#define ECHO_PIN 18
#define SOUND_SPEED 0.0343

// Encoder
#define ENCODER_PIN 34
const float CM_PER_GROOVE = 30.0 / 28.0;

// Navigation Parameters
const float OBSTACLE_THRESHOLD = 0.40;  // 40cm = too close!
const float SAFE_DISTANCE = 0.60;       // 60cm = comfortable
const float MOVE_DISTANCE_CM = 20.0;    // Move 20cm at a time
const int MOTOR_SPEED = 120;
const int TURN_SPEED = 100;
const int STARTUP_DELAY_SEC = 20;

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

// ================= GLOBALS =================
WiFiUDP udp;
MotorController motor;
EKF ekf;
Adafruit_MPU6050 mpu;

sensor_msgs__msg__Imu current_imu;
nav_msgs__msg__Odometry current_encoder_odom;

// Encoder
volatile long encoder_count = 0;

// Scan data (lightweight - only 5 zones)
float zone_distances[5]; // LEFT, FRONT-LEFT, FRONT, FRONT-RIGHT, RIGHT

// Position
float robot_x = 0.0;
float robot_y = 0.0;
float robot_yaw = 0.0;

// Mission Tracking
float total_distance_traveled = 0.0;  // In meters
const float MIN_TRAVEL_DISTANCE = 1.5; // Must travel 1.5m before considering "done"
const float RETURN_THRESHOLD = 0.50;   // Within 50cm of start = "returned home"
bool mission_complete = false;

// Timers
unsigned long last_imu_time = 0;
unsigned long startup_time = 0;
bool startup_complete = false;

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

// Single ultrasonic reading (internal use)
float readUltrasonicSingle() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    long duration = pulseIn(ECHO_PIN, HIGH, 30000);
    if (duration == 0) return 4.0; // No echo = far away
    
    float distanceM = (duration * SOUND_SPEED / 2.0) / 100.0;
    if (distanceM > 4.0) return 4.0;
    if (distanceM < 0.02) return 0.02;
    return distanceM;
}

// Averaged ultrasonic reading (10 samples, removes outliers)
float readUltrasonic() {
    const int NUM_SAMPLES = 10;
    float samples[NUM_SAMPLES];
    
    // Collect 10 samples
    for (int i = 0; i < NUM_SAMPLES; i++) {
        samples[i] = readUltrasonicSingle();
        delay(10); // Small delay between readings
    }
    
    // Sort samples (simple bubble sort for 10 elements)
    for (int i = 0; i < NUM_SAMPLES - 1; i++) {
        for (int j = 0; j < NUM_SAMPLES - i - 1; j++) {
            if (samples[j] > samples[j + 1]) {
                float temp = samples[j];
                samples[j] = samples[j + 1];
                samples[j + 1] = temp;
            }
        }
    }
    
    // Average middle 6 samples (discard 2 lowest and 2 highest outliers)
    float sum = 0;
    for (int i = 2; i < NUM_SAMPLES - 2; i++) {
        sum += samples[i];
    }
    return sum / 6.0;
}

void readIMU() {
    sensors_event_t a, g, temp;
    if (mpu.getEvent(&a, &g, &temp)) {
        current_imu.angular_velocity.z = g.gyro.z;
    }
}

// ================= UNIFIED SCANNING =================
// pattern: 90->0->180->90 (Slow)
// captures every 2 degrees
void performMissionScan(QuasarPacket& packet) {
    packet.scan_count = 181;
    // Initialize ranges with 0
    for(int i=0; i<181; i++) packet.ranges[i] = 0.0;
    
    Serial.println("[SCAN] 90 -> 0 (Positioning)");
    // 1. Center (90) to Left (0) - positioning (no read)
    for (int angle = 90; angle >= 0; angle-=2) {
        setServoAngle(angle);
        delay(stepDelay); 
    }

    Serial.println("[SCAN] 0 -> 180 (Capturing)");
    // 2. Left (0) to Right (180) - CAPTURE every 2 degrees
    for (int angle = 0; angle <= 180; angle+=2) {
        setServoAngle(angle);
        delay(stepDelay); // Wait for servo
        
        float dist = readUltrasonic();
        packet.ranges[angle] = dist; // Store at even index
        if (angle+1 <= 180) packet.ranges[angle+1] = dist; // Fill odd index for completeness
    }

    Serial.println("[SCAN] 180 -> 90 (Returning)");
    // 3. Right (180) to Center (90) - positioning
    for (int angle = 180; angle >= 90; angle-=2) {
        setServoAngle(angle);
        delay(stepDelay);
    }
    
    // Compute Navigation Zones from Scan Data (Average)
    // Zones: L(0-36), FL(36-72), F(72-108), FR(108-144), R(144-180)
    float sums[5] = {0,0,0,0,0};
    int counts[5] = {0,0,0,0,0};
    
    for (int i = 0; i <= 180; i+=2) {
        float val = packet.ranges[i];
        if (val <= 0.02 || val >= 4.0) continue; // Ignore valid/max
        
        int zone_idx = i / 36;
        if (zone_idx > 4) zone_idx = 4;
        
        sums[zone_idx] += val;
        counts[zone_idx]++;
    }
    
    for (int i = 0; i < 5; i++) {
        if (counts[i] > 0) zone_distances[i] = sums[i] / counts[i];
        else zone_distances[i] = 4.0; // Assume clear if no valid readings
    }
    
    Serial.printf("[ZONES] L:%.2f FL:%.2f F:%.2f FR:%.2f R:%.2f\n",
        zone_distances[0], zone_distances[1], zone_distances[2],
        zone_distances[3], zone_distances[4]);
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
}

// ================= MOVEMENT =================
void moveForward(float distance_cm) {
    int target_grooves = (int)(distance_cm / CM_PER_GROOVE);
    encoder_count = 0;
    
    motor.drive(MOTOR_SPEED, MOTOR_SPEED);
    
    while (encoder_count < target_grooves) {
        // Quick obstacle check while moving
        setServoAngle(90);
        float front_dist = readUltrasonic();
        if (front_dist < OBSTACLE_THRESHOLD) {
            Serial.println("[!] Obstacle detected while moving!");
            break;
        }
        delay(50);
    }
    
    motor.stop();
    
    float moved_cm = encoder_count * CM_PER_GROOVE;
    float moved_m = moved_cm / 100.0;
    robot_x += moved_m * cos(robot_yaw);
    robot_y += moved_m * sin(robot_yaw);
    total_distance_traveled += moved_m;
    
    Serial.printf("[POS] X:%.2f Y:%.2f Total:%.2fm\n", robot_x, robot_y, total_distance_traveled);
}

void turnLeft(int degrees) {
    Serial.printf("[TURN] Left %d degrees\n", degrees);
    int turn_time = degrees * 8; // Approximate: 8ms per degree
    motor.drive(-TURN_SPEED, TURN_SPEED);
    delay(turn_time);
    motor.stop();
    robot_yaw += radians(degrees);
}

void turnRight(int degrees) {
    Serial.printf("[TURN] Right %d degrees\n", degrees);
    int turn_time = degrees * 8;
    motor.drive(TURN_SPEED, -TURN_SPEED);
    delay(turn_time);
    motor.stop();
    robot_yaw -= radians(degrees);
}

// ================= LOOP CLOSURE CHECK =================
bool checkMissionComplete() {
    // Calculate distance from start (0,0)
    float dist_from_start = sqrt(robot_x * robot_x + robot_y * robot_y);
    
    // Mission complete if:
    // 1. Traveled at least MIN_TRAVEL_DISTANCE
    // 2. AND back near starting position
    if (total_distance_traveled > MIN_TRAVEL_DISTANCE && dist_from_start < RETURN_THRESHOLD) {
        return true;
    }
    return false;
}

// ================= BUG ALGORITHM =================
// Simple reactive navigation
void navigate() {
    // Check if mission is complete first
    if (checkMissionComplete()) {
        mission_complete = true;
        motor.stop();
        Serial.println("\n========================================");
        Serial.println("MISSION COMPLETE!");
        Serial.printf("Total distance traveled: %.2f meters\n", total_distance_traveled);
        Serial.printf("Final position: (%.2f, %.2f)\n", robot_x, robot_y);
        Serial.println("Robot has returned to start.");
        Serial.println("========================================\n");
        
        // Send final packet
        QuasarPacket packet;
        performMissionScan(packet);
        sendPacket(packet);
        return;
    }
    
    // Perform Unified Scan (Slow sweep, every 2 degrees)
    QuasarPacket packet;
    performMissionScan(packet);
    sendPacket(packet); // Send every scan so user sees map
    
    float left = zone_distances[0];
    float front_left = zone_distances[1];
    float front = zone_distances[2];
    float front_right = zone_distances[3];
    float right = zone_distances[4];
    
    // Decision Logic
    if (front > SAFE_DISTANCE && front_left > OBSTACLE_THRESHOLD && front_right > OBSTACLE_THRESHOLD) {
        // Path is clear - GO FORWARD
        Serial.println("[NAV] Clear ahead → Moving forward");
        moveForward(MOVE_DISTANCE_CM);
    }
    else if (front < OBSTACLE_THRESHOLD) {
        // Obstacle directly ahead - find best turn direction
        if (left > right) {
            Serial.println("[NAV] Blocked! → Turning LEFT");
            turnLeft(45);
        } else {
            Serial.println("[NAV] Blocked! → Turning RIGHT");
            turnRight(45);
        }
    }
    else if (front_left < OBSTACLE_THRESHOLD) {
        // Obstacle on front-left - slight right turn
        Serial.println("[NAV] Obstacle front-left → Adjusting RIGHT");
        turnRight(30);
    }
    else if (front_right < OBSTACLE_THRESHOLD) {
        // Obstacle on front-right - slight left turn
        Serial.println("[NAV] Obstacle front-right → Adjusting LEFT");
        turnLeft(30);
    }
    else {
        // Safe to proceed slowly
        Serial.println("[NAV] Cautious → Moving forward");
        moveForward(MOVE_DISTANCE_CM / 2);
    }
}

// ================= SETUP =================
void setup() {
    Serial.begin(115200);
    Serial.println("\n========================================");
    Serial.println("Lightweight SLAM Navigator");
    Serial.println("Using Bug Algorithm (Reactive)");
    Serial.println("========================================\n");
    
    Wire.begin(21, 22);

    // MPU6050
    if (!mpu.begin()) {
        Serial.println("[ERROR] MPU6050 not found!");
    } else {
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        Serial.println("[OK] MPU6050");
    }
    
    // Servo
    ledcAttach(SERVO_PIN, SERVO_FREQ, SERVO_RES);
    setServoAngle(90);
    Serial.println("[OK] Servo");
    
    // Ultrasonic
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    Serial.println("[OK] Ultrasonic");
    
    // Encoder
    pinMode(ENCODER_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderISR, RISING);
    Serial.println("[OK] Encoder");
    
    // EKF
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(6);
    ekf.init(millis()/1000.0, x0);
    
    // Motors
    motor.init();
    Serial.println("[OK] Motors");

    // WiFi
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("[WIFI] Connecting");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.printf("\n[WIFI] Connected: %s\n", WiFi.localIP().toString().c_str());
    udp.begin(local_port);
    
    // Startup delay
    Serial.printf("\n[STARTUP] %d second countdown...\n", STARTUP_DELAY_SEC);
    startup_time = millis();
}

// ================= MAIN LOOP =================
void loop() {
    // If mission complete, just blink onboard LED and do nothing
    if (mission_complete) {
        static unsigned long last_blink = 0;
        if (millis() - last_blink > 500) {
            digitalWrite(2, !digitalRead(2)); // Toggle onboard LED
            last_blink = millis();
        }
        return;
    }
    
    unsigned long now = millis();
    
    // Update IMU continuously
    if (now - last_imu_time >= 10) {
        last_imu_time = now;
        readIMU();
        ekf.predict(current_imu, now / 1000.0);
        
        nav_msgs__msg__Odometry odom = ekf.getOdom();
        robot_yaw = 2.0 * atan2(odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    }
    
    // Startup countdown
    if (!startup_complete) {
        int elapsed = (now - startup_time) / 1000;
        static int last_print = -1;
        if (elapsed != last_print && elapsed <= STARTUP_DELAY_SEC) {
            Serial.printf("[COUNTDOWN] %d...\n", STARTUP_DELAY_SEC - elapsed);
            last_print = elapsed;
        }
        if (elapsed >= STARTUP_DELAY_SEC) {
            startup_complete = true;
            Serial.println("\n[GO!] Starting autonomous navigation!\n");
        }
        return;
    }
    
    // Main navigation loop
    navigate();
    
    delay(100); // Small delay between navigation cycles
}
