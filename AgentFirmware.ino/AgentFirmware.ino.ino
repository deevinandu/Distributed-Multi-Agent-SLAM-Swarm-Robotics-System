#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUDP.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include "ekf.h"
#include "motor_control.h"

// ================= CONFIGURATION =================
#define WIFI_SSID "Gia"
#define WIFI_PASSWORD "e7jt92zp"
#define AGENT_ID 1 

// Network
const char* agent_ip = "172.25.35.255"; // Broadcast address for your new subnet
const int agent_port = 8888;
const int local_port = 8888;

// Servo (14-bit PWM)
#define SERVO_PIN 13
#define SERVO_FREQ 50
#define SERVO_RES 14
const int minDuty = 410;  
const int maxDuty = 1966;
const int stepDelay = 30; // 30ms per degree = slow methodical scan

// Ultrasonic (HC-SR04)
#define TRIG_PIN 5
#define ECHO_PIN 18
#define SOUND_SPEED 0.0343

// Encoder
#define ENCODER_PIN 34
const float CM_PER_GROOVE = 30.0 / 28.0;

// Navigation Parameters
const float OBSTACLE_THRESHOLD = 0.40;  // 40cm
const float SAFE_DISTANCE = 0.60;       // 60cm
const float MOVE_DISTANCE_CM = 20.0;    
const int MOTOR_SPEED = 190;            // High torque
const int TURN_SPEED = 180;
const int STARTUP_DELAY_SEC = 5;

// ================= PROTOCOL =================
struct __attribute__((packed)) QuasarPacket {
    char magic[4];
    uint8_t agent_id;
    float odom_x;
    float odom_y;
    float odom_yaw;
    int32_t encoder_total;
    uint16_t scan_count;
    float ranges[181];
};

// ================= GLOBALS =================
WiFiUDP udp;
MotorController motor;
Adafruit_MPU6050 mpu;
EKF ekf;

// State
sensor_msgs__msg__Imu current_imu;
volatile long encoder_count = 0;
volatile int32_t global_encoder_total = 0;
float zone_distances[5] = {4.0, 4.0, 4.0, 4.0, 4.0};

// Position & Mission
float robot_x = 0.0;
float robot_y = 0.0;
float robot_yaw = 0.0;
float total_distance_traveled = 0.0;
const float MIN_TRAVEL_DISTANCE = 1.6; // must travel 1.6m before loop closure
const float RETURN_THRESHOLD = 0.50;   // 50cm from start
bool mission_complete = false;

// Calibration
float gyroZ_offset = 0.0;
float accX_offset = 0.0;
unsigned long last_imu_time = 0;
unsigned long startup_time = 0;

// ================= INTERRUPTS =================
void IRAM_ATTR encoderISR() {
    encoder_count++;
    global_encoder_total++;
}

// ================= SENSORS =================
void setServoAngle(int angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    int duty = map(angle, 0, 180, minDuty, maxDuty);
    ledcWrite(SERVO_PIN, duty);
}

float readUltrasonicSingle() {
    // 1. Force a "Clean State" for the sonar trigger
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(5);
    
    // 2. High-Precision Triggering
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(15); // Increased to 15us for better hardware sensitivity 
    digitalWrite(TRIG_PIN, LOW);
    
    // 3. CRITICAL SECTION: Stop background tasks/WiFi from interrupting the timer
    noInterrupts();
    long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout (approx 5 meters)
    interrupts();
    
    // 4. Handle failure (TIMEOUT)
    if (duration == 0) {
        // [DEBUG] If you see this often, it's a Power/Wiring issue
        return 4.0; 
    }
    
    // Distance = Time * Speed / 2 (result in METERS)
    float dist = (duration * SOUND_SPEED / 2.0) / 100.0;
    
    // Keep it within sensor logic bounds [2cm - 400cm]
    return (dist > 4.0) ? 4.0 : ((dist < 0.02) ? 0.02 : dist);
}

float readUltrasonic() {
    const int N = 10; // Restored for high accuracy
    float s[N];
    for(int i=0; i<N; i++) { 
        s[i] = readUltrasonicSingle(); 
        delay(5); 
    }
    // Outlier-removal filter
    for(int i=0; i<N-1; i++) {
        for(int j=0; j<N-i-1; j++) {
            if(s[j] > s[j+1]) { float t=s[j]; s[j]=s[j+1]; s[j+1]=t; }
        }
    }
    float sum = 0;
    for(int i=2; i<N-2; i++) sum += s[i];
    return sum / 6.0;
}

void readIMU() {
    sensors_event_t a, g, temp;
    if (mpu.getEvent(&a, &g, &temp)) {
        current_imu.angular_velocity.z = g.gyro.z - gyroZ_offset;
        // Orientation correction: New Forward is now +X
        current_imu.linear_acceleration.x = (a.acceleration.x - accX_offset);
    }
}

// ================= SCANNING =================
void performMissionScan(QuasarPacket& packet) {
    packet.scan_count = 181;
    for(int i=0; i<181; i++) packet.ranges[i] = 0.0;
    
    // Pattern: 90 -> 0 (5 Seconds)
    Serial.println("[SCAN] Sweeping 90 -> 0 (5s)...");
    for(int a=90; a>=0; a--) { 
        setServoAngle(a); 
        delay(55); // 5000ms / 90 steps
        if (a % 45 == 0) Serial.printf("  Current: %d\n", a);
    }
    
    // Pattern: 0 -> 180 (10s + Capture)
    Serial.println("[SCAN] Capturing 0 to 180...");
    for(int a=0; a<=180; a++) {
        setServoAngle(a); 
        delay(80); // Settling delay
        
        if (a % 2 == 0) {
            float d = readUltrasonic();
            packet.ranges[a] = d;
            if (a+1 <= 180) packet.ranges[a+1] = d;
            Serial.printf("[SCAN] %d: %.2fm\n", a, d);
        }
    }

    Serial.println("[SCAN] Returning 180 -> 90 (5s)...");
    for(int a=180; a>=90; a--) { 
        setServoAngle(a); 
        delay(55); 
    }

    // Zone derivation
    float sums[5]={0}, counts[5]={0};
    for(int i=0; i<=180; i+=2) {
        int z = i/36; if(z>4) z=4;
        sums[z] += packet.ranges[i]; counts[z]++;
    }
    for(int i=0; i<5; i++) zone_distances[i] = (counts[i]>0) ? (sums[i]/counts[i]) : 4.0;
}

// ================= MISSION LOGIC =================
void sendPacket(QuasarPacket& packet) {
    memcpy(packet.magic, "QSRL", 4);
    packet.agent_id = AGENT_ID;
    packet.odom_x = robot_x; 
    packet.odom_y = robot_y; 
    packet.odom_yaw = robot_yaw;
    packet.encoder_total = global_encoder_total;
    
    IPAddress bIp; bIp.fromString(agent_ip);
    udp.beginPacket(bIp, agent_port);
    udp.write((const uint8_t*)&packet, sizeof(packet));
    udp.endPacket();
}

bool checkMissionComplete() {
    float distSq = robot_x*robot_x + robot_y*robot_y;
    return (total_distance_traveled > MIN_TRAVEL_DISTANCE && sqrt(distSq) < RETURN_THRESHOLD);
}

// ================= MOVEMENT =================
void moveForward(float dist_cm) {
    if (CM_PER_GROOVE <= 0) return; // Safety check
    int target = (int)(dist_cm / CM_PER_GROOVE);
    
    // Atomic Reset
    noInterrupts();
    encoder_count = 0;
    interrupts();
    
    motor.drive(MOTOR_SPEED, MOTOR_SPEED);
    Serial.printf("[MOVE] Forward %d grooves (%.1f cm)...\n", target, dist_cm);
    
    unsigned long start_time = millis();
    while(encoder_count < target && (millis() - start_time) < 10000) {
        // Use single reading for low latency during movement
        float d = readUltrasonicSingle();
        
        if (encoder_count % 5 == 0) {
            Serial.printf("  Progress: %d/%d | Dist: %.2f m\n", encoder_count, target, d);
        }
        
        if(d < OBSTACLE_THRESHOLD) {
            Serial.println("[HALT] Obstacle!");
            break;
        }
        delay(30); 
    }
    motor.stop();
    float m = (encoder_count * CM_PER_GROOVE) / 100.0;
    robot_x += m * cos(robot_yaw); robot_y += m * sin(robot_yaw);
    total_distance_traveled += m;
}

void turn(int degrees, bool left) {
    float start_yaw = robot_yaw;
    float target_rad = radians(abs(degrees));
    motor.drive(left ? -TURN_SPEED : TURN_SPEED, left ? TURN_SPEED : -TURN_SPEED);
    
    unsigned long start_time = millis();
    while(abs(robot_yaw - start_yaw) < target_rad && (millis() - start_time) < 5000) {
        readIMU();
        // Force EKF update during turn
        sensor_msgs__msg__Imu msg; 
        msg.angular_velocity.z = current_imu.angular_velocity.z; 
        msg.linear_acceleration.x = current_imu.linear_acceleration.x;
        ekf.predict(msg, millis()/1000.0);
        nav_msgs__msg__Odometry o = ekf.getOdom();
        
        // Correct Yaw Extraction
        robot_yaw = 2.0 * atan2(o.pose.pose.orientation.z, o.pose.pose.orientation.w);
        delay(10);
    }
    motor.stop();
}

void navigate() {
    if (checkMissionComplete()) {
        mission_complete = true; Serial.println("MISSION COMPLETE!");
        motor.stop(); return;
    }
    QuasarPacket p; performMissionScan(p); sendPacket(p);
    
    if(zone_distances[2] > SAFE_DISTANCE && zone_distances[1] > OBSTACLE_THRESHOLD && zone_distances[3] > OBSTACLE_THRESHOLD) {
        moveForward(MOVE_DISTANCE_CM);
    } else if(zone_distances[2] < OBSTACLE_THRESHOLD) {
        if(zone_distances[0] > zone_distances[4]) turn(45, true); else turn(45, false);
    } else if(zone_distances[1] < OBSTACLE_THRESHOLD) turn(30, false);
    else if(zone_distances[3] < OBSTACLE_THRESHOLD) turn(30, true);
    else moveForward(MOVE_DISTANCE_CM / 2.0);
}

// ================= CORE =================
void setup() {
    Serial.begin(115200); 
    delay(1000); // Give Serial monitor time to connect
    
    Serial.println("\n[BOOT] System Starting...");
    
    Serial.println("[BOOT] Initializing I2C (Pins 21, 22)...");
    Wire.begin(21, 22);
    Wire.setClock(100000); // Standard speed to be safe
    
    Serial.println("[BOOT] Initializing MPU6050...");
    if (!mpu.begin()) {
        Serial.println("[ERROR] MPU6050 not found! Hanging for safety.");
        while(1) { delay(1000); }
    }
    Serial.println("[BOOT] MPU6050 OK. Calibrating...");
    
    // Calibrate GYRO and ACCEL
    float gz_sum = 0, ax_sum = 0; int samples = 50;
    for(int i=0; i<samples; i++) {
        sensors_event_t a, g, t; 
        if (mpu.getEvent(&a,&g,&t)) {
            gz_sum += g.gyro.z; 
            ax_sum += a.acceleration.x; 
        }
        delay(20);
    }
    gyroZ_offset = gz_sum / samples; 
    accX_offset = ax_sum / samples;
    Serial.println("[BOOT] Calibration Done.");
    
    Serial.println("[BOOT] Initializing Encoder...");
    pinMode(ENCODER_PIN, INPUT); 
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderISR, RISING);
    
    Serial.println("[BOOT] Initializing Motors...");
    motor.init();
    
    Serial.println("[BOOT] Initializing Servo (RE-ATTEMPT)...");
    // Ensure servo is attached AFTER motors to avoid timer conflict
    ledcAttach(SERVO_PIN, SERVO_FREQ, SERVO_RES); 
    
    // WIGGLE TEST
    Serial.println("[BOOT] Servo Wiggle Test...");
    setServoAngle(45); delay(500);
    setServoAngle(135); delay(500);
    setServoAngle(90); delay(500);
    Serial.println("[BOOT] Servo Wiggle OK.");

    Serial.printf("[WIFI] Connecting to %s...\n", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    unsigned long start_wifi = millis();
    while(WiFi.status() != WL_CONNECTED && millis() - start_wifi < 10000) {
        delay(500);
        Serial.print(".");
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\n[WIFI] Connected! IP: %s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("\n[WIFI] FAILED to connect (Timeout 10s). Proceeding offline.");
    }
    
    udp.begin(local_port);
    
    Serial.println("[BOOT] Initializing EKF...");
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(6); 
    ekf.init(millis()/1000.0, x0);
    
    Serial.println("[BOOT] SETUP COMPLETE. Robot will move in 5 seconds.");
    startup_time = millis();
}

void loop() {
    if(mission_complete) {
        digitalWrite(2, (millis()/500)%2); return;
    }
    unsigned long now = millis();
    readIMU();
    sensor_msgs__msg__Imu msg; msg.angular_velocity.z = current_imu.angular_velocity.z; 
    msg.linear_acceleration.x = current_imu.linear_acceleration.x;
    ekf.predict(msg, now/1000.0);
    nav_msgs__msg__Odometry o = ekf.getOdom();
    
    // Correct Yaw Extraction (2 * atan2(z, w))
    robot_yaw = 2.0 * atan2(o.pose.pose.orientation.z, o.pose.pose.orientation.w); 
    
    if(now - startup_time > STARTUP_DELAY_SEC * 1000) {
        navigate();
    }
}
