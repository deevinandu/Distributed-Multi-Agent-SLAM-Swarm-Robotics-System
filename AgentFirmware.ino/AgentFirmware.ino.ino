#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUDP.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include "ekf.h"
#include "motor_control.h"
#include <esp_now.h> // New for Swarm Networking

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
const float OBSTACLE_THRESHOLD = 0.25;  // 25cm
const float SAFE_DISTANCE = 0.50;       // 50cm
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
    uint32_t v2v_count; // New: Monitor wireless transfers
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
float bucket_distances[18]; // 18 buckets of 10 degrees each

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

// V2V Swarm Communication
volatile float last_v2v_distance_cm = 400.0;
unsigned long last_v2v_time = 0;
volatile uint32_t v2v_packet_received_total = 0;

typedef struct struct_message {
    float distance; // Now receiving in CM
} struct_message;

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
    struct_message* received = (struct_message*)incomingData;
    last_v2v_distance_cm = received->distance;
    last_v2v_time = millis();
    v2v_packet_received_total++;
}

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
    // V2V LINK: We now read the wireless value (CM) and convert to Meters for Navigation
    if (millis() - last_v2v_time > 1000) {
        return 4.0; 
    }
    return last_v2v_distance_cm / 100.0; // Return in METERS for EKF/Logic
}

float readUltrasonic() {
    // Log the CM reading to Serial for the user
    float m = readUltrasonicSingle();
    return m;
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
            float d = readUltrasonic(); // d is in Meters
            packet.ranges[a] = d;
            if (a+1 <= 180) packet.ranges[a+1] = d;
            Serial.printf("[SCAN] %d: %.0f cm\n", a, d * 100.0);
        }
    }

    Serial.println("[SCAN] Returning 180 -> 90 (5s)...");
    for(int a=180; a>=90; a--) { 
        setServoAngle(a); 
        delay(55); 
    }

    // Bucket derivation (18 buckets, 10 degrees each)
    for(int b=0; b<18; b++) {
        float sum = 0;
        int count = 0;
        for(int i=b*10; i<(b+1)*10 && i<=180; i+=2) {
            if(packet.ranges[i] > 0.01) {
                sum += packet.ranges[i];
                count++;
            }
        }
        bucket_distances[b] = (count > 0) ? (sum / count) : 4.0;
    }
}

// ================= MISSION LOGIC =================
void sendPacket(QuasarPacket& packet) {
    memcpy(packet.magic, "QSRL", 4);
    packet.agent_id = AGENT_ID;
    packet.odom_x = robot_x; 
    packet.odom_y = robot_y; 
    packet.odom_yaw = robot_yaw;
    packet.encoder_total = global_encoder_total;
    packet.v2v_count = v2v_packet_received_total; // Send swarm status to laptop
    
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
void moveForwardReactive() {
    // Atomic Reset (Not strictly needed for distance, but good for Odom)
    noInterrupts();
    encoder_count = 0;
    interrupts();
    
    motor.drive(MOTOR_SPEED, MOTOR_SPEED);
    Serial.println("[MOVE] Reactive Forward until 25cm...");
    
    unsigned long start_time = millis();
    while((millis() - start_time) < 15000) { // 15s safety timeout
        float d = readUltrasonicSingle();
        
        if (encoder_count % 5 == 0) {
            Serial.printf("  Enc: %d | Dist: %.2f m\n", encoder_count, d);
        }
        
        if(d < OBSTACLE_THRESHOLD) {
            Serial.println("[HALT] 25cm Trigger Reached!");
            break;
        }
        delay(30); 
    }
    motor.stop();
    
    // Update Global Pose
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

    // 1. Initial State: If obstructed, scan and turn
    float d_now = readUltrasonicSingle();
    if (d_now <= OBSTACLE_THRESHOLD) {
        Serial.println("[NAV] Path blocked. Scanning...");
        
        QuasarPacket p; performMissionScan(p); sendPacket(p);
        
        // Calculate Left and Right average distances
        // Left = buckets 0-8 (0-90 degrees), Right = buckets 9-17 (90-180 degrees)
        float left_avg = 0, right_avg = 0;
        for(int i=0; i<9; i++) left_avg += bucket_distances[i];
        for(int i=9; i<18; i++) right_avg += bucket_distances[i];
        left_avg /= 9.0;
        right_avg /= 9.0;
        
        Serial.printf("[NAV] Left: %.2fm | Right: %.2fm\n", left_avg, right_avg);
        
        // LEFT-FIRST Priority: If left has >60cm clearance, turn left
        if(left_avg > 0.60) {
            Serial.println("[NAV] Turning LEFT 15 deg.");
            turn(15, true); // true = left
        } else if(right_avg > 0.60) {
            Serial.println("[NAV] Turning RIGHT 15 deg.");
            turn(15, false); // false = right
        } else {
            // Both sides blocked -> Turn 90 degrees to the more open side
            Serial.println("[NAV] Tight spot! Full turn.");
            turn(90, left_avg > right_avg);
        }
    } else {
        // 2. Path is clear -> Drive until 25cm
        moveForwardReactive();
        
        // 3. Scan to map the wall we found
        Serial.println("[NAV] At 25cm. Mapping wall...");
        QuasarPacket p; performMissionScan(p); sendPacket(p);
    }
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
        
        // --- Init ESP-NOW for Swarm V2V ---
        if (esp_now_init() == ESP_OK) {
            Serial.println("[SWARM] V2V Link Initialized.");
            esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
        }
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
