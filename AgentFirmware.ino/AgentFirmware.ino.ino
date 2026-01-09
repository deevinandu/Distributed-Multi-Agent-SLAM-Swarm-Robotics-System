#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUDP.h>

// IMPORTANT: Ensure "ekf.h", "ekf.cpp", "motor_control.h", "motor_control.cpp"
// are in the SAME folder as this .ino file!
#include "ekf.h"
#include "motor_control.h"

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Constants
#define WIFI_SSID "ONEPLUS_7T"
#define WIFI_PASSWORD "dvhy03555"
#define AGENT_ID 1 

// --- NETWORK CONFIG ---
const char* agent_ip = "10.132.45.255";
const int agent_port = 8888;
const int local_port = 8888;

// --- SERVO CONFIG (14-bit resolution for precision) ---
#define SERVO_PIN 13
#define SERVO_FREQ 50       // 50Hz for servos
#define SERVO_RES 14        // 14-bit resolution (0-16383)
// 0 deg   = 0.5ms pulse -> (0.5 / 20) * 16383 ≈ 410
// 180 deg = 2.4ms pulse -> (2.4 / 20) * 16383 ≈ 1966
const int minDuty = 410;  
const int maxDuty = 1966;
const int stepDelay = 20; // Servo step delay (ms)

// --- ULTRASONIC SENSOR (HC-SR04) ---
#define TRIG_PIN 5
#define ECHO_PIN 18
#define SOUND_SPEED 0.0343  // cm/microsecond

// --- QUASAR-LITE PROTOCOL ---
struct __attribute__((packed)) QuasarPacket {
    char magic[4];       // "QSRL"
    uint8_t agent_id;
    float odom_x;
    float odom_y;
    float odom_yaw;
    uint16_t scan_count; // 181
    float ranges[181];
};

struct __attribute__((packed)) CommandPacket {
    char magic[4];    // "CMD1"
    float linear_x;
    float angular_z;
};

// Global Objects
WiFiUDP udp;
MotorController motor;
EKF ekf;

// Sensors
Adafruit_MPU6050 mpu;

// Data Holders
sensor_msgs__msg__Imu current_imu;
nav_msgs__msg__Odometry current_encoder_odom;

// Timers
unsigned long last_imu_time = 0;
unsigned long last_pub_time = 0;
unsigned long last_cmd_time = 0;
const long CMD_TIMEOUT = 500; 
const long IMU_INTERVAL = 10; // 100Hz
const long PUB_INTERVAL = 5000; // 5 Seconds (increased for ultrasonic sweep)

// --- SERVO HELPER (14-bit) ---
void setServoAngle(int angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    
    int dutyCycle = map(angle, 0, 180, minDuty, maxDuty);
    ledcWrite(SERVO_PIN, dutyCycle);
}

// --- ULTRASONIC HELPER ---
float readUltrasonic() {
    // Clear trigger
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    
    // Send 10us pulse
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    // Measure echo (timeout after 30ms = ~5m max range)
    long duration = pulseIn(ECHO_PIN, HIGH, 30000);
    
    if (duration == 0) {
        return 0.0; // No echo (out of range)
    }
    
    // Calculate distance in METERS
    float distanceCm = duration * SOUND_SPEED / 2.0;
    float distanceM = distanceCm / 100.0;
    
    // Clamp to reasonable range (0.02m - 4m)
    if (distanceM > 4.0) distanceM = 0.0;
    if (distanceM < 0.02) distanceM = 0.0;
    
    return distanceM;
}

void readIMU(double t) {
    sensors_event_t a, g, temp;
    if (mpu.getEvent(&a, &g, &temp)) {
        current_imu.header.stamp.sec = (int)t;
        current_imu.header.stamp.nanosec = (uint32_t)((t - (int)t) * 1e9);
        current_imu.angular_velocity.z = g.gyro.z; 
        current_imu.linear_acceleration.z = a.acceleration.z;
    }
}

void readEncoders(double t) {
    current_encoder_odom.twist.twist.linear.x = 0.0; 
    current_encoder_odom.twist.twist.angular.z = 0.0; 
}

// REAL ULTRASONIC SCAN
void performScan(QuasarPacket& packet) {
    packet.scan_count = 181;
    
    Serial.println("Starting scan...");
    
    // Sweep from 0 to 180 degrees
    for (int angle = 0; angle <= 180; angle++) {
        setServoAngle(angle);
        delay(stepDelay); // Wait for servo to settle
        
        // Read REAL ultrasonic distance
        float distance = readUltrasonic();
        packet.ranges[angle] = distance;
        
        // Debug output every 30 degrees
        if (angle % 30 == 0) {
            Serial.printf("Angle %d: %.2f m\n", angle, distance);
        }
    }
    
    // Return to home position
    setServoAngle(0);
    Serial.println("Scan complete!");
}

void setup() {
    Serial.begin(115200);
    Wire.begin(21, 22);

    // Init MPU
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
    } else {
        Serial.println("MPU6050 initialized");
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    }
    
    // Init Servo (14-bit LEDC)
    if(!ledcAttach(SERVO_PIN, SERVO_FREQ, SERVO_RES)) {
        Serial.println("LEDC Attach FAILED!");
    } else {
        setServoAngle(0);
        Serial.println("Servo initialized");
    }
    
    // Init Ultrasonic
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    Serial.println("Ultrasonic initialized");

    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(6); 
    ekf.init(millis()/1000.0, x0);

    motor.init();

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi Connected");
    Serial.println(WiFi.localIP());

    udp.begin(local_port);
}

void loop() {
    unsigned long current_millis = millis();
    double now_sec = current_millis / 1000.0;

    if (current_millis - last_cmd_time > CMD_TIMEOUT) {
        motor.stop();
    }

    // Receive Commands
    int packetSize = udp.parsePacket();
    if (packetSize >= sizeof(CommandPacket)) {
        CommandPacket cmd;
        udp.read((char*)&cmd, sizeof(cmd)); 

        if (strncmp(cmd.magic, "CMD1", 4) == 0) {
            last_cmd_time = current_millis;
            float linear_x = cmd.linear_x;
            float angular_z = cmd.angular_z;
            
            float left_v = linear_x - angular_z * 0.15; 
            float right_v = linear_x + angular_z * 0.15;
            int left_pwm = (int)(left_v * 510);   
            int right_pwm = (int)(right_v * 510);
            motor.drive(left_pwm, right_pwm);
        }
    }

    // IMU Loop
    if (current_millis - last_imu_time >= IMU_INTERVAL) {
        last_imu_time = current_millis;
        readIMU(now_sec);
        ekf.predict(current_imu, now_sec);
    }

    // Publish Loop
    if (current_millis - last_pub_time >= PUB_INTERVAL) {
        last_pub_time = current_millis;

        readEncoders(now_sec);
        ekf.update(current_encoder_odom); 
        
        QuasarPacket packet;
        memcpy(packet.magic, "QSRL", 4);
        packet.agent_id = AGENT_ID;
        
        nav_msgs__msg__Odometry odom = ekf.getOdom();
        packet.odom_x = odom.pose.pose.position.x;
        packet.odom_y = odom.pose.pose.position.y;
        
        double qz = odom.pose.pose.orientation.z;
        double qw = odom.pose.pose.orientation.w;
        packet.odom_yaw = (float)(2.0 * atan2(qz, qw));

        performScan(packet);

        // BROADCAST UDP
        IPAddress broadcastIp;
        if(broadcastIp.fromString(agent_ip)) {
             udp.beginPacket(broadcastIp, agent_port);
        } else {
             udp.beginPacket("10.132.45.255", agent_port);
        }
        
        udp.write((const uint8_t*)&packet, sizeof(packet));
        udp.endPacket();
        
        Serial.println("Packet Sent");
    }
}
