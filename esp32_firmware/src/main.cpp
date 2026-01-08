#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUDP.h>

#include "ekf.h"
#include "motor_control.h"

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <VL53L0X.h>
#include <ESP32Servo.h>

// Constants
#define WIFI_SSID "SSID"
#define WIFI_PASSWORD "PASSWORD"
#define AGENT_ID 1 // Integer ID for binary protocol

// PC IP Address
const char* agent_ip = "192.168.1.100"; 
const int agent_port = 8888;
const int local_port = 8888;

// Quasar-Lite Protocol Definitions
// Packed to ensure no padding bytes, matching Python 'struct' alignment
struct __attribute__((packed)) QuasarPacket {
    char magic[4];       // "QSRL"
    uint8_t agent_id;
    float odom_x;
    float odom_y;
    float odom_yaw;
    uint16_t scan_count; // Fixed 181
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
VL53L0X sensor;
Servo servo;

// Data Holders
sensor_msgs__msg__Imu current_imu;
nav_msgs__msg__Odometry current_encoder_odom;

// Timers
unsigned long last_imu_time = 0;
unsigned long last_pub_time = 0;
unsigned long last_cmd_time = 0;
const long CMD_TIMEOUT = 500; // ms
const long IMU_INTERVAL = 10; // 100Hz
const long PUB_INTERVAL = 2000; // 0.5Hz (Every 2 seconds for scan)

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

// Blocking Scan -> Fills the packet logic directly
void performScan(QuasarPacket& packet) {
    packet.scan_count = 181;
    
    // Blocking Sweep 0 to 180
    for (int pos = 0; pos <= 180; pos++) {
        servo.write(pos);
        delay(15); 
        
        uint16_t range_mm = sensor.readRangeSingleMillimeters();
        if (sensor.timeoutOccurred()) { 
             packet.ranges[pos] = 0.0;
        } else {
             float range_m = range_mm / 1000.0;
             if (range_m > 2.0) range_m = 0.0;
             packet.ranges[pos] = range_m;
        }
    }
    servo.write(0); 
}

void setup() {
    Serial.begin(115200);
    Wire.begin(21, 22);

    // Init Sensors
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) { delay(10); }
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    sensor.setTimeout(500);
    if (!sensor.init()) {
        Serial.println("Failed to detect and initialize VL53L0X!");
        while (1) {}
    }

    servo.attach(13);
    servo.write(0);

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

    // 1. Safety Timeout
    if (current_millis - last_cmd_time > CMD_TIMEOUT) {
        motor.stop();
    }

    // 2. Receive Commands (Binary)
    int packetSize = udp.parsePacket();
    if (packetSize >= sizeof(CommandPacket)) {
        CommandPacket cmd;
        udp.read((char*)&cmd, sizeof(cmd)); // Read directly into struct

        // Validate Magic
        if (strncmp(cmd.magic, "CMD1", 4) == 0) {
            last_cmd_time = current_millis;
            
            float linear_x = cmd.linear_x;
            float angular_z = cmd.angular_z;
            
            // Drive Motors
            float left_v = linear_x - angular_z * 0.15; 
            float right_v = linear_x + angular_z * 0.15;
            int left_pwm = (int)(left_v * 510);   
            int right_pwm = (int)(right_v * 510);
            motor.drive(left_pwm, right_pwm);
        }
    }

    // 3. IMU Update (100Hz)
    if (current_millis - last_imu_time >= IMU_INTERVAL) {
        last_imu_time = current_millis;
        readIMU(now_sec);
        ekf.predict(current_imu, now_sec);
    }

    // 4. Publish Loop (0.5Hz - Wait for scan)
    if (current_millis - last_pub_time >= PUB_INTERVAL) {
        last_pub_time = current_millis;

        readEncoders(now_sec);
        ekf.update(current_encoder_odom); 
        
        // Prepare Packet
        QuasarPacket packet;
        memcpy(packet.magic, "QSRL", 4);
        packet.agent_id = AGENT_ID;
        
        nav_msgs__msg__Odometry odom = ekf.getOdom();
        packet.odom_x = odom.pose.pose.position.x;
        packet.odom_y = odom.pose.pose.position.y;
        
        // Yaw from Quaternion
        double qz = odom.pose.pose.orientation.z;
        double qw = odom.pose.pose.orientation.w;
        packet.odom_yaw = (float)(2.0 * atan2(qz, qw));

        // Blocking Scan Fill
        performScan(packet);

        // Send Raw Binary
        udp.beginPacket(agent_ip, agent_port);
        udp.write((const uint8_t*)&packet, sizeof(packet));
        udp.endPacket();
    }
}
