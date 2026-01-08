#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <sensor_msgs/msg/imu.h>
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
#define AGENT_ID "agent_1"

// PC IP Address for Agent Connection
IPAddress agent_ip(192, 168, 1, 100); 

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Global Objects
rcl_publisher_t odom_pub;
rcl_publisher_t scan_pub;
rcl_subscription_t cmd_sub;

nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__LaserScan scan_msg;
float scan_ranges[360]; 
geometry_msgs__msg__twist__Twist cmd_msg;

MotorController motor;
unsigned long last_cmd_time = 0;
const long CMD_TIMEOUT = 500; // ms

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

rcl_timer_t imu_timer;
rcl_timer_t pub_timer;

EKF ekf;

sensor_msgs__msg__Imu current_imu;
nav_msgs__msg__Odometry current_encoder_odom;

// Hardware Objects
Adafruit_MPU6050 mpu;
VL53L0X sensor;
Servo servo;

void error_loop(){
  while(1){
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
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

// Stub: Generates synthetic Encoder data (10Hz)
// NOTE: Still valid until real encoders are added
void readEncoders(double t) {
    current_encoder_odom.twist.twist.linear.x = 0.0; // Assume stopped if not moving
    current_encoder_odom.twist.twist.angular.z = 0.0; 
}

// Blocking Servo Sweep and Scan
void readLiDAR(double t) {
    scan_msg.header.stamp.sec = (int)t;
    scan_msg.header.stamp.nanosec = (uint32_t)((t - (int)t) * 1e9);
    
    // Servo Scan Params (180 degrees)
    scan_msg.angle_min = -1.57; // -90 deg
    scan_msg.angle_max = 1.57;  // +90 deg
    scan_msg.angle_increment = 3.14 / 180.0; // 1 deg step
    scan_msg.range_min = 0.05;
    scan_msg.range_max = 2.0; // VL53L0X limit approx 2m
    
    // Blocking Sweep
    int data_idx = 0;
    for (int pos = 0; pos <= 180; pos++) {
        servo.write(pos);
        delay(15); // Wait for servo to move
        
        uint16_t range_mm = sensor.readRangeSingleMillimeters();
        if (sensor.timeoutOccurred()) { 
             scan_msg.ranges.data[data_idx] = 0.0;
        } else {
             // Convert to meters
             float range_m = range_mm / 1000.0;
             if (range_m > 2.0) range_m = 0.0; // Filter invalid
             scan_msg.ranges.data[data_idx] = range_m;
        }
        data_idx++;
        if (data_idx >= 360) break; // Safety
    }
    
    // Reset servo slowly or quickly? Quickly for now, to be ready for next logic (or just stay there)
    // Actually, maybe better to sweep back? For now, we jump back to 0 next time.
    servo.write(0); 
}

void cmd_vel_callback(const void * msin) {
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msin;
    last_cmd_time = millis();

    float linear_x = msg->linear.x;
    float angular_z = msg->angular.z;

    float left_v = linear_x - angular_z * 0.15; 
    float right_v = linear_x + angular_z * 0.15;

    int left_pwm = (int)(left_v * 510);   
    int right_pwm = (int)(right_v * 510);

    motor.drive(left_pwm, right_pwm);
}

void imu_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    RCLC_UNUSED(timer);

    double now_sec = millis() / 1000.0;
    
    readIMU(now_sec);
    ekf.predict(current_imu, now_sec);
}

void pub_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    RCLC_UNUSED(timer);

    double now_sec = millis() / 1000.0;

    readEncoders(now_sec);
    
    // NOTE: This IS blocking!
    readLiDAR(now_sec);

    ekf.update(current_encoder_odom);
    odom_msg = ekf.getOdom();
    
    odom_msg.header.stamp.sec = (int)now_sec;
    odom_msg.header.stamp.nanosec = (uint32_t)((now_sec - (int)now_sec) * 1e9);

    RCSOFTCHECK(rcl_publish(&odom_pub, &odom_msg, NULL));
    RCSOFTCHECK(rcl_publish(&scan_pub, &scan_msg, NULL));
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
    servo.write(0); // Home position

    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(6); 
    ekf.init(millis()/1000.0, x0);

    scan_msg.ranges.data = scan_ranges;
    scan_msg.ranges.size = 181; // 0 to 180 degrees
    scan_msg.ranges.capacity = 360;
    
    static char scan_frame_id[] = "laser_link";
    scan_msg.header.frame_id.data = scan_frame_id;
    scan_msg.header.frame_id.size = strlen(scan_frame_id);
    scan_msg.header.frame_id.capacity = sizeof(scan_frame_id);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    char ssid[] = WIFI_SSID;
    char psk[] = WIFI_PASSWORD;
    size_t agent_port = 8888;
    set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);
    delay(2000);

    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "agent_controller", "", &support));

    RCCHECK(rclc_publisher_init_default(
        &odom_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "/" AGENT_ID "/odom"));

    RCCHECK(rclc_publisher_init_default(
        &scan_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
        "/" AGENT_ID "/scan"));

    RCCHECK(rclc_subscription_init_default(
        &cmd_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/" AGENT_ID "/cmd_vel"));

    RCCHECK(rclc_timer_init_default(
        &imu_timer,
        &support,
        RCL_MS_TO_NS(10), // 100Hz
        imu_timer_callback));

    // Increase pub timer to 2s to allow for blocking scan
    RCCHECK(rclc_timer_init_default(
        &pub_timer,
        &support,
        RCL_MS_TO_NS(2000), // 0.5Hz (Every 2 seconds)
        pub_timer_callback));

    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg, &cmd_vel_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &imu_timer));
    RCCHECK(rclc_executor_add_timer(&executor, &pub_timer));

    motor.init();
}

void loop() {
    // Safety Timeout
    if (millis() - last_cmd_time > CMD_TIMEOUT) {
        motor.stop();
    }
    
    // We increase timeout here because of long blocking operations
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
    delay(10);
}
