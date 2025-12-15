#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <sensor_msgs/msg/imu.h>

#include "ekf.h"

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
nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__LaserScan scan_msg;
float scan_ranges[360]; 

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

rcl_timer_t imu_timer;
rcl_timer_t pub_timer;

EKF ekf;

sensor_msgs__msg__Imu current_imu;
nav_msgs__msg__Odometry current_encoder_odom;

void error_loop(){
  while(1){
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

// Stub: Generates synthetic IMU data (Circle motion, 100Hz)
void readIMU(double t) {
    current_imu.header.stamp.sec = (int)t;
    current_imu.header.stamp.nanosec = (uint32_t)((t - (int)t) * 1e9);
    current_imu.angular_velocity.z = 0.5; 
    current_imu.linear_acceleration.z = 9.81;
}

// Stub: Generates synthetic Encoder data (10Hz)
void readEncoders(double t) {
    current_encoder_odom.twist.twist.linear.x = 0.2; 
    current_encoder_odom.twist.twist.angular.z = 0.5; 
}

// Stub: Generates synthetic LiDAR data (10Hz)
void readLiDAR(double t) {
    scan_msg.header.stamp.sec = (int)t;
    scan_msg.header.stamp.nanosec = (uint32_t)((t - (int)t) * 1e9);
    
    scan_msg.angle_min = -3.14;
    scan_msg.angle_max = 3.14;
    scan_msg.angle_increment = 6.28 / 360.0;
    scan_msg.range_min = 0.15;
    scan_msg.range_max = 12.0;

    for (int i=0; i<360; i++) {
        scan_msg.ranges.data[i] = 2.0;
    }
}

// IMU Callback (100Hz)
void imu_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    RCLC_UNUSED(timer);

    double now_sec = millis() / 1000.0;
    
    readIMU(now_sec);
    ekf.predict(current_imu, now_sec);
}

// Publisher Callback (10Hz)
void pub_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    RCLC_UNUSED(timer);

    double now_sec = millis() / 1000.0;

    readEncoders(now_sec);
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
    
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(6); 
    ekf.init(millis()/1000.0, x0);

    scan_msg.ranges.data = scan_ranges;
    scan_msg.ranges.size = 360;
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

    RCCHECK(rclc_timer_init_default(
        &imu_timer,
        &support,
        RCL_MS_TO_NS(10), // 100Hz
        imu_timer_callback));

    RCCHECK(rclc_timer_init_default(
        &pub_timer,
        &support,
        RCL_MS_TO_NS(100), // 10Hz
        pub_timer_callback));

    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &imu_timer));
    RCCHECK(rclc_executor_add_timer(&executor, &pub_timer));
}

void loop() {
    delay(1); 
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}
