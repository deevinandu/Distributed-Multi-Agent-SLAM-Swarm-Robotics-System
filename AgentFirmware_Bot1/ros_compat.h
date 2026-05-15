#ifndef ROS_COMPAT_H
#define ROS_COMPAT_H

#include <stdint.h>
#include <string.h>

// Minimal implementation of ROS 2 types for local usage
// NOTE: Renamed "String" to "RosString" to avoid Arduino conflict

struct RosString {
    char data[32]; // Fixed buffer for safety
    size_t size;
    size_t capacity;
    
    RosString() {
        size = 0;
        capacity = 32;
        data[0] = '\0';
    }
};

struct Time {
    int sec;
    uint32_t nanosec;
};

struct Header {
    Time stamp;
    RosString frame_id;
};

struct Vector3 {
    double x;
    double y;
    double z;
};

struct Quaternion {
    double x;
    double y;
    double z;
    double w;
};

struct Point {
    double x;
    double y;
    double z;
};

struct TwistStruct {
    Vector3 linear;
    Vector3 angular;
};

struct TwistWithCovariance {
    TwistStruct twist;
    double covariance[36];
};

struct Pose {
    Point position;
    Quaternion orientation;
};

struct PoseWithCovariance {
    Pose pose;
    double covariance[36];
};

struct nav_msgs__msg__Odometry {
    Header header;
    RosString child_frame_id;
    PoseWithCovariance pose;
    TwistWithCovariance twist;
};

struct sensor_msgs__msg__Imu {
    Header header;
    Quaternion orientation;
    Vector3 angular_velocity;
    Vector3 linear_acceleration;
};

#endif // ROS_COMPAT_H
