#ifndef EKF_H
#define EKF_H

#include <Arduino.h>
#include "eigen.h"      // Calls main Eigen matrix class library
#include <Eigen/LU>     // Calls inverse, determinant, LU decomp., etc.

// #include <Eigen/Dense> // Standard include (fails on some Arduino setups)
#include "ros_compat.h" // Replaces <sensor_msgs/msg/imu.h> etc.

class EKF {
public:
    EKF();

    void init(double t, const Eigen::VectorXd& x0);
    
    // Prediction step using IMU data
    void predict(const sensor_msgs__msg__Imu& imu_msg, double current_time);

    // Update step using Wheel Encoder data
    void update(const nav_msgs__msg__Odometry& odom_msg);

    nav_msgs__msg__Odometry getOdom() const;

    void setCovarianceMatrices(const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R_odom);

    Eigen::VectorXd getState() const { return x_; }

private:
    Eigen::VectorXd x_;
    Eigen::MatrixXd P_;
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd R_odom_;

    bool initialized_;
    double last_time_;

    static const int IDX_X = 0;
    static const int IDX_Y = 1;
    static const int IDX_THETA = 2;
    static const int IDX_V = 3;
    static const int IDX_OMEGA = 4;
    static const int IDX_BIAS_OMEGA = 5;
    static const int STATE_DIM = 6;
};

#endif // EKF_H
