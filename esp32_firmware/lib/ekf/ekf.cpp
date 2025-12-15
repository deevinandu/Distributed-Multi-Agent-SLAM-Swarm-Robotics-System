#include "ekf.h"
#include <cmath>
#include <cstring>

EKF::EKF() : initialized_(false), last_time_(0.0) {
    x_ = Eigen::VectorXd::Zero(STATE_DIM);
    P_ = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
    Q_ = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
    R_odom_ = Eigen::MatrixXd::Identity(2, 2);

    Q_.diagonal() << 0.01, 0.01, 0.01, 0.1, 0.1, 0.001; 
    R_odom_.diagonal() << 0.05, 0.05; 
}

void EKF::init(double t, const Eigen::VectorXd& x0) {
    x_ = x0;
    last_time_ = t;
    initialized_ = true;
}

void EKF::setCovarianceMatrices(const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R_odom) {
    Q_ = Q;
    R_odom_ = R_odom;
}

void EKF::predict(const sensor_msgs__msg__Imu& imu_msg, double current_time) {
    if (!initialized_) return;

    double dt = current_time - last_time_;
    if (dt <= 0) return; 
    last_time_ = current_time;

    double omega_measured = imu_msg.angular_velocity.z;

    double theta = x_(IDX_THETA);
    double v = x_(IDX_V);
    double bias_omega = x_(IDX_BIAS_OMEGA);

    double omega_corrected = omega_measured - bias_omega;

    double theta_new = theta + omega_corrected * dt;
    
    if (theta_new > M_PI) theta_new -= 2 * M_PI;
    else if (theta_new < -M_PI) theta_new += 2 * M_PI;

    double x_new = x_(IDX_X) + v * cos(theta) * dt;
    double y_new = x_(IDX_Y) + v * sin(theta) * dt;

    x_(IDX_X) = x_new;
    x_(IDX_Y) = y_new;
    x_(IDX_THETA) = theta_new;
    x_(IDX_OMEGA) = omega_corrected; 

    // Jacobian F
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
    
    F(IDX_X, IDX_THETA) = -v * sin(theta) * dt;
    F(IDX_X, IDX_V) = cos(theta) * dt;
    
    F(IDX_Y, IDX_THETA) = v * cos(theta) * dt;
    F(IDX_Y, IDX_V) = sin(theta) * dt;

    F(IDX_THETA, IDX_BIAS_OMEGA) = -dt;
    F(IDX_OMEGA, IDX_OMEGA) = 0.0; 
    F(IDX_OMEGA, IDX_BIAS_OMEGA) = -1.0; 

    P_ = F * P_ * F.transpose() + Q_;
}

void EKF::update(const nav_msgs__msg__Odometry& odom_msg) {
    if (!initialized_) return;

    Eigen::VectorXd z(2);
    z << odom_msg.twist.twist.linear.x, odom_msg.twist.twist.angular.z;

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, STATE_DIM);
    H(0, IDX_V) = 1.0;
    H(1, IDX_OMEGA) = 1.0;

    Eigen::VectorXd z_pred(2);
    z_pred << x_(IDX_V), x_(IDX_OMEGA);
    
    Eigen::VectorXd y = z - z_pred;

    Eigen::MatrixXd S = H * P_ * H.transpose() + R_odom_;
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

    x_ = x_ + K * y;

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
    P_ = (I - K * H) * P_;
}

nav_msgs__msg__Odometry EKF::getOdom() const {
    nav_msgs__msg__Odometry odom;
    
    // Safe string copy
    strncpy(odom.header.frame_id.data, "odom", odom.header.frame_id.capacity);
    odom.header.frame_id.size = strlen(odom.header.frame_id.data);
    
    strncpy(odom.child_frame_id.data, "base_link", odom.child_frame_id.capacity);
    odom.child_frame_id.size = strlen(odom.child_frame_id.data);

    odom.pose.pose.position.x = x_(IDX_X);
    odom.pose.pose.position.y = x_(IDX_Y);
    odom.pose.pose.position.z = 0.0;

    double theta = x_(IDX_THETA);
    odom.pose.pose.orientation.z = sin(theta / 2.0);
    odom.pose.pose.orientation.w = cos(theta / 2.0);

    odom.twist.twist.linear.x = x_(IDX_V);
    odom.twist.twist.angular.z = x_(IDX_OMEGA);

    return odom;
}
