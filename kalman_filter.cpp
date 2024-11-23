//
// Created by ganno on 11/22/2024.
//

#include "kalman_filter.h"
#include <iostream>

KalmanFilter::KalmanFilter() {
    // Initialize state vector
    x_ = Eigen::Vector4d::Zero();

    // Initialize state covariance with high uncertainty
    P_ = Eigen::Matrix4d::Identity() * 1000;

    // Initialize state transition matrix (constant velocity model)
    F_ = Eigen::Matrix4d::Identity();

    // Initialize measurement matrix (we only measure position [x, y])
    H_ = Eigen::MatrixXd(2, 4);
    H_ << 1, 0, 0, 0,
          0, 1, 0, 0;

    // Initialize measurement noise covariance
    R_ = Eigen::Matrix2d::Identity() * 0.005;  // Adjust based on sensor noise

    // Initialize process noise covariance
    Q_ = Eigen::Matrix4d::Identity() * 0.05;   // Adjust based on system dynamics
}

void KalmanFilter::predict(double dt) {
    // Update state transition matrix with dt
    F_(0, 2) = dt;
    F_(1, 3) = dt;

    // Update process noise
    updateProcessNoise(dt);

    // Predict state
    x_ = F_ * x_;

    // Predict covariance
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::update(const Eigen::Vector2d& measurement) {
    // Calculate Kalman gain
    Eigen::MatrixXd K = P_ * H_.transpose() *
        (H_ * P_ * H_.transpose() + R_).inverse();

    // Update state
    x_ = x_ + K * (measurement - H_ * x_);

    // Update covariance
    Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
    P_ = (I - K * H_) * P_;
}

void KalmanFilter::updateProcessNoise(double dt) {
    // Simple continuous white noise acceleration model
    double dt2 = dt * dt;
    double dt3 = dt2 * dt;
    double dt4 = dt3 * dt;

    Q_(0,0) = dt4/4; Q_(0,2) = dt3/2;
    Q_(1,1) = dt4/4; Q_(1,3) = dt3/2;
    Q_(2,0) = dt3/2; Q_(2,2) = dt2;
    Q_(3,1) = dt3/2; Q_(3,3) = dt2;

    // Adjust based on motion
    double q = 0.1;
    Q_ *= q;  // Process noise intensity
}

Eigen::Vector4d KalmanFilter::getState() const {
    return x_;
}

Eigen::Matrix4d KalmanFilter::getCovariance() const {
    return P_;
}