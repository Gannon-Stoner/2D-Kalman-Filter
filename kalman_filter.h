//
// Created by ganno on 11/22/2024.
//

#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <Eigen/Dense>

class KalmanFilter {
public:
    // Constructor
    KalmanFilter();

    // Main Kalman Filter functions
    void predict(double dt);
    void update(const Eigen::Vector2d& measurement);

    // Getters
    Eigen::Vector4d getState() const;
    Eigen::Matrix4d getCovariance() const;

private:
    void updateProcessNoise(double dt);

    Eigen::Vector4d x_;    // State vector [x, y, vx, vy]'
    Eigen::Matrix4d P_;    // State covariance matrix
    Eigen::Matrix4d F_;    // State transition matrix
    Eigen::MatrixXd H_;    // Measurement matrix
    Eigen::Matrix2d R_;    // Measurement noise covariance
    Eigen::Matrix4d Q_;    // Process noise covariance
};

#endif // KALMAN_FILTER_HPP