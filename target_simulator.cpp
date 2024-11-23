//
// Created by ganno on 11/22/2024.
//

// target_simulator.cpp
#include "target_simulator.h"
#include <cmath>

TargetSimulator::TargetSimulator(double x0, double y0, double vx0, double vy0, double noise_std)
    : x_(x0), y_(y0), vx_(vx0), vy_(vy0),
      noise_gen_(std::random_device{}()),
      noise_dist_(0.0, noise_std),
      time_(0) {
}

Eigen::Vector2d TargetSimulator::getMeasurement(double dt) {
    // Update time
    time_ += dt;

    // Create a circular motion with some variation
    double radius = 3.0;
    double angular_velocity = 0.5;
    double drift = 0.05;

    // Update position using circular motion with some drift
    x_ = radius * std::cos(angular_velocity * time_) + time_ * drift;
    y_ = radius * std::sin(angular_velocity * time_) + time_ * drift;

    // Calculate velocities
    vx_ = -radius * angular_velocity * std::sin(angular_velocity * time_) + 0.1;
    vy_ = radius * angular_velocity * std::cos(angular_velocity * time_) + 0.1;

    // Add measurement noise
    return Eigen::Vector2d(
        x_ + noise_dist_(noise_gen_),
        y_ + noise_dist_(noise_gen_)
    );
}

Eigen::Vector4d TargetSimulator::getTrueState() const {
    return Eigen::Vector4d(x_, y_, vx_, vy_);
}