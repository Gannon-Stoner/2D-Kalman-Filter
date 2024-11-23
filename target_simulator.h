//
// Created by ganno on 11/22/2024.
//

#ifndef TARGET_SIMULATOR_HPP
#define TARGET_SIMULATOR_HPP

#include <Eigen/Dense>
#include <random>

class TargetSimulator {
public:
    // Constructor
    TargetSimulator(double x0, double y0, double vx0, double vy0, double noise_std = 0.1);

    // Get noisy measurement of target position
    Eigen::Vector2d getMeasurement(double dt);

    // Get true state of target
    Eigen::Vector4d getTrueState() const;

private:
    double x_, y_, vx_, vy_, time_;
    std::mt19937 noise_gen_;
    std::normal_distribution<> noise_dist_;
};

#endif // TARGET_SIMULATOR_HPP
