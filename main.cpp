// main.cpp
#include "kalman_filter.h"
#include "target_simulator.h"
#include "tracker_display.h"
#include <vector>
#include <iostream>
#include <chrono>
#include <thread>

int main() {
    // Initialize simulation parameters
    const double dt = 0.1;  // Time step

    // Create target simulator (initial position: (0,0), velocity: (1,1))
    TargetSimulator simulator(0.0, 0.0, 1.0, 1.0, 0.1);

    // Create Kalman filter
    KalmanFilter kf;

    // Create display
    TrackerDisplay display(800, 600);

    // Simulation loop
    while (display.isOpen()) {
        // Process window events
        display.processEvents();

        // Get true state and noisy measurement
        Eigen::Vector2d measurement = simulator.getMeasurement(dt);
        Eigen::Vector4d true_state = simulator.getTrueState();

        // Update Kalman filter
        kf.predict(dt);
        kf.update(measurement);

        // Get estimated state
        Eigen::Vector4d est_state = kf.getState();

        // Extract positions for visualization
        Eigen::Vector2d true_pos(true_state[0], true_state[1]);
        Eigen::Vector2d est_pos(est_state[0], est_state[1]);

        // Update and render display
        display.update(true_pos, measurement, est_pos);
        display.render();

        // Print results
        std::cout << "True position: (" << true_pos.x() << ", " << true_pos.y() << ")\n";
        std::cout << "Measured position: (" << measurement.x() << ", " << measurement.y() << ")\n";
        std::cout << "Estimated position: (" << est_pos.x() << ", " << est_pos.y() << ")\n";
        std::cout << "Estimated velocity: (" << est_state[2] << ", " << est_state[3] << ")\n\n";

        // Control simulation speed
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    
    return 0;
}