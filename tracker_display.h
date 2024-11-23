//
// Created by ganno on 11/22/2024.
//

// tracker_display.hpp
#ifndef TRACKER_DISPLAY_HPP
#define TRACKER_DISPLAY_HPP

#include <SFML/Graphics.hpp>
#include <Eigen/Dense>
#include <deque>

class TrackerDisplay {
public:
    TrackerDisplay(unsigned int width = 800, unsigned int height = 600);

    // Update the display with new positions
    void update(const Eigen::Vector2d& true_pos,
                const Eigen::Vector2d& measured_pos,
                const Eigen::Vector2d& estimated_pos);

    // Draw the current state
    void render();

    // Check if window is still open
    bool isOpen() const;

    // Process window events
    void processEvents();

private:
    // Convert from world coordinates to screen coordinates
    sf::Vector2f worldToScreen(const Eigen::Vector2d& world_pos);

    // Convert from screen to world coordinates
    Eigen::Vector2d screenToWorld(const sf::Vector2f& screen_pos);

    sf::RenderWindow window_;
    sf::View view_;

    // Trail of previous positions
    std::deque<sf::Vector2f> true_trail_;
    std::deque<sf::Vector2f> measured_trail_;
    std::deque<sf::Vector2f> estimated_trail_;

    //Viewport Control
    float zoom_level_;
    Eigen::Vector2d view_center_;

    static constexpr size_t MAX_TRAIL_LENGTH = 200;
    static constexpr float SCALE = 30.0f;  // pixels per unit
    static constexpr float INITIAL_ZOOM = 1.0f;
};

#endif // TRACKER_DISPLAY_HPP
