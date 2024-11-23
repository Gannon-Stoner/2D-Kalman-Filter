//
// Created by ganno on 11/22/2024.
//

#include "tracker_display.h"

TrackerDisplay::TrackerDisplay(unsigned int width, unsigned int height)
    : window_(sf::VideoMode(width, height), "Kalman Filter Tracking Visualization")
    , zoom_level_(1.0f)
    , view_center_(Eigen::Vector2d::Zero()) {

    window_.setFramerateLimit(60);

    // Initalize view
    view_ = window_.getDefaultView();
    window_.setView(view_);
}

void TrackerDisplay::update(const Eigen::Vector2d& true_pos,
                          const Eigen::Vector2d& measured_pos,
                          const Eigen::Vector2d& estimated_pos) {
    // Convert positions to screen coordinates and add to trails
    true_trail_.push_back(worldToScreen(true_pos));
    measured_trail_.push_back(worldToScreen(measured_pos));
    estimated_trail_.push_back(worldToScreen(estimated_pos));

    // Update view center to follow the target smoothly
    view_center_ = estimated_pos;

    // Calculate view position in screen coordinates
    sf::Vector2f screen_center = worldToScreen(view_center_);

    // Create a new view centered on the target
    sf::View new_view(screen_center, sf::Vector2f(window_.getSize()));
    new_view.zoom(zoom_level_);
    window_.setView(new_view);

    // Limit trail length
    if (true_trail_.size() > MAX_TRAIL_LENGTH) {
        true_trail_.pop_front();
        measured_trail_.pop_front();
        estimated_trail_.pop_front();
    }
}

void TrackerDisplay::render() {
    window_.clear(sf::Color(50, 50, 50));  // Dark gray background

    // Draw coordinate grid
    float grid_size = 1.0f;  // Grid cell size in world units

    // Get the visible area in world coordinates
    sf::Vector2f viewSize = window_.getView().getSize();
    sf::Vector2f viewCenter = window_.getView().getCenter();

    // Calculate grid boundaries with extra padding
    float padding = 10.0f; // Add more cells beyond the visible area
    float left = viewCenter.x - viewSize.x/2 - (padding * grid_size * SCALE);
    float right = viewCenter.x + viewSize.x/2 + (padding * grid_size * SCALE);
    float top = viewCenter.y - viewSize.y/2 - (padding * grid_size * SCALE);
    float bottom = viewCenter.y + viewSize.y/2 + (padding * grid_size * SCALE);

    // Draw vertical grid lines
    for (float x = std::floor(left/SCALE); x <= std::ceil(right/SCALE); x += grid_size) {
        sf::RectangleShape line(sf::Vector2f(1, bottom - top));
        float screenX = x * SCALE;
        line.setPosition(screenX, top);
        line.setFillColor(sf::Color(100, 100, 100, 100));
        window_.draw(line);
    }

    // Draw horizontal grid lines
    for (float y = std::floor(top/SCALE); y <= std::ceil(bottom/SCALE); y += grid_size) {
        sf::RectangleShape line(sf::Vector2f(right - left, 1));
        float screenY = y;
        line.setPosition(left, screenY);
        line.setFillColor(sf::Color(100, 100, 100, 100));
        window_.draw(line);
    }

    // Draw trails
    auto drawTrail = [this](const std::deque<sf::Vector2f>& trail,
                           const sf::Color& color, float radius,
                           bool fade = true) {
        for (size_t i = 0; i < trail.size(); ++i) {
            sf::CircleShape point(radius);
            point.setPosition(trail[i]);

            // Calculate alpha based on position in trail
            uint8_t alpha = fade ?
                static_cast<uint8_t>(255.0f * (static_cast<float>(i) / trail.size())) :
                255;

            point.setFillColor(sf::Color(color.r, color.g, color.b, alpha));
            point.setOrigin(radius, radius);
            window_.draw(point);
        }
    };

    // Draw trails from oldest (thinner) to newest (thicker)
    drawTrail(true_trail_, sf::Color::Green, 2.0f);      // True position
    drawTrail(measured_trail_, sf::Color::Red, 2.0f);    // Measured position
    drawTrail(estimated_trail_, sf::Color::Blue, 3.0f);  // Estimated position

    // Draw thick axes lines
    sf::RectangleShape xAxis(sf::Vector2f(right - left, 2));
    sf::RectangleShape yAxis(sf::Vector2f(2, bottom - top));
    xAxis.setPosition(left, 0);
    yAxis.setPosition(0, top);
    xAxis.setFillColor(sf::Color(150, 150, 150));
    yAxis.setFillColor(sf::Color(150, 150, 150));
    window_.draw(xAxis);
    window_.draw(yAxis);

    // Draw legend (fixed on screen)
    sf::View defaultView = window_.getDefaultView();
    window_.setView(defaultView);

    // Add legend
    auto drawLegendItem = [this](const std::string& text, const sf::Color& color, float yPos) {
        sf::CircleShape point(5.0f);
        point.setPosition(10.0f, yPos);
        point.setFillColor(color);
        point.setOrigin(5.0f, 5.0f);

        static sf::Font font;
        static bool fontLoaded = false;
        if (!fontLoaded) {
            // Try to load the system Arial font
            if (!font.loadFromFile("C:/Windows/Fonts/arial.ttf")) {
                // If Arial fails, try Segoe UI (another common Windows font)
                if (!font.loadFromFile("C:/Windows/Fonts/segoeui.ttf")) {
                    // If both fail, just skip the text rendering
                    window_.draw(point);
                    return;
                }
            }
            fontLoaded = true;
        }

        sf::Text label;
        label.setFont(font);
        label.setString(text);
        label.setCharacterSize(14);
        label.setFillColor(sf::Color::White);
        label.setPosition(25.0f, yPos - 7.0f);

        window_.draw(point);
        window_.draw(label);
    };

    drawLegendItem("True Position", sf::Color::Green, 30.0f);
    drawLegendItem("Measured Position", sf::Color::Red, 55.0f);
    drawLegendItem("Estimated Position", sf::Color::Blue, 80.0f);

    // Restore the view
    window_.setView(view_);

    window_.display();
}

bool TrackerDisplay::isOpen() const {
    return window_.isOpen();
}

void TrackerDisplay::processEvents() {
    sf::Event event;
    while (window_.pollEvent(event)) {
        if (event.type == sf::Event::Closed)
            window_.close();
        else if (event.type == sf::Event::MouseWheelScrolled) {
            // Zoom with mouse wheel
            float zoom_factor = (event.mouseWheelScroll.delta > 0) ? 0.9f : 1.1f;
            zoom_level_ *= zoom_factor;
            view_.zoom(zoom_factor);
            window_.setView(view_);
        }
    }
}

sf::Vector2f TrackerDisplay::worldToScreen(const Eigen::Vector2d& world_pos) {
    // Convert world coordinates to screen coordinates relative to view center
    float screen_x = static_cast<float>(world_pos.x()) * SCALE;
    float screen_y = -static_cast<float>(world_pos.y()) * SCALE;  // Flip Y axis

    return sf::Vector2f(screen_x, screen_y);
}

Eigen::Vector2d TrackerDisplay::screenToWorld(const sf::Vector2f& screen_pos) {
    // Convert screen coordinates to world coordinates
    return Eigen::Vector2d(
        screen_pos.x / SCALE,
        -screen_pos.y / SCALE  // Flip Y axis back
    );
}