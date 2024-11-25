/**
 * @file main.cpp
 * @brief Entry point for the debris detection application.
 *
 * This file initializes the ROS2 framework, creates an instance of the 
 * `DebrisDetector` class, and starts the debris detection process.
 */

#include "debris_detection.hpp"

/**
 * @brief Main function for the debris detection application.
 * 
 * Initializes the ROS2 environment, creates an instance of the `DebrisDetector` class,
 * and starts the debris detection and handling process.
 * 
 * @param argc The number of command-line arguments.
 * @param argv The array of command-line arguments.
 * @return int Exit status of the program. Returns 0 on successful execution.
 */
int main(int argc, char* argv[]) {
    // Initialize the ROS2 framework
    rclcpp::init(argc, argv);

    // Create a shared pointer to the DebrisDetector object
    auto debris_detector = std::make_shared<DebrisDetector>();

    // Log the start of the debris detection process
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting debris detection...");

    // Begin detecting and handling debris
    debris_detector->detect_and_handle_debris();

    // Shut down the ROS2 framework
    rclcpp::shutdown();

    return 0;
}
