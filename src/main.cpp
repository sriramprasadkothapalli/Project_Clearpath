#include "debris_detection.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto debris_detector = std::make_shared<DebrisDetector>();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting debris detection...");
    debris_detector->detect_and_handle_debris();

    rclcpp::shutdown();
    return 0;
}
