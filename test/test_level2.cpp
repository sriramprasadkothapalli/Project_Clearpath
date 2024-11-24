#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include "debris_detection.hpp"

// Test suite for DebrisDetector
TEST_CASE("DebrisDetector Class Tests") {
    // Create a shared pointer for the node to manage ROS2 lifecycle
    auto debris_detector = std::make_shared<DebrisDetector>();

    SECTION("Initialization Check") {
        // Check if initial conditions are set correctly
        REQUIRE(debris_detector->is_debris_detected() == false);
        REQUIRE(debris_detector->get_current_orientation() == Approx(0.0));
    }

    SECTION("Image Processing Test") {
        // Simulate an image input
        cv::Mat test_image = cv::Mat::zeros(100, 100, CV_8UC3);
        auto image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", test_image).toImageMsg();
        debris_detector->process_image_callback(image_msg);

        // Verify that the image has been processed
        REQUIRE(!debris_detector->get_current_image().empty());
    }

    SECTION("Odometry Update Test") {
        // Simulate odometry data
        auto odom_msg = std::make_shared<OdomMsg>();
        odom_msg->pose.pose.orientation.z = 0.5; // Simulate a change in orientation
        debris_detector->process_odometry_callback(odom_msg);

        // Check if the current orientation is updated
        REQUIRE(debris_detector->get_current_orientation() == Approx(0.5));
    }

    SECTION("Debris Detection and Handling") {
        // Simulate debris detection
        bool debris_handled = debris_detector->detect_and_handle_debris();

        // Check the response of the debris handling mechanism
        REQUIRE(debris_handled == debris_detector->is_debris_detected());
    }
}
