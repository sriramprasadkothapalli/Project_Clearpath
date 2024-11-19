#include "debris_detection.hpp"
#include <functional>
#include <rclcpp/logging.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
using std::placeholders::_1;

DebrisDetector::DebrisDetector() : Node("debris_detector") {
    // Initialize publishers and subscribers
    velocity_publisher_ = this->create_publisher<TwistMsg>("cmd_vel", 10);
    debug_image_publisher_ = this->create_publisher<ImageMsg>("/debug_image", 10);
    debris_detection_publisher_ = this->create_publisher<BoolMsg>("/debris_detected", 10);

    image_subscriber_ = image_transport::create_subscription(
        this, "camera/image_raw", 
        std::bind(&DebrisDetector::process_image_callback, this, _1), 
        "raw"
    );

    odometry_subscriber_ = this->create_subscription<OdomMsg>(
        "odom", 10, 
        std::bind(&DebrisDetector::process_odometry_callback, this, _1)
    );

    // Initialize control and detection variables
    rotate_right_ = false;
    rotate_left_ = false;
    move_forward_ = false;
    stop_ = false;
    debris_detected_ = false;
    current_orientation_ = 0.0;
    initial_orientation_ = 0.0;
}

void DebrisDetector::navigate_to_debris() {
    TwistMsg velocity_command;
    if (rotate_right_) {
        velocity_command.angular.z = -0.05;
        velocity_command.linear.x = 0.0;
    } else if (rotate_left_) {
        velocity_command.angular.z = 0.05;
        velocity_command.linear.x = 0.0;
    } else if (move_forward_) {
        velocity_command.linear.x = 0.1;
        velocity_command.angular.z = 0.0;
    } else if (stop_) {
        velocity_command.linear.x = 0.0;
        velocity_command.angular.z = 0.0;
    }
    velocity_publisher_->publish(velocity_command);
}

void DebrisDetector::process_image_callback(const ImageMsg::ConstSharedPtr& msg) {
    try {
        cv::Mat hsv_image, binary_image;
        cv::Mat image = cv_bridge::toCvShare(msg, msg->encoding)->image;

        // Display the raw image (Camera POV)
        cv::imshow("Camera POV - Raw Image", image);

        // HSV ranges for debris colors (blue, green, red)
        cv::Mat blue_mask, red_mask, green_mask, debris_mask;

        cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);

        // Define thresholds
        cv::inRange(hsv_image, cv::Scalar(100, 80, 80), cv::Scalar(132, 255, 255), blue_mask);  // Blue
        cv::inRange(hsv_image, cv::Scalar(40, 80, 80), cv::Scalar(70, 255, 255), green_mask);  // Green
        cv::inRange(hsv_image, cv::Scalar(0, 80, 80), cv::Scalar(10, 255, 255), red_mask);
        cv::inRange(hsv_image, cv::Scalar(160, 80, 80), cv::Scalar(180, 255, 255), red_mask);  // Red (two ranges)

        // Combine masks
        cv::bitwise_or(blue_mask, green_mask, debris_mask);
        cv::bitwise_or(debris_mask, red_mask, debris_mask);

        // Detect contours
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(debris_mask, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

        debris_detected_ = !contours.empty();

        if (debris_detected_) {
            cv::Rect bounding_box = cv::boundingRect(contours[0]);
            int center_x = bounding_box.x + bounding_box.width / 2;
            int area = bounding_box.area();

            // Control logic based on detected debris position and size
            if (center_x < 180) {
                rotate_left_ = true;
                rotate_right_ = false;
            } else if (center_x > 200) {
                rotate_right_ = true;
                rotate_left_ = false;
            } else {
                rotate_left_ = false;
                rotate_right_ = false;
                move_forward_ = area <= 40000;
                stop_ = area > 40000;
            }

            // Draw bounding box around debris
            cv::rectangle(image, bounding_box, cv::Scalar(0, 255, 0), 2);
        }

        // Display the processed image
        cv::imshow("Debris Detection - Processed Image", debris_mask);

        // Add bounding box visualization to the raw image
        cv::imshow("Camera POV - Processed", image);

        // Publish debug image
        auto debug_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", debris_mask).toImageMsg();
        debug_image_publisher_->publish(*debug_msg);

        // Wait for a short period to refresh OpenCV windows
        cv::waitKey(1);

    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error converting image: %s", e.what());
    }
}

void DebrisDetector::process_odometry_callback(const OdomMsg::SharedPtr odom_msg) {
    tf2::Quaternion orientation(
        odom_msg->pose.pose.orientation.x,
        odom_msg->pose.pose.orientation.y,
        odom_msg->pose.pose.orientation.z,
        odom_msg->pose.pose.orientation.w
    );
    tf2::Matrix3x3 rotation_matrix(orientation);
    double roll, pitch, yaw;
    rotation_matrix.getRPY(roll, pitch, yaw);
    current_orientation_ = yaw;
}

bool DebrisDetector::detect_and_handle_debris() {
    rclcpp::spin_some(shared_from_this());
    initial_orientation_ = current_orientation_;

    while (rclcpp::ok()) {
        rclcpp::spin_some(shared_from_this());
        navigate_to_debris();
        if (stop_) {
            return true;
        }
    }
    return false;
}
