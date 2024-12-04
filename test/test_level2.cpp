// MIT License
// 
// Copyright (c) 2024 Bhavana
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


/**
 * @file test_level2.cpp
 * @brief Integration tests for full system functionality (Level 2)
 */

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "debris_detection.hpp"
#include "debris_remover.hpp"
#include "gazebo_msgs/srv/delete_entity.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/image.hpp"

class DebrisDetectionAndRemovalTest : public ::testing::Test {
 protected:
  std::shared_ptr<DebrisDetector> debris_detector_;
  std::shared_ptr<DebrisRemover> debris_remover_;
  rclcpp::Node::SharedPtr test_node_;

  void SetUp() override {
    rclcpp::init(0, nullptr);
    test_node_ = rclcpp::Node::make_shared("test_node");
    debris_detector_ = std::make_shared<DebrisDetector>();
    debris_remover_ = std::make_shared<DebrisRemover>();

    // Set up mock clients and services
    auto mock_client =
        test_node_->create_client<gazebo_msgs::srv::DeleteEntity>(
            "/delete_entity");
    debris_remover_->set_unspawn_client(mock_client);
    debris_remover_->set_remove_debris_node(test_node_);
  }

  void TearDown() override { rclcpp::shutdown(); }

  // Helper function to simulate debris detection in an image
  void simulateDebrisDetection(const cv::Mat& image) {
    auto image_msg = convertCvMatToImageMsg(image);
    debris_detector_->process_image_callback(image_msg);
  }

  // Helper function to convert OpenCV image to ROS message
  sensor_msgs::msg::Image::SharedPtr convertCvMatToImageMsg(
      const cv::Mat& image) {
    return cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image)
        .toImageMsg();
  }

  // Helper function to simulate odometry data
  void simulateOdometry(double orientation) {
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.pose.pose.orientation.w = orientation;  // Identity quaternion
    debris_detector_->process_odometry_callback(
        std::make_shared<nav_msgs::msg::Odometry>(odom_msg));
  }

  // Helper function to simulate the debris removal service response
  void simulateDebrisRemovalService(bool success) {
    auto mock_service =
        test_node_->create_service<gazebo_msgs::srv::DeleteEntity>(
            "/delete_entity",
            [&](const std::shared_ptr<gazebo_msgs::srv::DeleteEntity::Request>,
                std::shared_ptr<gazebo_msgs::srv::DeleteEntity::Response>
                    response) { response->success = success; });
    rclcpp::spin_some(test_node_);
  }
};

TEST_F(DebrisDetectionAndRemovalTest, FullDebrisDetectionAndRemovalTest) {
  // Step 1: Simulate image with debris detection at a specific position
  cv::Mat test_image_with_debris = cv::Mat::zeros(480, 640, CV_8UC3);
  cv::rectangle(test_image_with_debris, cv::Point(100, 100),
                cv::Point(200, 200), cv::Scalar(255, 0, 0),
                -1);  // Debris at (100,100)

  // Simulate debris detection and navigation decision
  simulateDebrisDetection(test_image_with_debris);

  EXPECT_TRUE(
      debris_detector_->is_debris_detected());  // Ensure debris is detected
  EXPECT_TRUE(debris_detector_->get_rotate_left());  // Ensure rotation left
                                                     // decision (since x < 180)

  // Step 2: Simulate odometry data (robot's orientation)
  simulateOdometry(0.0);  // Assuming initial orientation is 0

  // Step 3: Simulate navigation to debris (using velocity commands)
  debris_detector_->navigate_to_debris();

  // Step 4: Set up a mock debris removal service (similar to RemoveDebrisTest)
  auto mock_client = test_node_->create_client<gazebo_msgs::srv::DeleteEntity>(
      "/delete_entity");

  // Set the mock client in the DebrisRemover
  debris_remover_->set_unspawn_client(mock_client);

  // Set the test node as the remove_debris_node
  debris_remover_->set_remove_debris_node(test_node_);

  // Create a mock service that always succeeds
  auto mock_service =
      test_node_->create_service<gazebo_msgs::srv::DeleteEntity>(
          "/delete_entity",
          [&](const std::shared_ptr<gazebo_msgs::srv::DeleteEntity::Request>,
              std::shared_ptr<gazebo_msgs::srv::DeleteEntity::Response>
                  response) {
            response->success = true;  // Simulate successful debris removal
          });

  // Allow some time for the service to be registered
  rclcpp::spin_some(test_node_);

  // Step 5: Test the debris removal process
  bool debris_removed = debris_remover_->remove_debris("test_object");

  // Ensure the debris removal function returns true (indicating successful
  // removal)
  EXPECT_TRUE(debris_removed);  // Ensure debris was successfully removed
}

TEST_F(DebrisDetectionAndRemovalTest, NoDebrisDetectedTest) {
  // Simulate image without debris
  cv::Mat test_image_without_debris = cv::Mat::zeros(480, 640, CV_8UC3);

  // Simulate debris detection
  simulateDebrisDetection(test_image_without_debris);

  EXPECT_FALSE(
      debris_detector_->is_debris_detected());  // Ensure no debris detected

  // Simulate that debris removal is not triggered
  bool debris_removed = debris_remover_->remove_debris("test_object");
  EXPECT_FALSE(debris_removed);  // Ensure debris removal was not attempted
}

TEST_F(DebrisDetectionAndRemovalTest, DebrisDetectionFailureTest) {
  // Simulate debris detection failure (mock failure scenario)
  cv::Mat test_image_with_debris = cv::Mat::zeros(480, 640, CV_8UC3);
  cv::rectangle(test_image_with_debris, cv::Point(250, 100),
                cv::Point(350, 200), cv::Scalar(255, 0, 0),
                -1);  // Debris at (250,100)

  // Simulate debris detection
  simulateDebrisDetection(test_image_with_debris);

  // Simulate failure of debris removal service (service responds with failure)
  simulateDebrisRemovalService(false);

  // Simulate the removal attempt
  bool debris_removed = debris_remover_->remove_debris("test_object");

  EXPECT_FALSE(debris_removed);  // Ensure debris removal failed due to service
                                 // unavailability
}
