/**
 * @file test_level1.cpp
 * @brief Unit tests for individual components (Level 1)
 */

#include <gtest/gtest.h>
#include <tf2/LinearMath/Quaternion.h>

#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include "cv_bridge/cv_bridge.h"
#include "debris_detection.hpp"
#include "debris_remover.hpp"
#include "gazebo_msgs/srv/delete_entity.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"

// Test fixture for DebrisDetector
class DebrisDetectorTest : public ::testing::Test {
 protected:
  std::shared_ptr<DebrisDetector> debris_detector_;

  void SetUp() override {
    rclcpp::init(0, nullptr);
    debris_detector_ = std::make_shared<DebrisDetector>();
  }

  void TearDown() override { rclcpp::shutdown(); }

  // Helper function to create a test image with debris at a specific position
  cv::Mat createTestImageWithDebrisAtPosition(int x, int y) {
    cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);
    cv::rectangle(image, cv::Point(x, y), cv::Point(x + 100, y + 100),
                  cv::Scalar(255, 0, 0), -1);
    return image;
  }

  // Helper function to create a test image without debris
  cv::Mat createTestImageWithoutDebris() {
    return cv::Mat::zeros(480, 640, CV_8UC3);
  }

  // Helper function to publish an image message
  sensor_msgs::msg::Image::SharedPtr convertCvMatToImageMsg(
      const cv::Mat& image) {
    auto image_msg =
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();

    return image_msg;
  }
};

// Test fixture for DebrisRemover
class DebrisRemoverTest : public ::testing::Test {
 protected:
  std::shared_ptr<DebrisRemover> debris_remover_;
  rclcpp::Node::SharedPtr test_node_;

  // Set up the test environment
  void SetUp() override {
    rclcpp::init(0, nullptr);
    debris_remover_ = std::make_shared<DebrisRemover>();
    test_node_ = rclcpp::Node::make_shared("test_node");
  }

  // Clean up after the test
  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(DebrisRemoverTest, ServiceUnavailableTest) {
  // Create a mock client but do not set up the service callback
  auto mock_client = test_node_->create_client<gazebo_msgs::srv::DeleteEntity>(
      "/delete_entity");

  // Set the mock client in the DebrisRemover using the setter
  debris_remover_->set_unspawn_client(mock_client);

  // Set the test node as the remove_debris_node using the setter
  debris_remover_->set_remove_debris_node(test_node_);

  // Simulate a service unavailable scenario by not setting up the mock service
  bool result = debris_remover_->remove_debris("test_object");

  // Assert that debris removal failed due to service unavailability
  EXPECT_FALSE(result);
}

TEST_F(DebrisRemoverTest, RemoveDebrisTest) {
  // Create a mock client
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
                  response) { response->success = true; });

  // Allow some time for the service to be registered
  rclcpp::spin_some(test_node_);

  // Test remove_debris function
  bool result = debris_remover_->remove_debris("test_object");

  EXPECT_TRUE(result);
}

// Test the creation of DebrisRemover and basic state
TEST_F(DebrisRemoverTest, ConstructorTest) {
  EXPECT_NE(debris_remover_, nullptr);
  EXPECT_EQ(debris_remover_->get_debris_counter(), 0);
}

// Test the setter and getter for debris_counter
TEST_F(DebrisRemoverTest, DebrisCounterTest) {
  debris_remover_->set_debris_counter(5);
  EXPECT_EQ(debris_remover_->get_debris_counter(), 5);
}

// Test case for detecting debris in an image with a colored rectangle
TEST_F(DebrisDetectorTest, DetectsDebrisInImage) {
  // Test case 1: Center_x < 180, which triggers rotate_left_
  auto test_image_left = createTestImageWithDebrisAtPosition(
      100, 100);  // Debris placed at (100,100)
  auto image_msg_left = convertCvMatToImageMsg(test_image_left);

  debris_detector_->process_image_callback(image_msg_left);

  EXPECT_TRUE(debris_detector_->is_debris_detected());
  EXPECT_TRUE(debris_detector_->get_rotate_left());
  EXPECT_FALSE(debris_detector_->get_rotate_right());
  EXPECT_FALSE(debris_detector_->get_move_forward());

  // Test case 2: Center_x > 200, which triggers rotate_right_
  auto test_image_right = createTestImageWithDebrisAtPosition(
      250, 100);  // Debris placed at (250,100)
  auto image_msg_right = convertCvMatToImageMsg(test_image_right);

  debris_detector_->process_image_callback(image_msg_right);

  EXPECT_TRUE(debris_detector_->is_debris_detected());
  EXPECT_FALSE(debris_detector_->get_rotate_left());
  EXPECT_TRUE(debris_detector_->get_rotate_right());
  EXPECT_FALSE(debris_detector_->get_move_forward());

  // Test case 3: Center_x between 180 and 200, area <= 40000, triggers
  // move_forward_
  auto test_image_center = createTestImageWithDebrisAtPosition(
      140, 100);  // Debris placed at (190,100)
  auto image_msg_center = convertCvMatToImageMsg(test_image_center);

  debris_detector_->process_image_callback(image_msg_center);

  EXPECT_TRUE(debris_detector_->is_debris_detected());
  EXPECT_FALSE(debris_detector_->get_rotate_left());
  EXPECT_FALSE(debris_detector_->get_rotate_right());
  EXPECT_TRUE(debris_detector_->get_move_forward());
}

// Test case for handling an image without debris
TEST_F(DebrisDetectorTest, NoDebrisDetectedInImage) {
  auto test_image = createTestImageWithoutDebris();
  auto image_msg = convertCvMatToImageMsg(test_image);

  debris_detector_->process_image_callback(image_msg);

  EXPECT_FALSE(debris_detector_->is_debris_detected());
}

// Test case for checking initial orientation update from odometry
TEST_F(DebrisDetectorTest, ProcessesOdometryCorrectly) {
  nav_msgs::msg::Odometry odom_msg;

  odom_msg.pose.pose.orientation.w = 1.0;  // Identity quaternion

  debris_detector_->process_odometry_callback(
      std::make_shared<nav_msgs::msg::Odometry>(odom_msg));

  EXPECT_NEAR(debris_detector_->get_current_orientation(), 0.0, 1e-5);
}

// Test case for navigate_to_debris function
TEST_F(DebrisDetectorTest, NavigateToDebrisTest) {
  // Mock publisher for velocity commands
  debris_detector_->set_velocity_publisher(
      debris_detector_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",
                                                                    10));

  // Helper lambda to test each navigation state
  auto test_navigation_state = [&](bool rotate_right, bool rotate_left,
                                   bool move_forward, bool stop,
                                   double expected_linear_x,
                                   double expected_angular_z) {
    // Set navigation states
    debris_detector_->set_rotate_right(rotate_right);
    debris_detector_->set_rotate_left(rotate_left);
    debris_detector_->set_move_forward(move_forward);
    debris_detector_->set_stop(stop);

    // Capture published velocity command
    geometry_msgs::msg::Twist last_command;
    auto sub = debris_detector_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        [&](geometry_msgs::msg::Twist::SharedPtr msg) { last_command = *msg; });

    // Call the function to test
    debris_detector_->navigate_to_debris();

    // Allow for message propagation
    rclcpp::spin_some(debris_detector_);

    // Validate the published velocity command
    EXPECT_NEAR(last_command.linear.x, expected_linear_x, 1e-5);
    EXPECT_NEAR(last_command.angular.z, expected_angular_z, 1e-5);
  };

  // Test each state
  test_navigation_state(true, false, false, false, 0.02, -0.1);  // Rotate right
  test_navigation_state(false, true, false, false, 0.02, 0.1);   // Rotate left
  test_navigation_state(false, false, true, false, 0.10, 0.0);   // Move forward
  test_navigation_state(false, false, false, true, 0.0, 0.0);    // Stop
}

TEST_F(DebrisDetectorTest, DetectAndHandleDebrisTest) {
  // Mock the necessary functions
  debris_detector_->set_debris_detected(true);
  debris_detector_->set_stop(true);

  // Test the function
  EXPECT_TRUE(debris_detector_->detect_and_handle_debris());

  // Verify the state after execution
  EXPECT_FALSE(debris_detector_->get_stop());
  EXPECT_FALSE(debris_detector_->get_move_forward());
  EXPECT_FALSE(debris_detector_->get_rotate_left());
  EXPECT_FALSE(debris_detector_->get_rotate_right());
}

TEST_F(DebrisDetectorTest, Move2NextDebrisTest) {
  // Mock the necessary functions
  debris_detector_->set_current_orientation(0.0);
  debris_detector_->set_debris_detected(true);

  // Test the function
  EXPECT_TRUE(debris_detector_->move2next_debris());
}
