/**
 * @file debris_detection.hpp
 * @brief Header file for debris detection 
 */

#pragma once

#include <chrono>
#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>

#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/msg/twist.hpp"
#include "image_transport/image_transport.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;
using TwistMsg = geometry_msgs::msg::Twist;
using OdomMsg = nav_msgs::msg::Odometry;
using ImageMsg = sensor_msgs::msg::Image;
using BoolMsg = std_msgs::msg::Bool;

class DebrisDetector : public rclcpp::Node {
 private:
  // Variables for image and lidar feeds
  cv::Mat current_image_;
  rclcpp::NodeOptions node_options_;
  image_transport::Subscriber image_subscriber_;

  // Publishers and subscribers
  rclcpp::Publisher<TwistMsg>::SharedPtr velocity_publisher_;
  rclcpp::Publisher<ImageMsg>::SharedPtr debug_image_publisher_;
  rclcpp::Publisher<BoolMsg>::SharedPtr debris_detection_publisher_;
  rclcpp::Subscription<OdomMsg>::SharedPtr odometry_subscriber_;

  // Control and detection variables
  bool rotate_right_;
  bool rotate_left_;
  bool move_forward_;
  bool stop_;
  bool debris_detected_;
  double current_orientation_;
  double initial_orientation_;

 public:
  /**
   * @brief Construct a new Debris Detector object
   */
  DebrisDetector();

  /**
   * @brief Move the robot based on detected debris
   */
  void navigate_to_debris();

  /**
   * @brief Callback for image data to detect debris
   *
   * @param image_msg Image data
   */
  void process_image_callback(const ImageMsg::ConstSharedPtr& image_msg);

  /**
   * @brief Callback for odometry data to track orientation
   *
   * @param odom_msg Odometry data
   */
  void process_odometry_callback(const OdomMsg::SharedPtr odom_msg);

  /**
   * @brief Main function to detect debris and adjust robot behavior
   *
   * @return true If debris is detected and handled
   * @return false If no debris is detected
   */
  bool detect_and_handle_debris();

  bool move2next_debris();

  // Getter functions to access private members for testing
  cv::Mat& get_current_image() { return current_image_; }
  bool is_debris_detected() { return debris_detected_; }
  double get_current_orientation() { return current_orientation_; }
  bool get_rotate_left() { return rotate_left_; }
  bool get_rotate_right() { return rotate_right_; }
  bool get_stop() const { return stop_; }
  bool get_move_forward() const { return move_forward_; }

  // Setter functions to set private members
  void set_rotate_right(bool value) { rotate_right_ = value; }
  void set_rotate_left(bool value) { rotate_left_ = value; }
  void set_move_forward(bool value) { move_forward_ = value; }
  void set_stop(bool value) { stop_ = value; }
  void set_debris_detected(bool value) { debris_detected_ = value; }
  void set_current_orientation(double value) { current_orientation_ = value; }


  // Setter for velocity_publisher_
  void set_velocity_publisher(const rclcpp::Publisher<TwistMsg>::SharedPtr& publisher) {
    velocity_publisher_ = publisher;
  }
};
