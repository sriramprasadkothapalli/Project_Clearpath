// MIT License
// 
// Copyright (c) 2024 Tathya
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
 * @file debris_remover.hpp
 * @brief Header file for debris remover
 */

#pragma once

#include <chrono>
#include <fstream>
#include <functional>
#include <gazebo_msgs/srv/delete_entity.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <list>
#include <memory>
#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

using TwistVelPub = rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr;
using PoseSub = rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr;
using LaserSub = rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr;
using Timer = rclcpp::TimerBase::SharedPtr;
using namespace std::chrono_literals;

/**
 * @class DebrisRemover
 * @brief A ROS2 node for detecting and removing debris in the robot's environment.
 */
class DebrisRemover : public rclcpp::Node {
 public:
   /**
   * @brief Constructor for the DebrisRemover node.
   */
  DebrisRemover();

   /**
   * @brief Removes debris from the environment by sending a request to the `/delete_entity` service.
   * @param object The name of the debris object to be removed.
   * @return True if the debris was successfully removed, false otherwise.
   */
  virtual bool remove_debris(std::string);

  /**
   * @brief Retrieves the most recent debris index.
   * @return The index of the most recently detected debris.
   */
  int get_recent_debris();

  /**
   * @brief Getter for the debris counter.
   * @return The current value of the debris counter.
   */
  int get_debris_counter() const { return debris_counter; }

  /**
   * @brief Setter for the debris counter.
   * @param count The new value for the debris counter.
   */
  void set_debris_counter(int count) { debris_counter = count; }

  /**
   * @brief Setter for the unspawn client.
   * @param client The client to use for debris removal service calls.
   */
  void set_unspawn_client(rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr client) {
    unspawn_client = client;
  }

  /**
   * @brief Setter for the node used to handle debris removal operations.
   * @param node The node to be set for debris removal operations.
   */
  void set_remove_debris_node(rclcpp::Node::SharedPtr node) {
    remove_debris_node = node;
  }

  /**
   * @brief Getter for the node used to handle debris removal operations.
   * @return The node used for debris removal operations.
   */
  rclcpp::Node::SharedPtr get_remove_debris_node() const {
    return remove_debris_node;
  }

 private:
  sensor_msgs::msg::LaserScan scan_;
  rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr unspawn_client;
  Timer timer_;
  rclcpp::Node::SharedPtr remove_debris_node;
  int debris_counter;
  std::list<int> debris_idx{};
};