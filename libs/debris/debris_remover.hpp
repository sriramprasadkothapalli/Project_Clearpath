/**
 * @file debris_remover.hpp
 * @author Tathya Bhatt
 * @brief Header file for dynamically modiying the object after detection
 * @version 0.1
 * @copyright Copyright (c) 2024
 */

#pragma once


#include <chrono>
#include <fstream>
#include <functional>
#include <memory>
#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
// #include <sensor_msgs/msg/detail/laser_scan__struct.hpp>
#include <string>
#include <list>

#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/srv/delete_entity.hpp>
#include <std_msgs/msg/string.hpp>
// #include <gazebo_msgs/srv/detail/delete_entity__struct.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using TwistVelPub = rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr;
using PoseSub = rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr;
using LaserSub = rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr;
using Timer = rclcpp::TimerBase::SharedPtr;
using namespace std::chrono_literals;



class DebrisRemover : public rclcpp::Node {
    public:
        DebrisRemover();

        bool remove_debris();

        int get_recent_debris();

    private:
        sensor_msgs::msg::LaserScan scan_;
        rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr unspawn_client;
        Timer timer_;
        rclcpp::Node::SharedPtr remove_debris_node;
        int debris_counter;
        std::list<int> debris_idx {};
};