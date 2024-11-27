/**
 * @file debris_remover.cpp
 * @author Tathya Bhatt
 * @brief C++ file for debris remover implementation
 * @version 0.1
 * @copyright Copyright (c) 2024
 */

#include "debris_remover.hpp"
#include <gazebo_msgs/srv/detail/delete_entity__struct.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/future_return_code.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/utilities.hpp>
#include <vector>

using namespace std::chrono_literals;
using std::placeholders::_1;

DebrisRemover::DebrisRemover() : rclcpp::Node("debris_remover"){
    unspawn_client = create_client<gazebo_msgs::srv::DeleteEntity>("/delete_entity");

    remove_debris_node = rclcpp::Node::make_shared("debris_remover");

    debris_idx = {};

    debris_counter = 3;

    for (int i=0; i<debris_counter; i++){
        debris_idx.push_back(i);
    }
}

bool DebrisRemover::remove_debris() {
    while (!unspawn_client->wait_for_service(5s)){
        if(!rclcpp::ok()){
            RCLCPP_ERROR(this->get_logger(), "Service Call Failed");

            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Service Not Available");
    }

    auto request = std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();
    // int closest_debris = get_recent_debris();
    // RCLCPP_INFO(this->get_logger(), "No Debris Left %d", closest_debris);
    // if (closest_debris == -1) {
    //     RCLCPP_ERROR(this->get_logger(), "No Debris Left");
    //     return false;
    // }
    
    
    request->name = "trash_block_0";

    auto srv_response = unspawn_client->async_send_request(request);
    auto ret = rclcpp::spin_until_future_complete(remove_debris_node, srv_response, 3s);

    if (ret==rclcpp::FutureReturnCode::SUCCESS) {
        return true;
    } else {
        return false;
    }    
}

int DebrisRemover::get_recent_debris(){
    if (debris_idx.empty()){
        return -1;
    };

    int closest_idx = static_cast<int> (debris_idx.front());
    debris_idx.pop_front();
    return closest_idx;
}