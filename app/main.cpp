/**
 * @file main.cpp
 * @brief Entry point for the debris detection application.
 *
 * This file initializes the ROS2 framework, creates an instance of the
 * `DebrisDetector` class, and starts the debris detection process.
 */

#include <rclcpp/utilities.hpp>

#include "debris_detection.hpp"
#include "debris_remover.hpp"

/**
 * @brief Main function for the debris detection application.
 *
 * Initializes the ROS2 environment, creates an instance of the `DebrisDetector`
 *class,
 * and starts the debris detection and handling process.
 *
 * @param argc The number of command-line arguments.
 * @param argv The array of command-line arguments.
 * @return int Exit status of the program. Returns 0 on successful execution.
 */
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto debris_remover = std::make_shared<DebrisRemover>();
  auto debris_detector = std::make_shared<DebrisDetector>();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting debris detection...");

  std::vector<std::string> object_list = {"trash_block_0", "beer_1",
                                          "coke_can_2", "cricket_ball_4"};

  while (rclcpp::ok() && !object_list.empty()) {
    auto iterator = object_list.begin();

    while (iterator != object_list.end()) {
      if (debris_detector->detect_and_handle_debris()) {
        rclcpp::sleep_for(std::chrono::seconds(1));

        if (debris_detector->detect_and_handle_debris()) {
          if (debris_remover->remove_debris(*iterator)) {
            RCLCPP_INFO(rclcpp::get_logger("main"), "Removed %s",
                        iterator->c_str());
            iterator = object_list.erase(iterator);
          } else {
            // RCLCPP_ERROR(rclcpp::get_logger("main"), "Failed to remove %s",
            // iterator->c_str());
            ++iterator;
          }
        } else {
          RCLCPP_INFO(rclcpp::get_logger("main"),
                      "Object %s not detected, moving to next",
                      iterator->c_str());
          ++iterator;
        }
      } else {
        RCLCPP_INFO(rclcpp::get_logger("main"),
                    "No debris detected, continuing search");
        break;
      }
    }

    if (object_list.empty()) {
      RCLCPP_INFO(rclcpp::get_logger("main"),
                  "All objects removed. Closing node.");
      break;
    }

    if (debris_detector->move2next_debris()) {
      rclcpp::sleep_for(std::chrono::seconds(1));
    } else {
      RCLCPP_INFO(rclcpp::get_logger("main"),
                  "No more debris to move to. Ending detection.");
      break;
    }
  }

  rclcpp::shutdown();
  return 0;
}
