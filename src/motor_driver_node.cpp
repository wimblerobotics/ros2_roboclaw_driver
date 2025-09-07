// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Wimblerobotics
// https://github.com/wimblerobotics/ros2_roboclaw_driver

#include <rcutils/logging_macros.h>

#include <rclcpp/rclcpp.hpp>
#include <string>

#include "ros2_roboclaw_driver/motor_driver.h"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("motor_driver_node");

  MotorDriver& motorDriver = MotorDriver::singleton();
  motorDriver.onInit(node);

  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10));
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
  qos.avoid_ros_namespace_conventions(false);

  std::string statusTopicName;
  node->declare_parameter<std::string>("roboclaw_status_topic", "roboclaw_status");
  node->get_parameter("roboclaw_status_topic", statusTopicName);
  RCUTILS_LOG_INFO("[motor_driver_node] roboclaw_status_topic: %s", statusTopicName.c_str());

  rclcpp::Publisher<ros2_roboclaw_driver::msg::RoboClawStatus>::SharedPtr statusPublisher =
      node->create_publisher<ros2_roboclaw_driver::msg::RoboClawStatus>(statusTopicName, qos);

  motorDriver.setStatusPublisher(statusPublisher);

  // Use single-threaded executor (all work done in timers/subscriptions serially)
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
