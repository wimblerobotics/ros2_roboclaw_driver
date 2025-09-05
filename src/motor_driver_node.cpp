// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2025 Michael Wimble. https://github.com/wimblerobotics/ros2_roboclaw_driver
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "motor_driver.h"
#include "roboclaw.h"
#include "ros2_roboclaw_driver/msg/robo_claw_status.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("ros2_roboclaw_driver_node");
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
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
