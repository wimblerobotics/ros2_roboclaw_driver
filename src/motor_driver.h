#pragma once

#include <stdint.h>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "roboclaw.h"

class MotorDriver : public rclcpp::Node {
 public:
  static MotorDriver &singleton();

  void onInit(rclcpp::Node::SharedPtr node);

  RoboClaw &roboClaw() { return *roboclaw_; }

 private:
  MotorDriver();

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) const;

  uint32_t accel_quad_pulses_per_second_;
  std::string device_name_;
  uint8_t device_port_;
  float m1_p_;
  float m1_i_;
  float m1_d_;
  uint32_t m1_qpps_;
  float m1_max_current_;
  float m2_p_;
  float m2_i_;
  float m2_d_;
  uint32_t m2_qpps_;
  float m2_max_current_;
  float max_angular_velocity_;  // Maximum allowed angular velocity.
  float max_linear_velocity_;   // Maximum allowed linear velocity.
  double max_seconds_uncommanded_travel_;
  uint32_t quad_pulses_per_meter_;
  uint32_t quad_pulses_per_revolution_;
  RoboClaw *roboclaw_;
  uint8_t vmin_;
  uint8_t vtime_;
  double wheel_radius_;
  double wheel_separation_;

  rclcpp::Node::SharedPtr node_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub_;

  static MotorDriver *g_singleton;
};
