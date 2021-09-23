#pragma once

#include <stdint.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
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

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  uint32_t accel_quad_pulses_per_second_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
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
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;

  static MotorDriver *g_singleton;
};
