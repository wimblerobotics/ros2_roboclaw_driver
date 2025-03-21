#pragma once

#include <stdint.h>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>

#include "roboclaw.h"

class MotorDriver : public rclcpp::Node {
 public:
  static MotorDriver &singleton();

  void onInit(rclcpp::Node::SharedPtr node);

 private:
  MotorDriver();

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) const;

  // For publishing odom, joint state, etc.
  static void publisherThread();

  uint32_t accel_quad_pulses_per_second_;
  uint32_t baud_rate_;
  std::string device_name_;
  uint8_t device_port_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
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
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  bool publish_odom_;
  bool publish_joint_states_;
  std::thread publisher_thread_;  // For publishing odom, joint state, etc.
  uint32_t quad_pulses_per_meter_;
  float quad_pulses_per_revolution_;
  double wheel_radius_;
  double wheel_separation_;

  rclcpp::Node::SharedPtr node_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub_;

  static MotorDriver *g_singleton;
};
