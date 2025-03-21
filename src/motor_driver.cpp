
#include "motor_driver.h"
#include "roboclaw.h"

#include <math.h>
#include <rcutils/logging_macros.h>
#include <stdint.h>

#include <algorithm>
#include <chrono>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "roboclaw.h"

MotorDriver::MotorDriver()
    : Node("motor_driver_node"), device_name_("foo_bar"),
      wheel_radius_(0.10169), wheel_separation_(0.345) {
  this->declare_parameter<int>("accel_quad_pulses_per_second", 600);
  this->declare_parameter<int>("baud_rate", 38400);
  this->declare_parameter<std::string>("device_name", "roboclaw");
  this->declare_parameter<int>("device_port", 123);
  this->declare_parameter<float>("m1_p", 0.0);
  this->declare_parameter<float>("m1_i", 0.0);
  this->declare_parameter<float>("m1_d", 0.0);
  this->declare_parameter<int>("m1_qpps", 0);
  this->declare_parameter<float>("m1_max_current", 0.0);
  this->declare_parameter<float>("m2_p", 0.0);
  this->declare_parameter<float>("m2_i", 0.0);
  this->declare_parameter<float>("m2_d", 0.0);
  this->declare_parameter<int>("m2_qpps", 0);
  this->declare_parameter<float>("max_angular_velocity", 0.0);
  this->declare_parameter<float>("max_linear_velocity", 0.0);
  this->declare_parameter<float>("m2_max_current", 0.0);
  this->declare_parameter<float>("max_seconds_uncommanded_travel", 0.0);
  this->declare_parameter<bool>("publish_joint_states", true);
  this->declare_parameter<bool>("publish_odom", true);
  this->declare_parameter<int>("quad_pulses_per_meter", 0);
  this->declare_parameter<float>("quad_pulses_per_revolution", 0);
  this->declare_parameter<float>("wheel_radius", 0.0);
  this->declare_parameter<float>("wheel_separation", 0.0);
}

void MotorDriver::cmdVelCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg) const {
  if (RoboClaw::singleton() != nullptr) {
    double x_velocity =
        std::min(std::max((float)msg->linear.x, -max_linear_velocity_),
                 max_linear_velocity_);
    double yaw_velocity =
        std::min(std::max((float)msg->angular.z, -max_angular_velocity_),
                 max_angular_velocity_);
    if ((msg->linear.x == 0) && (msg->angular.z == 0)) {
      RoboClaw::singleton()->stop();
    } else if ((fabs(x_velocity) > 0.01) || (fabs(yaw_velocity) > 0.01)) {
      const double m1_desired_velocity =
          x_velocity - (yaw_velocity * wheel_separation_ / 2.0) / wheel_radius_;
      const double m2_desired_velocity =
          x_velocity + (yaw_velocity * wheel_separation_ / 2.0) / wheel_radius_;

      const int32_t m1_quad_pulses_per_second =
          m1_desired_velocity * quad_pulses_per_meter_;
      const int32_t m2_quad_pulses_per_second =
          m2_desired_velocity * quad_pulses_per_meter_;
      const int32_t m1_max_distance =
          fabs(m1_quad_pulses_per_second * max_seconds_uncommanded_travel_);
      const int32_t m2_max_distance =
          fabs(m2_quad_pulses_per_second * max_seconds_uncommanded_travel_);
      RoboClaw::singleton()->doMixedSpeedAccelDist(
          accel_quad_pulses_per_second_, m1_quad_pulses_per_second,
          m1_max_distance, m2_quad_pulses_per_second, m2_max_distance);
    }
  }
}

void MotorDriver::onInit(rclcpp::Node::SharedPtr node) {
  node_ = node;
  this->get_parameter("accel_quad_pulses_per_second",
                      accel_quad_pulses_per_second_);
  this->get_parameter("baud_rate", baud_rate_);
  this->get_parameter("device_name", device_name_);
  this->get_parameter("device_port", device_port_);
  this->get_parameter("m1_p", m1_p_);
  this->get_parameter("m1_i", m1_i_);
  this->get_parameter("m1_d", m1_d_);
  this->get_parameter("m1_qpps", m1_qpps_);
  this->get_parameter("m1_max_current", m1_max_current_);
  this->get_parameter("m2_p", m2_p_);
  this->get_parameter("m2_i", m2_i_);
  this->get_parameter("m2_d", m2_d_);
  this->get_parameter("m2_qpps", m2_qpps_);
  this->get_parameter("m2_max_current", m2_max_current_);
  max_angular_velocity_ = -12.34;
  this->get_parameter("max_angular_velocity", max_angular_velocity_);
  this->get_parameter("max_linear_velocity", max_linear_velocity_);
  this->get_parameter("max_seconds_uncommanded_travel",
                      max_seconds_uncommanded_travel_);
  this->get_parameter("publish_joint_states", publish_joint_states_);
  this->get_parameter("publish_dom", publish_odom_);
  this->get_parameter("quad_pulses_per_meter", quad_pulses_per_meter_);
  this->get_parameter("quad_pulses_per_revolution",
                      quad_pulses_per_revolution_);
  this->get_parameter("wheel_radius", wheel_radius_);
  this->get_parameter("wheel_separation", wheel_separation_);

  RCUTILS_LOG_INFO("accel_quad_pulses_per_second: %d",
                   accel_quad_pulses_per_second_);
  RCUTILS_LOG_INFO("baud_rate: %d", baud_rate_);
  RCUTILS_LOG_INFO("device_name: %s", device_name_.c_str());
  RCUTILS_LOG_INFO("device_port: %d", device_port_);
  RCUTILS_LOG_INFO("m1_p: %f", m1_p_);
  RCUTILS_LOG_INFO("m1_i: %f", m1_i_);
  RCUTILS_LOG_INFO("m1_d: %f", m1_d_);
  RCUTILS_LOG_INFO("m1_qpps: %d", m1_qpps_);
  RCUTILS_LOG_INFO("m1_max_current: %f", m1_max_current_);
  RCUTILS_LOG_INFO("m2_p: %f", m2_p_);
  RCUTILS_LOG_INFO("m2_i: %f", m2_i_);
  RCUTILS_LOG_INFO("m2_d: %f", m2_d_);
  RCUTILS_LOG_INFO("m2_qpps: %d", m2_qpps_);
  RCUTILS_LOG_INFO("m2_max_current: %f", m2_max_current_);
  RCUTILS_LOG_INFO("max_angular_velocity: %f", max_angular_velocity_);
  RCUTILS_LOG_INFO("max_linear_velocity: %f", max_linear_velocity_);
  RCUTILS_LOG_INFO("max_seconds_uncommanded_travel: %f",
                   max_seconds_uncommanded_travel_);
  RCUTILS_LOG_INFO("publish_joint_states: %s",
                   publish_joint_states_ ? "True" : "False");
  RCUTILS_LOG_INFO("quad_pulses_per_meter: %d", quad_pulses_per_meter_);
  RCUTILS_LOG_INFO("quad_pulses_per_revolution: %3.4f",
                   quad_pulses_per_revolution_);
  RCUTILS_LOG_INFO("wheel_radius: %f", wheel_radius_);
  RCUTILS_LOG_INFO("wheel_separation: %f", wheel_separation_);

  RoboClaw::TPIDQ m1Pid = {m1_p_, m1_i_, m1_d_, m1_qpps_, m1_max_current_};
  RoboClaw::TPIDQ m2Pid = {m2_p_, m2_i_, m2_d_, m2_qpps_, m2_max_current_};

  new RoboClaw(m1Pid, m2Pid, m1_max_current_, m2_max_current_,
               device_name_.c_str(), device_port_, baud_rate_);
  RCUTILS_LOG_INFO("Main battery: %f",
                   RoboClaw::singleton()->getMainBatteryLevel());

  auto qos = rclcpp::QoS(
      rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10));
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  qos.durability(rclcpp::DurabilityPolicy::Volatile);
  qos.avoid_ros_namespace_conventions(false);

  cmdVelSub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", qos,
      std::bind(&MotorDriver::cmdVelCallback, this, std::placeholders::_1));

  if (publish_joint_states_) {
    joint_state_publisher_ =
        this->create_publisher<sensor_msgs::msg::JointState>(
            "puck_joint_states", qos);
  }

  if (publish_odom_) {
    odom_publisher_ =
        this->create_publisher<nav_msgs::msg::Odometry>("odom", qos);
  }

  if (publish_joint_states_ || publish_odom_) {
    this->publisher_thread_ = std::thread(&MotorDriver::publisherThread);
  }
}

void MotorDriver::publisherThread() {
  static rclcpp::Clock::SharedPtr clock =
      std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  rclcpp::WallRate loop_rate(20);
  rclcpp::Time now = clock->now();
  rclcpp::Time last_time = now;

  while (rclcpp::ok()) {
    loop_rate.sleep();
    if (RoboClaw::singleton() != nullptr) {
      RoboClaw::singleton()->readSensorGroup();

      nav_msgs::msg::Odometry odometry_msg;
      sensor_msgs::msg::JointState joint_state_msg;

      odometry_msg.header.stamp = clock->now();
      odometry_msg.header.frame_id = "base_link";

      joint_state_msg.header.stamp = clock->now();
      joint_state_msg.header.frame_id = "base_link";

      if (g_singleton->publish_joint_states_) {
        float encoder_left = RoboClaw::singleton()->getM1Encoder() * 1.0;
        float encoder_right = RoboClaw::singleton()->getM2Encoder() * 1.0;
        double radians_left =
            ((encoder_left * 1.0) / g_singleton->quad_pulses_per_revolution_) *
            2.0 * M_PI;
        double radians_right =
            ((encoder_right * 1.0) / g_singleton->quad_pulses_per_revolution_) *
            2.0 * M_PI;
        joint_state_msg.name.push_back("front_left_wheel");
        joint_state_msg.name.push_back("front_right_wheel");
        joint_state_msg.position.push_back(radians_left);
        joint_state_msg.position.push_back(radians_right);
        g_singleton->joint_state_publisher_->publish(joint_state_msg);
      }
    }
  }
}

MotorDriver &MotorDriver::singleton() {
  if (!g_singleton) {
    g_singleton = new MotorDriver();
  }

  return *g_singleton;
}

MotorDriver *MotorDriver::g_singleton = nullptr;