
#include "motor_driver.h"

#include <math.h>
#include <rcutils/logging_macros.h>
#include <stdint.h>

#include <algorithm>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "roboclaw.h"

MotorDriver::MotorDriver()
    : Node("motor_driver_node"),
      device_name_("foo_bar"),
      wheel_radius_(0.10169),
      wheel_separation_(0.345) {
  this->declare_parameter<int>("accel_quad_pulses_per_second", 600);
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
  this->declare_parameter<int>("quad_pulses_per_meter", 0);
  this->declare_parameter<int>("quad_pulses_per_revolution", 0);
  this->declare_parameter<int>("vmin", 1);
  this->declare_parameter<int>("vtime", 2);
  this->declare_parameter<float>("wheel_radius", 0.0);
  this->declare_parameter<float>("wheel_separation", 0.0);
}

void MotorDriver::cmdVelCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg) const {
  double x_velocity =
      std::min(std::max((float)msg->linear.x, -max_linear_velocity_),
               max_linear_velocity_);
  double yaw_velocity =
      std::min(std::max((float)msg->angular.z, -max_angular_velocity_),
               max_angular_velocity_);
  if ((msg->linear.x == 0) && (msg->angular.z == 0)) {
    roboclaw_->doMixedSpeedDist(0, 0, 0, 0);
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
    roboclaw_->doMixedSpeedAccelDist(
        accel_quad_pulses_per_second_, m1_quad_pulses_per_second,
        m1_max_distance, m2_quad_pulses_per_second, m2_max_distance);
  }
}

void MotorDriver::onInit(rclcpp::Node::SharedPtr node) {
  node_ = node;
  this->get_parameter("accel_quad_pulses_per_second",
                      accel_quad_pulses_per_second_);
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
  this->get_parameter("quad_pulses_per_meter", quad_pulses_per_meter_);
  this->get_parameter("quad_pulses_per_revolution",
                      quad_pulses_per_revolution_);
  this->get_parameter("vmin", vmin_);
  this->get_parameter("vtime", vtime_);
  this->get_parameter("wheel_radius", wheel_radius_);
  this->get_parameter("wheel_separation", wheel_separation_);

  RCUTILS_LOG_INFO("accel_quad_pulses_per_second: %d",
                   accel_quad_pulses_per_second_);
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
  RCUTILS_LOG_INFO("quad_pulses_per_meter: %d", quad_pulses_per_meter_);
  RCUTILS_LOG_INFO("quad_pulses_per_revolution: %d",
                   quad_pulses_per_revolution_);
  RCUTILS_LOG_INFO("vmin: %d", vmin_);
  RCUTILS_LOG_INFO("vtime: %d", vtime_);
  RCUTILS_LOG_INFO("wheel_radius: %f", wheel_radius_);
  RCUTILS_LOG_INFO("wheel_separation: %f", wheel_separation_);

  RoboClaw::TPIDQ m1Pid = {m1_p_, m1_i_, m1_d_, m1_qpps_, m1_max_current_};
  RoboClaw::TPIDQ m2Pid = {m2_p_, m2_i_, m2_d_, m2_qpps_, m2_max_current_};

  roboclaw_ = new RoboClaw(m1Pid, m2Pid, m1_max_current_, m2_max_current_,
                           device_name_.c_str(), device_port_, vmin_, vtime_);
  RCUTILS_LOG_INFO("Main battery: %f", roboclaw_->getMainBatteryLevel());

  rclcpp::QoS bestEffortQos(10);
  bestEffortQos.keep_last(10);
  bestEffortQos.best_effort();
  bestEffortQos.durability_volatile();

  cmdVelSub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", bestEffortQos,
      std::bind(&MotorDriver::cmdVelCallback, this, std::placeholders::_1));
}

MotorDriver &MotorDriver::singleton() {
  if (!g_singleton) {
    g_singleton = new MotorDriver();
  }

  return *g_singleton;
}

MotorDriver *MotorDriver::g_singleton = nullptr;