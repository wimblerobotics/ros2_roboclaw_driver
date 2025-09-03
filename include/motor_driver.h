// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2025 Michael Wimble. https://github.com/wimblerobotics/ros2_roboclaw_driver
#pragma once

#include <atomic>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <mutex>  // Include mutex header
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <thread>

#include "roboclaw.h"
#include "ros2_roboclaw_driver/msg/robo_claw_status.hpp"

struct CachedCmdVel {
  geometry_msgs::msg::Twist twist;
  std::chrono::steady_clock::time_point stamp;
  uint64_t seq{0};
};

class MotorDriver : public rclcpp::Node {
 public:
  MotorDriver();
  static MotorDriver& singleton();

  void onInit(rclcpp::Node::SharedPtr node);
  std::pair<int32_t, int32_t> getEncodersForStatus();

  // Accessors for cached PID (used by status publishing)
  const RoboClaw::TPIDQ& getCachedM1Pid() const {
    return cached_m1_pid_;
  }
  const RoboClaw::TPIDQ& getCachedM2Pid() const {
    return cached_m2_pid_;
  }
  rclcpp::Publisher<ros2_roboclaw_driver::msg::RoboClawStatus>::SharedPtr getStatusPublisher()
      const {
    return status_publisher_;
  }
  void setStatusPublisher(
      rclcpp::Publisher<ros2_roboclaw_driver::msg::RoboClawStatus>::SharedPtr pub) {
    status_publisher_ = pub;
  }

  void processCmdVel();

 private:
  void declareParameters();
  void initializeParameters();
  void logParameters() const;
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void publisherThread();
  void setupStatsTimer();
  void controlLoop();  // New unified high-rate loop (replaces IoExecutor + publisherThread)

  // Odometry methods
  void integrateOdometry();
  double normalizeAngle(double angle);
  std::pair<int32_t, int32_t> getEncodersForOdometry();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<ros2_roboclaw_driver::msg::RoboClawStatus>::SharedPtr status_publisher_;
  // publisher_thread_ removed; functionality moved to controlLoop

  int accel_quad_pulses_per_second_;
  int baud_rate_;
  std::string device_name_;
  int device_port_;
  bool do_debug_;
  bool do_low_level_debug_;
  float m1_p_;
  float m1_i_;
  float m1_d_;
  int m1_qpps_;
  float m1_max_current_;
  float m2_p_;
  float m2_i_;
  float m2_d_;
  int m2_qpps_;
  float m2_max_current_;
  float max_angular_velocity_;
  float max_linear_velocity_;
  float max_seconds_uncommanded_travel_;
  bool publish_joint_states_;
  bool publish_odom_;
  int quad_pulses_per_meter_;
  float quad_pulses_per_revolution_;
  float sensor_update_rate_;  // Hz
  float wheel_radius_;
  float wheel_separation_;

  // High-rate loop config
  int loop_sleep_ms_{1};
  int odom_rate_hz_{50};
  int joint_state_rate_hz_{50};
  int status_rate_hz_{1};
  int retry_count_{3};
  int retry_quiet_ms_{10};
  int command_resend_period_ms_{50};  // keepalive resend period
  double small_velocity_threshold_{0.001};

  // Safety parameters
  float max_runaway_seconds_{0.5f};
  float max_runaway_linear_velocity_{0.2f};
  float max_runaway_angular_velocity_{0.5f};
  bool log_each_cmd_vel_{true};

  // Cached latest cmd_vel
  std::shared_ptr<CachedCmdVel> latest_cmd_vel_;  // single writer (callback) + single reader (loop)
  uint64_t next_cmd_vel_seq_{1};
  uint64_t last_processed_seq_{0};
  // Metrics
  uint64_t cmd_processed_count_{0};
  uint64_t cmd_missed_count_{0};
  double cmd_latency_ema_ms_{0.0};
  double cmd_latency_max_ms_{0.0};
  std::chrono::steady_clock::time_point last_cmd_metrics_log_{};
  std::chrono::steady_clock::time_point last_motor_command_send_time_{};
  double last_sent_x_{0.0};
  double last_sent_yaw_{0.0};
  std::chrono::steady_clock::time_point last_processed_seq_time_{};
  std::chrono::steady_clock::time_point prev_processed_seq_time_{};
  // Cached PID values (loaded at init, reused in status publishing)
  RoboClaw::TPIDQ cached_m1_pid_{};
  RoboClaw::TPIDQ cached_m2_pid_{};

  // Loop frequency diagnostics
  uint64_t loop_iteration_count_{0};
  std::chrono::steady_clock::time_point last_loop_freq_log_{};
  std::thread control_loop_thread_;
  // Track last processed cmd_vel stamp to avoid resending identical commands
  std::chrono::steady_clock::time_point last_processed_cmd_vel_stamp_{};

  // Incremental sensor polling state & cache
  int incremental_sensor_index_{0};
  uint32_t encoder_left_{0};
  uint32_t encoder_right_{0};
  int32_t velocity_left_{0};
  int32_t velocity_right_{0};
  struct MotorCurrentsCache {
    float m1Current = 0.0f;
    float m2Current = 0.0f;
  } motor_currents_;
  float logic_voltage_{0.0f};
  float main_voltage_{0.0f};
  float temperature_{0.0f};
  uint32_t status_bits_{0};

  static MotorDriver* g_singleton;
};
