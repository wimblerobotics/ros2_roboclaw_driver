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

class MotorDriver {
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

 private:
  RoboClaw* roboclaw_;
  void declareParameters(rclcpp::Node& node);
  void initializeParameters(rclcpp::Node& node);
  void validateRequiredParametersOrDie();
  void logParameters() const;
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void processCmdVel();
  void publisherThread();
  void setupStatsTimer();
  void controlLoopCallback();  // Timer callback for unified high-rate loop
  void getFreshEncoders(uint32_t& encoder_left_, uint32_t& encoder_right_,
                        uint8_t& encoder_left_status_, uint8_t& encoder_right_status_);

  // Odometry methods
  void integrateOdometry();
  double normalizeAngle(double angle);
  std::pair<int32_t, int32_t> getEncodersForOdometry();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<ros2_roboclaw_driver::msg::RoboClawStatus>::SharedPtr status_publisher_;
  rclcpp::TimerBase::SharedPtr control_timer_;  // Timer for control loop

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
  int loop_sleep_ms_{20};  // 50Hz control rate
  int odom_rate_hz_{50};
  int joint_state_rate_hz_{50};
  int status_rate_hz_{20};
  int retry_count_{3};
  int retry_quiet_ms_{10};
  double small_velocity_threshold_{0.001};

  // Safety parameters
  float max_runaway_seconds_{0.5f};
  float max_runaway_linear_velocity_{0.2f};
  float max_runaway_angular_velocity_{0.5f};
  bool log_each_cmd_vel_{true};

  // Cached latest cmd_vel
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
  // Track last processed cmd_vel stamp to avoid resending identical commands
  std::chrono::steady_clock::time_point last_processed_cmd_vel_stamp_{};

  // Incremental sensor polling state & cache
  int incremental_sensor_index_{1};  // 1..4 used for non-encoder groups
  // Status data collection state machine (6 states: encoders, velocities, currents, logic_bat,
  // main_bat, temp_status)
  enum StatusDataState {
    ENCODERS_SPEED = 0,
    MOTOR_CURRENTS = 1,
    LOGIC_BATTERY = 2,
    MAIN_BATTERY = 3,
    TEMPERATURE = 4,
    STATUS_BITS = 5
  };
  StatusDataState status_data_state_{ENCODERS_SPEED};
  std::chrono::steady_clock::time_point last_status_data_collection_{};
  double status_data_interval_ms_{0.0};  // Will be calculated as status_rate period / 6
  // Status data collection flags (encoders always read for odom/joint_states)
  uint32_t encoder_left_{0};
  uint8_t encoder_left_status_{0};
  uint32_t encoder_right_{0};
  uint8_t encoder_right_status_{0};
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

  typedef struct CachedCmdVel {
    geometry_msgs::msg::Twist twist;
    std::chrono::steady_clock::time_point stamp;
    uint64_t seq{0};
    mutable std::mutex mutex;  // Mutex for thread-safe access
  } CachedCmdVel;

  static CachedCmdVel cached_cmd_vel_;

  static MotorDriver* g_singleton;
};
