// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2025 Michael Wimble.
// https://github.com/wimblerobotics/ros2_roboclaw_driver

#include "ros2_roboclaw_driver/motor_driver.h"

#include <math.h>
#include <rcutils/logging_macros.h>
#include <stdint.h>

#include <algorithm>
#include <chrono>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <iomanip>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>

#include "ros2_roboclaw_driver/RoboClaw.h"
#include "ros2_roboclaw_driver/roboclaw_cmd_do_buffered_m1m2_drive_speed_accel_distance.h"
#include "ros2_roboclaw_driver/roboclaw_cmd_read_encoder.h"
#include "ros2_roboclaw_driver/roboclaw_cmd_read_encoder_speed.h"
#include "ros2_roboclaw_driver/roboclaw_cmd_read_logic_battery_voltage.h"
#include "ros2_roboclaw_driver/roboclaw_cmd_read_main_battery_voltage.h"
#include "ros2_roboclaw_driver/roboclaw_cmd_read_motor_currents.h"
#include "ros2_roboclaw_driver/roboclaw_cmd_read_motor_velocity_pidq.h"
#include "ros2_roboclaw_driver/roboclaw_cmd_read_speed_m1.h"
#include "ros2_roboclaw_driver/roboclaw_cmd_read_status.h"
#include "ros2_roboclaw_driver/roboclaw_cmd_read_temperature.h"

MotorDriver::MotorDriver() : device_name_("foo_bar"), wheel_radius_(0.10169), wheel_separation_(0.345) {}

void MotorDriver::declareParameters(rclcpp::Node& node) {
  node.declare_parameter<int>("accel_quad_pulses_per_second", 600);
  node.declare_parameter<int>("baud_rate", 38400);
  node.declare_parameter<std::string>("device_name", "roboclaw");
  node.declare_parameter<int>("device_port", 123);
  node.declare_parameter<bool>("do_debug", false);
  node.declare_parameter<bool>("do_low_level_debug", false);
  node.declare_parameter<float>("m1_p", 0.0);
  node.declare_parameter<float>("m1_i", 0.0);
  node.declare_parameter<float>("m1_d", 0.0);
  node.declare_parameter<int>("m1_qpps", 0);
  node.declare_parameter<float>("m1_max_current", 0.0);
  node.declare_parameter<float>("m2_p", 0.0);
  node.declare_parameter<float>("m2_i", 0.0);
  node.declare_parameter<float>("m2_d", 0.0);
  node.declare_parameter<int>("m2_qpps", 0);
  node.declare_parameter<float>("max_angular_velocity", 0.0);
  node.declare_parameter<float>("max_linear_velocity", 0.0);
  node.declare_parameter<float>("m2_max_current", 0.0);
  node.declare_parameter<float>("max_seconds_uncommanded_travel", 0.0);
  node.declare_parameter<bool>("publish_joint_states", true);
  node.declare_parameter<bool>("publish_odom", true);
  node.declare_parameter<int>("quad_pulses_per_meter", 0);
  node.declare_parameter<float>("quad_pulses_per_revolution", 0);
  node.declare_parameter<float>("sensor_update_rate", 20.0);
  node.declare_parameter<float>("wheel_radius", 0.0);
  node.declare_parameter<float>("wheel_separation", 0.0);
  node.declare_parameter<int>("loop_sleep_ms", 1);
  node.declare_parameter<int>("odom_rate_hz", 50);
  node.declare_parameter<int>("joint_state_rate_hz", 50);
  node.declare_parameter<int>("status_rate_hz", 20);
  node.declare_parameter<int>("retry_count", 3);
  node.declare_parameter<int>("retry_quiet_ms", 10);
  node.declare_parameter<bool>("log_each_cmd_vel", true);
  node.declare_parameter<float>("max_runaway_seconds", 0.5f);
  node.declare_parameter<float>("max_runaway_linear_velocity", 0.2f);
  node.declare_parameter<float>("max_runaway_angular_velocity", 0.5f);
  node.declare_parameter<int>("cmd_resend_period_ms", 200);
}

void MotorDriver::initializeParameters(rclcpp::Node& node) {
  node.get_parameter("accel_quad_pulses_per_second", accel_quad_pulses_per_second_);
  node.get_parameter("baud_rate", baud_rate_);
  node.get_parameter("device_name", device_name_);
  node.get_parameter("device_port", device_port_);
  node.get_parameter("do_debug", do_debug_);
  node.get_parameter("do_low_level_debug", do_low_level_debug_);
  node.get_parameter("m1_p", m1_p_);
  node.get_parameter("m1_i", m1_i_);
  node.get_parameter("m1_d", m1_d_);
  node.get_parameter("m1_qpps", m1_qpps_);
  node.get_parameter("m1_max_current", m1_max_current_);
  node.get_parameter("m2_p", m2_p_);
  node.get_parameter("m2_i", m2_i_);
  node.get_parameter("m2_d", m2_d_);
  node.get_parameter("m2_qpps", m2_qpps_);
  node.get_parameter("m2_max_current", m2_max_current_);
  node.get_parameter("max_angular_velocity", max_angular_velocity_);
  node.get_parameter("max_linear_velocity", max_linear_velocity_);
  node.get_parameter("max_seconds_uncommanded_travel", max_seconds_uncommanded_travel_);
  node.get_parameter("publish_joint_states", publish_joint_states_);
  node.get_parameter("publish_odom", publish_odom_);
  node.get_parameter("quad_pulses_per_meter", quad_pulses_per_meter_);
  node.get_parameter("quad_pulses_per_revolution", quad_pulses_per_revolution_);
  node.get_parameter("sensor_update_rate", sensor_update_rate_);
  node.get_parameter("wheel_radius", wheel_radius_);
  node.get_parameter("wheel_separation", wheel_separation_);
  node.get_parameter("loop_sleep_ms", loop_sleep_ms_);
  node.get_parameter("odom_rate_hz", odom_rate_hz_);
  node.get_parameter("joint_state_rate_hz", joint_state_rate_hz_);
  node.get_parameter("status_rate_hz", status_rate_hz_);
  node.get_parameter("retry_count", retry_count_);
  node.get_parameter("retry_quiet_ms", retry_quiet_ms_);
  node.get_parameter("max_runaway_seconds", max_runaway_seconds_);
  node.get_parameter("max_runaway_linear_velocity", max_runaway_linear_velocity_);
  node.get_parameter("max_runaway_angular_velocity", max_runaway_angular_velocity_);
  logParameters();
}

void MotorDriver::validateRequiredParametersOrDie() {
  bool ok = true;
  if (device_name_.empty() || device_name_ == "roboclaw") {
    RCUTILS_LOG_FATAL("Required parameter 'device_name' not loaded from config (got '%s').", device_name_.c_str());
    ok = false;
  }
  if (quad_pulses_per_meter_ <= 0) {
    RCUTILS_LOG_FATAL("Required parameter 'quad_pulses_per_meter' must be > 0 (got %d).", quad_pulses_per_meter_);
    ok = false;
  }
  if (wheel_radius_ <= 0.0f) {
    RCUTILS_LOG_FATAL("Required parameter 'wheel_radius' must be > 0 (got %f).", wheel_radius_);
    ok = false;
  }
  if (wheel_separation_ <= 0.0f) {
    RCUTILS_LOG_FATAL("Required parameter 'wheel_separation' must be > 0 (got %f).", wheel_separation_);
    ok = false;
  }
  if (!ok) {
    // Force immediate shutdown so we don't run with unsafe defaults.
    rclcpp::shutdown();
    throw std::runtime_error("Missing required parameters");
  }
}

void MotorDriver::logParameters() const {
  RCUTILS_LOG_INFO("accel_quad_pulses_per_second: %d", accel_quad_pulses_per_second_);
  RCUTILS_LOG_INFO("baud_rate: %d", baud_rate_);
  RCUTILS_LOG_INFO("device_name: %s", device_name_.c_str());
  RCUTILS_LOG_INFO("device_port: %d", device_port_);
  RCUTILS_LOG_INFO("do_debug: %s", do_debug_ ? "True" : "False");
  RCUTILS_LOG_INFO("do_low_level_debug: %s", do_low_level_debug_ ? "True" : "False");
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
  RCUTILS_LOG_INFO("max_seconds_uncommanded_travel: %f", max_seconds_uncommanded_travel_);
  RCUTILS_LOG_INFO("publish_joint_states: %s", publish_joint_states_ ? "True" : "False");
  RCUTILS_LOG_INFO("publish_odom: %s", publish_odom_ ? "True" : "False");
  RCUTILS_LOG_INFO("quad_pulses_per_meter: %d", quad_pulses_per_meter_);
  RCUTILS_LOG_INFO("quad_pulses_per_revolution: %3.4f", quad_pulses_per_revolution_);
  RCUTILS_LOG_INFO("sensor_update_rate: %f", sensor_update_rate_);
  RCUTILS_LOG_INFO("wheel_radius: %f", wheel_radius_);
  RCUTILS_LOG_INFO("wheel_separation: %f", wheel_separation_);
  RCUTILS_LOG_INFO("loop_sleep_ms: %d", loop_sleep_ms_);
  RCUTILS_LOG_INFO("odom_rate_hz: %d", odom_rate_hz_);
  RCUTILS_LOG_INFO("joint_state_rate_hz: %d", joint_state_rate_hz_);
  RCUTILS_LOG_INFO("status_rate_hz: %d", status_rate_hz_);
  RCUTILS_LOG_INFO("retry_count: %d", retry_count_);
  RCUTILS_LOG_INFO("retry_quiet_ms: %d", retry_quiet_ms_);
  RCUTILS_LOG_INFO("max_runaway_seconds: %f", max_runaway_seconds_);
  RCUTILS_LOG_INFO("max_runaway_linear_velocity: %f", max_runaway_linear_velocity_);
  RCUTILS_LOG_INFO("max_runaway_angular_velocity: %f", max_runaway_angular_velocity_);
}

void MotorDriver::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  cached_cmd_vel_.mutex.lock();
  cached_cmd_vel_.twist = *msg;
  cached_cmd_vel_.stamp = std::chrono::steady_clock::now();
  cached_cmd_vel_.seq++;

  cached_cmd_vel_.mutex.unlock();
}

void MotorDriver::processCmdVel() {
  cached_cmd_vel_.mutex.lock();
  bool should_send_command = false;
  double latency_ms = 0.0;

  // Check if we have a new cmd_vel to process
  if (cached_cmd_vel_.seq != last_processed_seq_) {
    should_send_command = true;
    if (cached_cmd_vel_.seq > last_processed_seq_ + 1) {
      cmd_missed_count_ += (cached_cmd_vel_.seq - (last_processed_seq_ + 1));
    }
    last_processed_seq_ = cached_cmd_vel_.seq;
    cmd_processed_count_++;
    latency_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - cached_cmd_vel_.stamp).count();
    if (cmd_latency_ema_ms_ == 0.0)
      cmd_latency_ema_ms_ = latency_ms;
    else
      cmd_latency_ema_ms_ = 0.9 * cmd_latency_ema_ms_ + 0.1 * latency_ms;
    cmd_latency_max_ms_ = std::max(cmd_latency_max_ms_, latency_ms);
  }

  if (should_send_command) {
    const double x_velocity =
        std::clamp((double)cached_cmd_vel_.twist.linear.x, -(double)max_linear_velocity_, (double)max_linear_velocity_);
    const double yaw_velocity = std::clamp((double)cached_cmd_vel_.twist.angular.z, -(double)max_angular_velocity_,
                                           (double)max_angular_velocity_);
    const double m1_desired_velocity = x_velocity - (yaw_velocity * wheel_separation_ / 2.0) / wheel_radius_;
    const double m2_desired_velocity = x_velocity + (yaw_velocity * wheel_separation_ / 2.0) / wheel_radius_;
    const int32_t m1_qpps = (int32_t)(m1_desired_velocity * quad_pulses_per_meter_);
    const int32_t m2_qpps = (int32_t)(m2_desired_velocity * quad_pulses_per_meter_);
    const int32_t m1_max_distance = (int32_t)fabs(m1_qpps * max_seconds_uncommanded_travel_);
    const int32_t m2_max_distance = (int32_t)fabs(m2_qpps * max_seconds_uncommanded_travel_);
    try {
      CmdDoBufferedM1M2DriveSpeedAccelDistance cmd = CmdDoBufferedM1M2DriveSpeedAccelDistance(
          *roboclaw_, accel_quad_pulses_per_second_, m1_qpps, m1_max_distance, m2_qpps, m2_max_distance);
      cmd.execute();

      int32_t speed = 0;
      CmdReadEncoderSpeed cmd_m1_read_encoder_speed(*roboclaw_, RoboClaw::kM1, speed);
      cmd_m1_read_encoder_speed.execute();
      // CmdReadSpeedM1 cmd1(*roboclaw_, speed);
      // cmd1.execute();
      RCUTILS_LOG_DEBUG("M1 speed: %d", speed);
    } catch (const std::exception& ex) {
      RCUTILS_LOG_ERROR("Failed to send motor command for cmd_vel: %s", ex.what());
    } catch (...) {
      RCUTILS_LOG_ERROR("Failed to send motor command for cmd_vel: unknown exception");
    }

    auto now_metrics = std::chrono::steady_clock::now();
    if (!last_cmd_metrics_log_.time_since_epoch().count())
      last_cmd_metrics_log_ = now_metrics;
    if (std::chrono::duration<double>(now_metrics - last_cmd_metrics_log_).count() >= 1.0) {
      double processed_rate = cmd_latency_ema_ms_ > 0 ? 1000.0 / cmd_latency_ema_ms_ : 0.0;
      RCUTILS_LOG_INFO(
          "[cmd_vel proc] last_seq=%llu processed=%llu missed=%llu "
          "lat_ema=%.2fms "
          "lat_max=%.2fms est_rate=%.1fHz",
          (unsigned long long)last_processed_seq_, (unsigned long long)cmd_processed_count_,
          (unsigned long long)cmd_missed_count_, cmd_latency_ema_ms_, cmd_latency_max_ms_, processed_rate);
      cmd_latency_max_ms_ = 0.0;
      last_cmd_metrics_log_ = now_metrics;
    }
  }  // end of if (should_send_command)

  cached_cmd_vel_.mutex.unlock();
}

void MotorDriver::onInit(rclcpp::Node::SharedPtr node) {
  node_ = node;
  declareParameters(*node_);
  initializeParameters(*node_);
  validateRequiredParametersOrDie();

  RoboClaw::TPIDQ m1Pid = {m1_p_, m1_i_, m1_d_, (uint32_t)m1_qpps_, m1_max_current_};
  RoboClaw::TPIDQ m2Pid = {m2_p_, m2_i_, m2_d_, (uint32_t)m2_qpps_, m2_max_current_};

  roboclaw_ = new RoboClaw(m1Pid, m2Pid, m1_max_current_, m2_max_current_, device_name_.c_str(), device_port_,
                           baud_rate_, do_debug_, do_low_level_debug_);
  // Configure retry behavior on RoboClaw singleton
  if (roboclaw_) {
    roboclaw_->setRetryParams(retry_count_, retry_quiet_ms_);
    cached_m1_pid_ = m1Pid;
    cached_m2_pid_ = m2Pid;
  }

  // Initial battery read (direct)
  try {
    CmdReadMainBatteryVoltage cmd = CmdReadMainBatteryVoltage(*roboclaw_, main_voltage_);
    cmd.execute();
    RCUTILS_LOG_INFO("Main battery: %f", main_voltage_);
  } catch (...) {
    RCUTILS_LOG_ERROR("Initial battery read failed");
  }

  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10));
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  qos.durability(rclcpp::DurabilityPolicy::Volatile);
  qos.avoid_ros_namespace_conventions(false);

  cmdVelSub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", qos, std::bind(&MotorDriver::cmdVelCallback, this, std::placeholders::_1));

  if (publish_joint_states_) {
    joint_state_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", qos);
  }

  if (publish_odom_) {
    odom_publisher_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", qos);
  }

  // Start unified control loop timer (publishes and drives motors)
  auto timer_period = std::chrono::milliseconds(loop_sleep_ms_);
  control_timer_ = node_->create_wall_timer(timer_period, std::bind(&MotorDriver::controlLoopCallback, this));
  // setupStatsTimer(); //###
}

// publisherThread removed (functionality moved into controlLoop)

MotorDriver& MotorDriver::singleton() {
  if (!g_singleton) {
    g_singleton = new MotorDriver();
  }

  return *g_singleton;
}

MotorDriver* MotorDriver::g_singleton = nullptr;

void MotorDriver::getFreshEncoders(uint32_t& encoder_left_, uint32_t& encoder_right_, uint8_t& encoder_left_status_,
                                   uint8_t& encoder_right_status_) {
  RoboClaw::EncodeResult encoder_result;
  static uint64_t last_loop_count = 0;
  static uint32_t cached_encoder_left = 0;
  static uint32_t cached_encoder_right = 0;
  static uint8_t cached_encoder_left_status = 0;
  static uint8_t cached_encoder_right_status = 0;

  if (loop_iteration_count_ != last_loop_count) {
    // Only read once per control loop iteration
    auto* rc = roboclaw_;
    if (rc) {
      CmdReadEncoder cmd1 = CmdReadEncoder(*rc, RoboClaw::kM1, encoder_result);
      cmd1.execute();
      cached_encoder_left = encoder_result.value;
      cached_encoder_left_status = encoder_result.status;
      CmdReadEncoder cmd2 = CmdReadEncoder(*rc, RoboClaw::kM2, encoder_result);
      cmd2.execute();
      cached_encoder_right = encoder_result.value;
      cached_encoder_right_status = encoder_result.status;
      last_loop_count = loop_iteration_count_;
    }
  }

  // Always return cached values
  encoder_left_ = cached_encoder_left;
  encoder_right_ = cached_encoder_right;
  encoder_left_status_ = cached_encoder_left_status;
  encoder_right_status_ = cached_encoder_right_status;
}

void MotorDriver::controlLoopCallback() {
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

  // Process queued commands with TeensyV2-style filtering (highest priority)
  // processQueuedCommands();
  processCmdVel();

  // Publish timing trackers
  static auto last_odom_pub = std::chrono::steady_clock::now();
  static auto last_joint_pub = std::chrono::steady_clock::now();
  static auto last_status_pub = std::chrono::steady_clock::now();
  static int32_t last_left_encoder = 0;
  static int32_t last_right_encoder = 0;
  static bool encoders_initialized = false;
  static bool status_interval_initialized = false;
  static struct Pose2D {
    float x = 0;
    float y = 0;
    float theta = 0;
  } current_pose;
  static struct Velocity2D {
    float linear_x = 0;
    float angular_z = 0;
  } current_velocity;

  // Initialize status data collection interval (1/6 of status publish period)
  if (!status_interval_initialized) {
    status_data_interval_ms_ = (1000.0 / std::max(1, status_rate_hz_)) / 6.0;
    status_interval_initialized = true;
  }

  // Control loop body (executed once per timer callback)
  auto* rc = roboclaw_;
  if (rc) {
    // Process cmd_vel immediately when available (highest priority)
    // processCmdVel();
    auto now_tp = std::chrono::steady_clock::now();

    // State machine for status data collection (spread over status publish
    // period)
    if (!last_status_data_collection_.time_since_epoch().count()) {
      last_status_data_collection_ = now_tp;
    }

    if (std::chrono::duration<double>(now_tp - last_status_data_collection_).count() * 1000.0 >=
        status_data_interval_ms_) {
      try {
        switch (status_data_state_) {
          case ENCODERS_SPEED: {
            CmdReadEncoderSpeed cmd1 = CmdReadEncoderSpeed(*rc, RoboClaw::kM1, velocity_left_);
            cmd1.execute();
            CmdReadEncoderSpeed cmd2 = CmdReadEncoderSpeed(*rc, RoboClaw::kM2, velocity_right_);
            cmd2.execute();
            break;
          }
          case MOTOR_CURRENTS: {
            RoboClaw::TMotorCurrents motor_currents;
            CmdReadMotorCurrents cmd = CmdReadMotorCurrents(*rc, motor_currents);
            cmd.execute();
            motor_currents_.m1Current = motor_currents.m1Current;
            motor_currents_.m2Current = motor_currents.m2Current;
            break;
          }
          case LOGIC_BATTERY: {
            CmdReadLogicBatteryVoltage cmd_logic = CmdReadLogicBatteryVoltage(*rc, logic_voltage_);
            cmd_logic.execute();
            break;
          }
          case MAIN_BATTERY: {
            CmdReadMainBatteryVoltage cmd_main = CmdReadMainBatteryVoltage(*rc, main_voltage_);
            cmd_main.execute();
            break;
          }
          case TEMPERATURE: {
            CmdReadTemperature cmd_temp = CmdReadTemperature(*rc, temperature_);
            cmd_temp.execute();
            break;
          }
          case STATUS_BITS: {
            CmdReadStatus cmd_status = CmdReadStatus(*rc, status_bits_);
            cmd_status.execute();
            break;
          }
        }
        // Advance to next state
        status_data_state_ = static_cast<StatusDataState>((status_data_state_ + 1) % 6);
        last_status_data_collection_ = now_tp;
      } catch (...) {
        // If command fails, still advance state to prevent getting stuck
        status_data_state_ = static_cast<StatusDataState>((status_data_state_ + 1) % 6);
      }
    }

    // Joint states publish
    if (publish_joint_states_ && joint_state_publisher_ &&
        (std::chrono::duration<double>(now_tp - last_joint_pub).count() >= 1.0 / std::max(1, joint_state_rate_hz_))) {
      getFreshEncoders(encoder_left_, encoder_right_, encoder_left_status_, encoder_right_status_);
      sensor_msgs::msg::JointState js;
      js.header.stamp = clock->now();
      js.name = {"front_left_wheel", "front_right_wheel"};
      double radians_left = fmod((encoder_left_ / quad_pulses_per_revolution_) * 2.0 * M_PI, 2.0 * M_PI);
      double radians_right = fmod((encoder_right_ / quad_pulses_per_revolution_) * 2.0 * M_PI, 2.0 * M_PI);
      double velocity_left_rad_s = velocity_left_ / wheel_radius_;
      double velocity_right_rad_s = velocity_right_ / wheel_radius_;
      js.position = {radians_left, radians_right};
      js.velocity = {velocity_left_rad_s, velocity_right_rad_s};
      joint_state_publisher_->publish(js);
      last_joint_pub = now_tp;
    }

    // Odometry publish
    if (publish_odom_ && odom_publisher_ &&
        (std::chrono::duration<double>(now_tp - last_odom_pub).count() >= 1.0 / std::max(1, odom_rate_hz_))) {
      getFreshEncoders(encoder_left_, encoder_right_, encoder_left_status_, encoder_right_status_);
      if (!encoders_initialized) {
        last_left_encoder = encoder_left_;
        last_right_encoder = encoder_right_;
        encoders_initialized = true;
      }
      int32_t current_left_encoder = encoder_left_;
      int32_t current_right_encoder = encoder_right_;
      int32_t delta_encoder_m1 = current_left_encoder - last_left_encoder;
      int32_t delta_encoder_m2 = current_right_encoder - last_right_encoder;
      last_left_encoder = current_left_encoder;
      last_right_encoder = current_right_encoder;
      float wheel_circumference = (float)M_PI * wheel_radius_ * 2.0f;
      float dist_m1 = (delta_encoder_m1 / quad_pulses_per_revolution_) * wheel_circumference;
      float dist_m2 = (delta_encoder_m2 / quad_pulses_per_revolution_) * wheel_circumference;
      float delta_distance = (dist_m1 + dist_m2) / 2.0f;
      float delta_theta = (dist_m2 - dist_m1) / wheel_separation_;
      current_pose.x += delta_distance * cos(current_pose.theta + delta_theta / 2.0f);
      current_pose.y += delta_distance * sin(current_pose.theta + delta_theta / 2.0f);
      current_pose.theta += delta_theta;
      while (current_pose.theta > M_PI)
        current_pose.theta -= 2.0f * M_PI;
      while (current_pose.theta < -M_PI)
        current_pose.theta += 2.0f * M_PI;
      current_velocity.linear_x = delta_distance * std::max(1,
                                                            odom_rate_hz_);  // approx current_velocity.angular_z =
                                                                             // delta_theta *
      std::max(1, odom_rate_hz_);
      nav_msgs::msg::Odometry odom;
      odom.header.stamp = clock->now();
      odom.header.frame_id = "base_link";
      float half_theta = current_pose.theta / 2.0f;
      float q[4];
      q[0] = cos(half_theta);
      q[1] = 0;
      q[2] = 0;
      q[3] = sin(half_theta);
      odom.pose.pose.position.x = current_pose.x;
      odom.pose.pose.position.y = current_pose.y;
      odom.pose.pose.position.z = 0;
      odom.pose.pose.orientation.x = q[1];
      odom.pose.pose.orientation.y = q[2];
      odom.pose.pose.orientation.z = q[3];
      odom.pose.pose.orientation.w = q[0];
      odom.twist.twist.linear.x = current_velocity.linear_x;
      odom.twist.twist.angular.z = current_velocity.angular_z;
      odom_publisher_->publish(odom);
      last_odom_pub = now_tp;
    }

    // Status publish (now integrated in control loop)
    if (status_publisher_ &&
        (std::chrono::duration<double>(now_tp - last_status_pub).count() >= 1.0 / std::max(1, status_rate_hz_))) {
      // Ensure we have fresh encoder data for status message
      // (Get fresh data if neither joint states nor odom published in this
      // cycle)
      bool joint_states_published =
          publish_joint_states_ && joint_state_publisher_ &&
          (std::chrono::duration<double>(now_tp - last_joint_pub).count() >= 1.0 / std::max(1, joint_state_rate_hz_));
      bool odom_published =
          publish_odom_ && odom_publisher_ &&
          (std::chrono::duration<double>(now_tp - last_odom_pub).count() >= 1.0 / std::max(1, odom_rate_hz_));

      if (!joint_states_published && !odom_published) {
        getFreshEncoders(encoder_left_, encoder_right_, encoder_left_status_, encoder_right_status_);
      }

      ros2_roboclaw_driver::msg::RoboClawStatus msg;
      msg.header.stamp = clock->now();
      const auto& m1 = cached_m1_pid_;
      const auto& m2 = cached_m2_pid_;
      msg.m1_p = m1.p;
      msg.m1_i = m1.i;
      msg.m1_d = m1.d;
      msg.m1_qpps = m1.qpps;
      msg.m2_p = m2.p;
      msg.m2_i = m2.i;
      msg.m2_d = m2.d;
      msg.m2_qpps = m2.qpps;
      try {
        msg.m1_current_speed = velocity_left_;
        msg.m2_current_speed = velocity_right_;
        msg.m1_motor_current = motor_currents_.m1Current;
        msg.m2_motor_current = motor_currents_.m2Current;
        msg.m1_encoder_value = encoder_left_;
        msg.m1_encoder_status = encoder_left_status_;
        msg.m2_encoder_value = encoder_right_;
        msg.m2_encoder_status = encoder_right_status_;
        msg.main_battery_voltage = main_voltage_;
        msg.logic_battery_voltage = logic_voltage_;
        msg.temperature = temperature_;
        msg.error_status = status_bits_;
        char error_buffer[256];
        rc->decodeErrorStatus(status_bits_, error_buffer, sizeof(error_buffer));
        msg.error_string = error_buffer;
        status_publisher_->publish(msg);
      } catch (...) {
      }
      last_status_pub = now_tp;
    }
  }

  // Loop frequency diagnostics (log ~1Hz)
  loop_iteration_count_++;
  auto now_loop = std::chrono::steady_clock::now();
  if (!last_loop_freq_log_.time_since_epoch().count())
    last_loop_freq_log_ = now_loop;
  if (std::chrono::duration<double>(now_loop - last_loop_freq_log_).count() >= 1.0) {
    double hz =
        loop_iteration_count_ / std::max(1.0, std::chrono::duration<double>(now_loop - last_loop_freq_log_).count());
    RCUTILS_LOG_INFO("[control_loop] freq=%.1f Hz", hz);
    loop_iteration_count_ = 0;
    last_loop_freq_log_ = now_loop;
  }
}

MotorDriver::CachedCmdVel MotorDriver::cached_cmd_vel_;
