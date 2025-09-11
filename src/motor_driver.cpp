// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Wimblerobotics
// https://github.com/wimblerobotics/ros2_roboclaw_driver

#include "ros2_roboclaw_driver/motor_driver.h"

#include <math.h>
#include <rcutils/logging_macros.h>
#include <stdint.h>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>

#include "ros2_roboclaw_driver/RoboClaw.h"
#include "ros2_roboclaw_driver/roboclaw_cmd_do_buffered_m1m2_drive_speed_accel_distance.h"
#include "ros2_roboclaw_driver/roboclaw_cmd_read_encoder_values.h"
#include "ros2_roboclaw_driver/roboclaw_cmd_read_logic_battery_voltage.h"
#include "ros2_roboclaw_driver/roboclaw_cmd_read_main_battery_voltage.h"
#include "ros2_roboclaw_driver/roboclaw_cmd_read_motor_currents.h"
#include "ros2_roboclaw_driver/roboclaw_cmd_read_motor_velocity_pidq.h"
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
  node.declare_parameter<bool>("publish_odom_tf", false);
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
  node.get_parameter("publish_odom_tf", publish_odom_tf_);
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

  // Precompute scaling factors (Option A distance scaling) with protection
  if (quad_pulses_per_meter_ > 0) {
    meters_per_pulse_ = 1.0 / static_cast<double>(quad_pulses_per_meter_);
  }
  if (quad_pulses_per_revolution_ > 0) {
    radians_per_pulse_ = (2.0 * M_PI) / static_cast<double>(quad_pulses_per_revolution_);
  }
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
  RCUTILS_LOG_INFO("publish_odom_tf: %s", publish_odom_tf_ ? "True" : "False");
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
    if (publish_odom_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
    }
  }

  // Start unified control loop timer (publishes and drives motors)
  auto timer_period = std::chrono::milliseconds(loop_sleep_ms_);
  control_timer_ = node_->create_wall_timer(timer_period, std::bind(&MotorDriver::controlLoopCallback, this));
}

MotorDriver& MotorDriver::singleton() {
  if (!g_singleton) {
    g_singleton = new MotorDriver();
  }

  return *g_singleton;
}

MotorDriver* MotorDriver::g_singleton = nullptr;

void MotorDriver::getFreshEncoders(uint32_t& encoder_left, uint32_t& encoder_right) {
  static uint64_t last_loop_count = 0;
  static uint32_t cached_encoder_left = 0;
  static uint32_t cached_encoder_right = 0;

  if (loop_iteration_count_ != last_loop_count) {
    // Only read once per control loop iteration
    auto* rc = roboclaw_;
    if (rc) {
      CmdReadEncoderValues cmd1 = CmdReadEncoderValues(*rc, cached_encoder_left, cached_encoder_right);
      cmd1.execute();
      last_loop_count = loop_iteration_count_;
    }
  }

  // Always return cached values
  encoder_left = cached_encoder_left;
  encoder_right = cached_encoder_right;
}

void MotorDriver::controlLoopCallback() {
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

  // Process cmd_vel commands
  processCmdVel();

  // Publish timing trackers
  static auto last_odom_pub = std::chrono::steady_clock::now();
  static auto last_joint_pub = std::chrono::steady_clock::now();
  static auto last_status_pub = std::chrono::steady_clock::now();
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
  static double joint_left_accum = 0.0;   // continuous wheel angle (rad)
  static double joint_right_accum = 0.0;  // continuous wheel angle (rad)

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
      getFreshEncoders(encoder_left_, encoder_right_);
      static int32_t last_left_encoder = 0;
      static int32_t last_right_encoder = 0;
      static bool encoders_initialized = false;
      if (!encoders_initialized) {
        last_left_encoder = encoder_left_;
        last_right_encoder = encoder_right_;
        encoders_initialized = true;
      }
      double dt = std::chrono::duration<double>(now_tp - last_joint_pub).count();
      if (dt <= 0.010f)
        return;

      sensor_msgs::msg::JointState js;
      js.header.stamp = clock->now();
      js.name = {"front_left_wheel", "front_right_wheel"};
      // Continuous accumulation using Option A scaling for distance -> radians via pulses->revs->radians
      // Use pulses to radians directly if available
      if (radians_per_pulse_ > 0.0) {
        joint_left_accum = encoder_left_ * radians_per_pulse_;
        joint_right_accum = encoder_right_ * radians_per_pulse_;
      }

      // Instantaneous wheel angular velocity from pulses/sec
      double wheel_left_rad_s = 0.0;
      double wheel_right_rad_s = 0.0;

      double velocity_left = (double)(encoder_left_ - last_left_encoder) / dt;
      double velocity_right = (double)(encoder_right_ - last_right_encoder) / dt;
      if (radians_per_pulse_ > 0.0) {
        wheel_left_rad_s = velocity_left * radians_per_pulse_;
        wheel_right_rad_s = velocity_right * radians_per_pulse_;
      }

      last_left_encoder = encoder_left_;
      last_right_encoder = encoder_right_;

      js.position.clear();
      js.position.push_back(joint_left_accum);
      js.position.push_back(joint_right_accum);
      js.velocity.clear();
      js.velocity.push_back(wheel_left_rad_s);
      js.velocity.push_back(wheel_right_rad_s);
      joint_state_publisher_->publish(js);
      last_joint_pub = now_tp;
    }

    // Odometry publish
    if (publish_odom_ && odom_publisher_ &&
        (std::chrono::duration<double>(now_tp - last_odom_pub).count() >= 1.0 / std::max(1, odom_rate_hz_))) {
      getFreshEncoders(encoder_left_, encoder_right_);
      static int32_t last_left_encoder = 0;
      static int32_t last_right_encoder = 0;
      static bool encoders_initialized = false;

      if (!encoders_initialized) {
        last_left_encoder = encoder_left_;
        last_right_encoder = encoder_right_;
        encoders_initialized = true;
      }

      // Actual dt
      double dt = std::chrono::duration<double>(now_tp - last_odom_pub).count();
      // if (dt <= 0.0) dt = 1.0 / std::max(1, odom_rate_hz_);
      if (dt <= 0.010f)
        return;

      int32_t delta_left = encoder_left_ - last_left_encoder;
      int32_t delta_right = encoder_right_ - last_right_encoder;
      last_left_encoder = encoder_left_;
      last_right_encoder = encoder_right_;

      // Distances using Option A scaling
      double dist_left = delta_left * meters_per_pulse_;
      double dist_right = delta_right * meters_per_pulse_;

      double delta_distance = (dist_left + dist_right) / 2.0;
      double delta_theta = (dist_right - dist_left) / wheel_separation_;

      current_pose.x += delta_distance * cos(current_pose.theta + delta_theta / 2.0);
      current_pose.y += delta_distance * sin(current_pose.theta + delta_theta / 2.0);
      current_pose.theta += delta_theta;
      // Normalize
      current_pose.theta = std::atan2(std::sin(current_pose.theta), std::cos(current_pose.theta));

      // Update velocity
      current_velocity.linear_x = delta_distance / dt;
      current_velocity.angular_z = delta_theta / dt;

      // // Instantaneous (from wheel velocities) vs integrated
      // double v_left = 0.0, v_right = 0.0;
      // if (meters_per_pulse_ > 0.0) {
      //   // velocity_* is pulses/sec; convert to m/s
      //   v_left = velocity_left_ * meters_per_pulse_;
      //   v_right = velocity_right_ * meters_per_pulse_;
      // }
      // current_velocity.linear_x = (v_left + v_right) / 2.0;
      // current_velocity.angular_z = (v_right - v_left) / wheel_separation_;

      nav_msgs::msg::Odometry odom;
      odom.header.stamp = clock->now();
      odom.header.frame_id = "odom";
      odom.child_frame_id = "base_link";
      odom.pose.pose.position.x = current_pose.x;
      odom.pose.pose.position.y = current_pose.y;
      odom.pose.pose.position.z = 0.0;
      double half_yaw = current_pose.theta / 2.0;
      odom.pose.pose.orientation.x = 0.0;
      odom.pose.pose.orientation.y = 0.0;
      odom.pose.pose.orientation.z = std::sin(half_yaw);
      odom.pose.pose.orientation.w = std::cos(half_yaw);
      odom.twist.twist.linear.x = current_velocity.linear_x;
      odom.twist.twist.linear.y = 0.0;
      odom.twist.twist.angular.z = current_velocity.angular_z;
      // {  // Convert quaternion to Euler angle (yaw)
      //   tf2::Quaternion q(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z,
      //                     odom.pose.pose.orientation.w);
      //   double roll, pitch, yaw;
      //   tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      //   double euler_z = yaw;
      //   RCUTILS_LOG_INFO("Odom: x=%.3f y=%.3f yaw=%.3f lin_x=%.3f ang_z=%.3f, encoder_left:%d encoder_right:%d",
      //                    current_pose.x, current_pose.y, euler_z, current_velocity.linear_x,
      //                    current_velocity.angular_z, encoder_left_, encoder_right_);
      // }

      // Improved covariance (simple heuristic)
      // Position covariance grows modestly with motion; yaw higher when rotating
      double lin_var = 0.02;                                                // m^2 baseline
      double yaw_var = 0.05 + 0.1 * std::fabs(current_velocity.angular_z);  // rad^2
      std::fill(odom.pose.covariance.begin(), odom.pose.covariance.end(), 0.0);
      odom.pose.covariance[0] = lin_var;   // x
      odom.pose.covariance[7] = lin_var;   // y
      odom.pose.covariance[35] = yaw_var;  // yaw
      std::fill(odom.twist.covariance.begin(), odom.twist.covariance.end(), 0.0);
      odom.twist.covariance[0] = lin_var;   // vx
      odom.twist.covariance[35] = yaw_var;  // wyaw

      odom_publisher_->publish(odom);

      // TF
      if (publish_odom_tf_ && tf_broadcaster_) {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = odom.header.stamp;
        tf_msg.header.frame_id = "odom";
        tf_msg.child_frame_id = "base_link";
        tf_msg.transform.translation.x = current_pose.x;
        tf_msg.transform.translation.y = current_pose.y;
        tf_msg.transform.translation.z = 0.0;
        tf_msg.transform.rotation = odom.pose.pose.orientation;
        tf_broadcaster_->sendTransform(tf_msg);
      }

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
        getFreshEncoders(encoder_left_, encoder_right_);
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
