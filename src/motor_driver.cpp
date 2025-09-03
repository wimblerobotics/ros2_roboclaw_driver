// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2025 Michael Wimble. https://github.com/wimblerobotics/ros2_roboclaw_driver

#include "motor_driver.h"

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

#include "roboclaw.h"
#include "roboclaw_cmd_do_buffered_m1m2_drive_speed_accel_distance.h"

MotorDriver::MotorDriver()
    : Node("motor_driver_node"),
      device_name_("foo_bar"),
      wheel_radius_(0.10169),
      wheel_separation_(0.345) {
  declareParameters();
  initializeParameters();
}

void MotorDriver::declareParameters() {
  this->declare_parameter<int>("accel_quad_pulses_per_second", 600);
  this->declare_parameter<int>("baud_rate", 38400);
  this->declare_parameter<std::string>("device_name", "roboclaw");
  this->declare_parameter<int>("device_port", 123);
  this->declare_parameter<bool>("do_debug", false);
  this->declare_parameter<bool>("do_low_level_debug", false);
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
  this->declare_parameter<float>("sensor_update_rate", 20.0);  // Hz
  this->declare_parameter<float>("wheel_radius", 0.0);
  this->declare_parameter<float>("wheel_separation", 0.0);
  // New loop & retry parameters
  this->declare_parameter<int>("loop_sleep_ms", 1);
  this->declare_parameter<int>("odom_rate_hz", 50);
  this->declare_parameter<int>("joint_state_rate_hz", 50);
  this->declare_parameter<int>("status_rate_hz", 1);
  this->declare_parameter<int>("retry_count", 3);
  this->declare_parameter<int>("retry_quiet_ms", 10);
  this->declare_parameter<bool>("log_each_cmd_vel", true);
  // Safety parameters
  this->declare_parameter<float>("max_runaway_seconds", 0.5f);
  this->declare_parameter<float>("max_runaway_linear_velocity", 0.2f);
  this->declare_parameter<float>("max_runaway_angular_velocity", 0.5f);
}

void MotorDriver::initializeParameters() {
  this->get_parameter("accel_quad_pulses_per_second", accel_quad_pulses_per_second_);
  this->get_parameter("baud_rate", baud_rate_);
  this->get_parameter("device_name", device_name_);
  this->get_parameter("device_port", device_port_);
  this->get_parameter("do_debug", do_debug_);
  this->get_parameter("do_low_level_debug", do_low_level_debug_);
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
  this->get_parameter("max_angular_velocity", max_angular_velocity_);
  this->get_parameter("max_linear_velocity", max_linear_velocity_);
  this->get_parameter("max_seconds_uncommanded_travel", max_seconds_uncommanded_travel_);
  this->get_parameter("publish_joint_states", publish_joint_states_);
  this->get_parameter("publish_odom", publish_odom_);
  this->get_parameter("quad_pulses_per_meter", quad_pulses_per_meter_);
  this->get_parameter("quad_pulses_per_revolution", quad_pulses_per_revolution_);
  this->get_parameter("sensor_update_rate", sensor_update_rate_);
  this->get_parameter("wheel_radius", wheel_radius_);
  this->get_parameter("wheel_separation", wheel_separation_);
  // New loop & retry params
  this->get_parameter("loop_sleep_ms", loop_sleep_ms_);
  this->get_parameter("odom_rate_hz", odom_rate_hz_);
  this->get_parameter("joint_state_rate_hz", joint_state_rate_hz_);
  this->get_parameter("status_rate_hz", status_rate_hz_);
  this->get_parameter("retry_count", retry_count_);
  this->get_parameter("retry_quiet_ms", retry_quiet_ms_);
  this->get_parameter("log_each_cmd_vel", log_each_cmd_vel_);
  // Safety
  this->get_parameter("max_runaway_seconds", max_runaway_seconds_);
  this->get_parameter("max_runaway_linear_velocity", max_runaway_linear_velocity_);
  this->get_parameter("max_runaway_angular_velocity", max_runaway_angular_velocity_);

  logParameters();
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
  RCUTILS_LOG_INFO("log_each_cmd_vel: %s", log_each_cmd_vel_ ? "True" : "False");
}

void MotorDriver::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  static std::chrono::steady_clock::time_point last_cb_time;
  static uint64_t cb_count = 0;
  static double cb_interval_ema_ms = 0.0;
  static double cb_interval_max_ms = 0.0;
  static std::chrono::steady_clock::time_point last_metrics_log;

  if (!last_metrics_log.time_since_epoch().count())
    last_metrics_log = std::chrono::steady_clock::now();

  if (RoboClaw::singleton() != nullptr) {
    auto now = std::chrono::steady_clock::now();
    if (cb_count > 0) {
      double dt_ms = std::chrono::duration<double, std::milli>(now - last_cb_time).count();
      if (cb_interval_ema_ms == 0.0)
        cb_interval_ema_ms = dt_ms;
      else
        cb_interval_ema_ms = 0.9 * cb_interval_ema_ms + 0.1 * dt_ms;
      cb_interval_max_ms = std::max(cb_interval_max_ms, dt_ms);
    }
    last_cb_time = now;
    cb_count++;

    // Cache latest cmd_vel for high-rate control loop
    auto cached = std::make_shared<CachedCmdVel>();
    cached->twist = *msg;
    cached->stamp = now;
    cached->seq = next_cmd_vel_seq_++;
    latest_cmd_vel_ = cached;  // simple store

    // Periodic metrics log (every ~1s)
    if (std::chrono::duration<double>(now - last_metrics_log).count() >= 1.0) {
      double avg_hz = cb_interval_ema_ms > 0 ? 1000.0 / cb_interval_ema_ms : 0.0;
      RCUTILS_LOG_INFO(
          "[cmd_vel stats] count=%lu ema_dt=%.2f ms max_dt=%.2f ms approx_rate=%.1f Hz",
          (unsigned long)cb_count, cb_interval_ema_ms, cb_interval_max_ms, avg_hz);
      cb_interval_max_ms = 0.0;  // reset max each interval window
      last_metrics_log = now;
    }
  }
}

void MotorDriver::onInit(rclcpp::Node::SharedPtr node) {
  node_ = node;
  initializeParameters();

  RoboClaw::TPIDQ m1Pid = {m1_p_, m1_i_, m1_d_, (uint32_t)m1_qpps_, m1_max_current_};
  RoboClaw::TPIDQ m2Pid = {m2_p_, m2_i_, m2_d_, (uint32_t)m2_qpps_, m2_max_current_};

  new RoboClaw(m1Pid, m2Pid, m1_max_current_, m2_max_current_, device_name_.c_str(), device_port_,
               baud_rate_, do_debug_, do_low_level_debug_);
  // Configure retry behavior on RoboClaw singleton
  if (RoboClaw::singleton()) {
    RoboClaw::singleton()->setRetryParams(retry_count_, retry_quiet_ms_);
    cached_m1_pid_ = m1Pid;
    cached_m2_pid_ = m2Pid;
  }

  // Allow some time for initial sensor readings
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Initial battery read (direct)
  try {
    RCUTILS_LOG_INFO("Main battery: %f", RoboClaw::singleton()->getMainBatteryLevel());
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
    joint_state_publisher_ =
        this->create_publisher<sensor_msgs::msg::JointState>("joint_states", qos);
  }

  if (publish_odom_) {
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", qos);
  }

  // Start unified control loop (publishes and drives motors)
  control_loop_thread_ = std::thread(&MotorDriver::controlLoop, this);
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

void MotorDriver::controlLoop() {
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  auto next_time = std::chrono::steady_clock::now();
  // Publish timing trackers
  auto last_odom_pub = std::chrono::steady_clock::now();
  auto last_joint_pub = last_odom_pub;
  auto last_status_pub = last_odom_pub;
  static int32_t last_left_encoder = 0;
  static int32_t last_right_encoder = 0;
  static bool encoders_initialized = false;
  struct Pose2D {
    float x = 0;
    float y = 0;
    float theta = 0;
  } current_pose;
  struct Velocity2D {
    float linear_x = 0;
    float angular_z = 0;
  } current_velocity;

  while (rclcpp::ok()) {
    next_time += std::chrono::milliseconds(loop_sleep_ms_);

    auto* rc = RoboClaw::singleton();
    if (rc) {
      // Incremental sensor polling (one piece per loop)
      try {
        switch (incremental_sensor_index_) {
          case 0:
            encoder_left_ = rc->getM1Encoder();
            break;
          case 1:
            encoder_right_ = rc->getM2Encoder();
            break;
            // case 2:
            //   velocity_left_ = rc->getVelocity(RoboClaw::kM1);
            //   break;
            // case 3:
            //   velocity_right_ = rc->getVelocity(RoboClaw::kM2);
            //   break;
            // case 4: {
            //   auto c = rc->getMotorCurrents();
            //   motor_currents_.m1Current = c.m1Current;
            //   motor_currents_.m2Current = c.m2Current;
            // } break;
            // case 5:
            //   logic_voltage_ = rc->getLogicBatteryLevel();
            //   break;
            // case 6:
            //   main_voltage_ = rc->getMainBatteryLevel();
            //   break;
            // case 7:
            //   temperature_ = rc->getTemperature();
            //   break;
            // case 8:
            //   status_bits_ = rc->getErrorStatus();
            //   break;
            // default:
            //   break;
        }
        incremental_sensor_index_ = (incremental_sensor_index_ + 1) % 9;
      } catch (...) {
      }

      // Process each new cmd_vel exactly once (always send command, even tiny velocities)
      auto cached = latest_cmd_vel_;
      if (cached && cached->seq != last_processed_seq_) {
        prev_processed_seq_time_ = last_processed_seq_time_;
        last_processed_seq_time_ = std::chrono::steady_clock::now();
        if (cached->seq > last_processed_seq_ + 1) {
          cmd_missed_count_ += (cached->seq - (last_processed_seq_ + 1));
        }
        last_processed_seq_ = cached->seq;
        cmd_processed_count_++;
        double latency_ms = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - cached->stamp)
                                .count();
        if (cmd_latency_ema_ms_ == 0.0)
          cmd_latency_ema_ms_ = latency_ms;
        else
          cmd_latency_ema_ms_ = 0.9 * cmd_latency_ema_ms_ + 0.1 * latency_ms;
        cmd_latency_max_ms_ = std::max(cmd_latency_max_ms_, latency_ms);
        const double x_velocity =
            std::clamp((double)cached->twist.linear.x, -(double)max_linear_velocity_,
                       (double)max_linear_velocity_);
        const double yaw_velocity =
            std::clamp((double)cached->twist.angular.z, -(double)max_angular_velocity_,
                       (double)max_angular_velocity_);
        const double m1_desired_velocity =
            x_velocity - (yaw_velocity * wheel_separation_ / 2.0) / wheel_radius_;
        const double m2_desired_velocity =
            x_velocity + (yaw_velocity * wheel_separation_ / 2.0) / wheel_radius_;
        const int32_t m1_qpps = (int32_t)(m1_desired_velocity * quad_pulses_per_meter_);
        const int32_t m2_qpps = (int32_t)(m2_desired_velocity * quad_pulses_per_meter_);
        const int32_t m1_max_distance = (int32_t)fabs(m1_qpps * max_seconds_uncommanded_travel_);
        const int32_t m2_max_distance = (int32_t)fabs(m2_qpps * max_seconds_uncommanded_travel_);
        try {
          CmdDoBufferedM1M2DriveSpeedAccelDistance c(*rc, accel_quad_pulses_per_second_, m1_qpps,
                                                     m1_max_distance, m2_qpps, m2_max_distance);
          c.execute();

          // rc->doMixedSpeedAccelDist(accel_quad_pulses_per_second_, m1_qpps, m1_max_distance,
          //                           m2_qpps, m2_max_distance);
          last_motor_command_send_time_ = std::chrono::steady_clock::now();
          last_sent_x_ = x_velocity;
          last_sent_yaw_ = yaw_velocity;
          if (log_each_cmd_vel_) {
            double inter_arrival_ms = 0.0;
            if (prev_processed_seq_time_.time_since_epoch().count()) {
              inter_arrival_ms = std::chrono::duration<double, std::milli>(
                                     last_processed_seq_time_ - prev_processed_seq_time_)
                                     .count();
            }
            RCUTILS_LOG_INFO(
                "[cmd_vel handle] seq=%llu latency=%.2fms ia=%.2fms x=%.4f yaw=%.4f m1_qpps=%d "
                "m2_qpps=%d dist=(%d,%d)",
                (unsigned long long)last_processed_seq_, latency_ms, inter_arrival_ms, x_velocity,
                yaw_velocity, m1_qpps, m2_qpps, m1_max_distance, m2_max_distance);
          }
        } catch (...) {
        }
        auto now_metrics = std::chrono::steady_clock::now();
        if (!last_cmd_metrics_log_.time_since_epoch().count())
          last_cmd_metrics_log_ = now_metrics;
        if (std::chrono::duration<double>(now_metrics - last_cmd_metrics_log_).count() >= 1.0) {
          double processed_rate = cmd_latency_ema_ms_ > 0 ? 1000.0 / cmd_latency_ema_ms_ : 0.0;
          RCUTILS_LOG_INFO(
              "[cmd_vel proc] last_seq=%llu processed=%llu missed=%llu lat_ema=%.2fms "
              "lat_max=%.2fms est_rate=%.1fHz",
              (unsigned long long)last_processed_seq_, (unsigned long long)cmd_processed_count_,
              (unsigned long long)cmd_missed_count_, cmd_latency_ema_ms_, cmd_latency_max_ms_,
              processed_rate);
          cmd_latency_max_ms_ = 0.0;
          last_cmd_metrics_log_ = now_metrics;
        }
      }

      auto now_tp = std::chrono::steady_clock::now();
      // Joint states publish
      if (publish_joint_states_ && joint_state_publisher_ &&
          (std::chrono::duration<double>(now_tp - last_joint_pub).count() >=
           1.0 / std::max(1, joint_state_rate_hz_))) {
        sensor_msgs::msg::JointState js;
        js.header.stamp = clock->now();
        js.name = {"front_left_wheel", "front_right_wheel"};
        double radians_left =
            fmod((encoder_left_ / quad_pulses_per_revolution_) * 2.0 * M_PI, 2.0 * M_PI);
        double radians_right =
            fmod((encoder_right_ / quad_pulses_per_revolution_) * 2.0 * M_PI, 2.0 * M_PI);
        double velocity_left_rad_s = velocity_left_ / wheel_radius_;
        double velocity_right_rad_s = velocity_right_ / wheel_radius_;
        js.position = {radians_left, radians_right};
        js.velocity = {velocity_left_rad_s, velocity_right_rad_s};
        joint_state_publisher_->publish(js);
        last_joint_pub = now_tp;
      }

      // Odometry publish
      if (publish_odom_ && odom_publisher_ &&
          (std::chrono::duration<double>(now_tp - last_odom_pub).count() >=
           1.0 / std::max(1, odom_rate_hz_))) {
        if (!encoders_initialized) {
          last_left_encoder = (int32_t)rc->getM1Encoder();
          last_right_encoder = (int32_t)rc->getM2Encoder();
          encoders_initialized = true;
        }
        int32_t current_left_encoder = (int32_t)rc->getM1Encoder();
        int32_t current_right_encoder = (int32_t)rc->getM2Encoder();
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
        current_velocity.linear_x = delta_distance * std::max(1, odom_rate_hz_);  // approx
        current_velocity.angular_z = delta_theta * std::max(1, odom_rate_hz_);
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
      if (status_publisher_ && (std::chrono::duration<double>(now_tp - last_status_pub).count() >=
                                1.0 / std::max(1, status_rate_hz_))) {
        ros2_roboclaw_driver::msg::RoboClawStatus msg;
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
          msg.m1_current_speed = rc->getVelocity(RoboClaw::kM1);
          msg.m2_current_speed = rc->getVelocity(RoboClaw::kM2);
          auto c = rc->getMotorCurrents();
          msg.m1_motor_current = c.m1Current;
          msg.m2_motor_current = c.m2Current;
          msg.m1_encoder_value = rc->getM1Encoder();
          msg.m1_encoder_status = rc->getM1EncoderStatus();
          msg.m2_encoder_value = rc->getM2Encoder();
          msg.m2_encoder_status = rc->getM2EncoderStatus();
          msg.main_battery_voltage = rc->getMainBatteryLevel();
          msg.logic_battery_voltage = rc->getLogicBatteryLevel();
          msg.temperature = rc->getTemperature();
          msg.error_status = rc->getErrorStatus();
          msg.error_string = rc->getErrorString();
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
          loop_iteration_count_ /
          std::max(1.0, std::chrono::duration<double>(now_loop - last_loop_freq_log_).count());
      RCUTILS_LOG_INFO("[control_loop] freq=%.1f Hz", hz);
      loop_iteration_count_ = 0;
      last_loop_freq_log_ = now_loop;
    }
    std::this_thread::sleep_until(next_time);
  }
}
