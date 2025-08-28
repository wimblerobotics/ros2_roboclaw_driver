// MIT License
// Author: Michael Wimble <mike@wimblerobotics.com>
#include "roboclaw_driver/driver_node.hpp"

#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <chrono>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "roboclaw_driver/params.hpp"
#include "roboclaw_driver/roboclaw_device.hpp"

namespace roboclaw_driver {

  namespace {
    inline double nowSec() { return rclcpp::Clock().now().seconds(); }
  }  // namespace

  DriverNode::DriverNode(const rclcpp::NodeOptions& opts) : rclcpp::Node("roboclaw_driver", opts) {
    declareParameters();
    initObjects();
    auto qos = rclcpp::SystemDefaultsQoS();
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", qos, std::bind(&DriverNode::cmdVelCallback, this, std::placeholders::_1));
    status_pub_ = create_publisher<ros2_roboclaw_driver::msg::RoboClawStatus>("roboclaw/status", 10);
    estop_pub_ = create_publisher<ros2_roboclaw_driver::msg::RoboClawEStop>("roboclaw/estop", 10);
    estop_cmd_sub_ = create_subscription<ros2_roboclaw_driver::msg::RoboClawEStop>(
      "roboclaw/estop_command", 10,
      std::bind(&DriverNode::estopMsgCallback, this, std::placeholders::_1));
    reset_enc_srv_ = create_service<ros2_roboclaw_driver::srv::ResetEncoders>(
      "roboclaw/reset_encoders",
      std::bind(&DriverNode::onResetEncoders, this, std::placeholders::_1, std::placeholders::_2));
    clear_all_estops_srv_ = create_service<ros2_roboclaw_driver::srv::ClearAllEStops>(
      "roboclaw/clear_all_estops",
      std::bind(&DriverNode::onClearAllEStops, this, std::placeholders::_1, std::placeholders::_2));
    clear_estop_source_srv_ = create_service<ros2_roboclaw_driver::srv::ClearEStopSource>(
      "roboclaw/clear_estop_source", std::bind(&DriverNode::onClearEStopSource, this,
        std::placeholders::_1, std::placeholders::_2));
    sensor_timer_ =
      create_wall_timer(std::chrono::milliseconds(50), std::bind(&DriverNode::sensorTimer, this));
    status_timer_ =
      create_wall_timer(std::chrono::seconds(1), std::bind(&DriverNode::statusTimer, this));
    last_cmd_time_ = nowSec();
    param_cb_handle_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& params) { return this->onParamSet(params); });
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
  }

  void DriverNode::declareParameters() {
    using namespace params;
    params::Defaults d;
    declare_parameter<std::string>(kPort, "/dev/ttyACM0");
    declare_parameter<int>(kBaudRate, d.baud_rate);
    declare_parameter<int>(kDeviceAddress, d.device_address);
    declare_parameter<double>(kAccelQpps, d.accel_qpps);
    declare_parameter<double>(kMaxLinearVel, d.max_linear_vel);
    declare_parameter<double>(kMaxAngularVel, d.max_angular_vel);
    declare_parameter<double>(kCmdTimeout, d.cmd_timeout);
    // Motor PID and QPPS parameters
    declare_parameter<double>(kM1P, 7.26239);
    declare_parameter<double>(kM1I, 2.43);
    declare_parameter<double>(kM1D, 0.0);
    declare_parameter<int>(kM1QPPS, 2437);
    declare_parameter<double>(kM2P, 7.26239);
    declare_parameter<double>(kM2I, 1.28767);
    declare_parameter<double>(kM2D, 0.0);
    declare_parameter<int>(kM2QPPS, 2437);
    declare_parameter<double>(kM1MaxCurrent, 6.0);
    declare_parameter<double>(kM2MaxCurrent, 6.0);
    declare_parameter<bool>(kPublishOdom, d.publish_odom);
    declare_parameter<bool>(kPublishJointStates, d.publish_joint_states);
    declare_parameter<bool>(kPublishTF, d.publish_tf);
    declare_parameter<double>(kWheelRadius, d.wheel_radius);
    declare_parameter<double>(kWheelSeparation, d.wheel_separation);
    declare_parameter<double>(kStatusRate, d.status_rate);
    declare_parameter<double>(kSensorPollRate, d.sensor_poll_rate);
    declare_parameter<double>(kOdomRate, d.odom_rate);
    // Pulses/encoder scaling (previously optional)
    declare_parameter<int>(kQuadPulsesPerMeter, 0);
    declare_parameter<double>(kQuadPulsesPerRev, 0.0);
    // Safety core switches
    declare_parameter<bool>(kSafetyEnabled, d.safety_enabled);
    declare_parameter<double>(kOverCurrentLimitM1, d.overcurrent_limit_m1);
    declare_parameter<double>(kOverCurrentLimitM2, d.overcurrent_limit_m2);
    declare_parameter<double>(kOverCurrentDetectTime, d.overcurrent_detect_time);
    declare_parameter<double>(kOverCurrentClearTime, d.overcurrent_clear_time);
    declare_parameter<double>(kOverCurrentHysteresis, d.overcurrent_hysteresis);
    declare_parameter<double>(kTemp1Limit, d.temp1_limit);
    declare_parameter<double>(kTemp2Limit, d.temp2_limit);
    declare_parameter<double>(kTempClearDelta, d.temp_clear_delta);
    declare_parameter<double>(kRunawaySpeedFactor, d.runaway_speed_factor);
    declare_parameter<double>(kRunawayDetectTime, d.runaway_detect_time);
    declare_parameter<double>(kStallSpeedRatio, d.stall_speed_ratio);
    declare_parameter<double>(kStallMinCommand, d.stall_min_command);
    declare_parameter<double>(kStallTimeout, d.stall_timeout);
    declare_parameter<bool>(kEstopAutoClear, d.estop_auto_clear);
    declare_parameter<double>(kEstopRepeatWindow, d.estop_repeat_window);
    declare_parameter<int>(kEstopRepeatLimit, d.estop_repeat_limit);
    // Frames & joints
    declare_parameter<std::string>(params::kFrameOdom, "odom");
    declare_parameter<std::string>(params::kFrameBase, "base_link");
    declare_parameter<std::string>(params::kLeftJointName, d.left_joint);
    declare_parameter<std::string>(params::kRightJointName, d.right_joint);
    // Odometry covariance
    declare_parameter<double>(params::kOdomLinearCov, d.odom_linear_cov);
    declare_parameter<double>(params::kOdomAngularCov, d.odom_angular_cov);
  }

  void DriverNode::initObjects() {
    // Log all configuration parameters for debugging
    RCLCPP_INFO(this->get_logger(), "=== RoboClaw Driver Configuration ===");

    // Device configuration
    std::string port = get_parameter(params::kPort).as_string();
    int baud = get_parameter(params::kBaudRate).as_int();
    int addr = get_parameter(params::kDeviceAddress).as_int();
    RCLCPP_INFO(this->get_logger(), "Device: port=%s, baud=%d, address=%d", port.c_str(), baud, addr);

    // Motion parameters
    double accel_qpps = get_parameter(params::kAccelQpps).as_double();
    double max_linear = get_parameter(params::kMaxLinearVel).as_double();
    double max_angular = get_parameter(params::kMaxAngularVel).as_double();
    double cmd_timeout = get_parameter(params::kCmdTimeout).as_double();
    RCLCPP_INFO(this->get_logger(), "Motion: accel=%.1f qpps, max_linear=%.3f m/s, max_angular=%.3f rad/s, timeout=%.3f s",
      accel_qpps, max_linear, max_angular, cmd_timeout);

    // PID parameters
    double m1_p = get_parameter(params::kM1P).as_double();
    double m1_i = get_parameter(params::kM1I).as_double();
    double m1_d = get_parameter(params::kM1D).as_double();
    int m1_qpps = get_parameter(params::kM1QPPS).as_int();
    double m2_p = get_parameter(params::kM2P).as_double();
    double m2_i = get_parameter(params::kM2I).as_double();
    double m2_d = get_parameter(params::kM2D).as_double();
    int m2_qpps = get_parameter(params::kM2QPPS).as_int();
    RCLCPP_INFO(this->get_logger(), "M1 PID: P=%.5f, I=%.5f, D=%.5f, QPPS=%d", m1_p, m1_i, m1_d, m1_qpps);
    RCLCPP_INFO(this->get_logger(), "M2 PID: P=%.5f, I=%.5f, D=%.5f, QPPS=%d", m2_p, m2_i, m2_d, m2_qpps);

    // Physical parameters
    wheel_radius_ = get_parameter(params::kWheelRadius).as_double();
    wheel_separation_ = get_parameter(params::kWheelSeparation).as_double();
    pulses_per_meter_ = get_parameter(params::kQuadPulsesPerMeter).as_int();
    pulses_per_rev_ = get_parameter(params::kQuadPulsesPerRev).as_double();
    RCLCPP_INFO(this->get_logger(), "Robot: wheel_radius=%.5f m, wheel_sep=%.3f m", wheel_radius_, wheel_separation_);
    RCLCPP_INFO(this->get_logger(), "Encoder scaling: %d pulses/meter, %.2f pulses/revolution", pulses_per_meter_, pulses_per_rev_);

    // Publishing configuration
    publish_odom_ = get_parameter(params::kPublishOdom).as_bool();
    publish_joint_ = get_parameter(params::kPublishJointStates).as_bool();
    publish_tf_ = get_parameter(params::kPublishTF).as_bool();
    double sensor_rate = get_parameter(params::kSensorPollRate).as_double();
    double status_rate = get_parameter(params::kStatusRate).as_double();
    double odom_rate = get_parameter(params::kOdomRate).as_double();
    RCLCPP_INFO(this->get_logger(), "Publishing: odom=%s, joints=%s, tf=%s",
      publish_odom_ ? "yes" : "no", publish_joint_ ? "yes" : "no", publish_tf_ ? "yes" : "no");
    RCLCPP_INFO(this->get_logger(), "Rates: sensor=%.1f Hz, status=%.1f Hz, odom=%.1f Hz",
      sensor_rate, status_rate, odom_rate);

    // Safety configuration
    bool safety_enabled = get_parameter(params::kSafetyEnabled).as_bool();
    double m1_max_current = get_parameter(params::kM1MaxCurrent).as_double();
    double m2_max_current = get_parameter(params::kM2MaxCurrent).as_double();
    RCLCPP_INFO(this->get_logger(), "Safety: enabled=%s, M1_max_current=%.1fA, M2_max_current=%.1fA",
      safety_enabled ? "yes" : "no", m1_max_current, m2_max_current);

    RCLCPP_INFO(this->get_logger(), "=====================================");

    odom_.configure(wheel_radius_, wheel_separation_, pulses_per_rev_ > 0 ? pulses_per_rev_ : 1.0);

    // Replace transport/protocol/hardware with direct device
    try {
      roboclaw_dev_ = std::make_unique<RoboClawDevice>(port, baud, static_cast<uint8_t>(addr));

      // MANDATORY: Test device communication by reading firmware version first
      std::string version = roboclaw_dev_->version();
      if (version.empty()) {
        RCLCPP_ERROR(this->get_logger(), "CRITICAL: RoboClaw device failed to respond to version command");
        RCLCPP_ERROR(this->get_logger(), "Check device address (%d), baud rate (%d), and connections", addr, baud);
        throw std::runtime_error("RoboClaw version check failed - device not responding properly");
      }

      RCLCPP_INFO(this->get_logger(), "RoboClaw device connected successfully!");
      RCLCPP_INFO(this->get_logger(), "  Firmware version: %s", version.c_str());

      // Initialize motor PID and QPPS settings
      std::string err;
      RCLCPP_INFO(this->get_logger(), "Setting motor PID parameters...");

      if (!roboclaw_dev_->setPID(1, m1_p, m1_i, m1_d, static_cast<uint32_t>(m1_qpps), err)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set M1 PID parameters: %s", err.c_str());
        throw std::runtime_error("Failed to configure M1 PID settings");
      }

      if (!roboclaw_dev_->setPID(2, m2_p, m2_i, m2_d, static_cast<uint32_t>(m2_qpps), err)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set M2 PID parameters: %s", err.c_str());
        throw std::runtime_error("Failed to configure M2 PID settings");
      }

      RCLCPP_INFO(this->get_logger(), "Motor PID parameters configured successfully");

      // Reset encoders to clear any random initial values
      if (!roboclaw_dev_->resetEncoders(err)) {
        RCLCPP_WARN(this->get_logger(), "Failed to reset encoders: %s", err.c_str());
      } else {
        RCLCPP_INFO(this->get_logger(), "Encoders reset to zero");
      }

      // Capture initial device state for safety systems
      Snapshot initial_snap{};
      if (roboclaw_dev_->readSnapshot(initial_snap, err)) {
        RCLCPP_INFO(this->get_logger(), "Initial device state captured:");
        RCLCPP_INFO(this->get_logger(), "  M1 encoder: %u, M2 encoder: %u",
          initial_snap.m1_enc.value, initial_snap.m2_enc.value);
        RCLCPP_INFO(this->get_logger(), "  Main voltage: %.1fV, Logic voltage: %.1fV",
          initial_snap.volts.main, initial_snap.volts.logic);
        RCLCPP_INFO(this->get_logger(), "  Temperature1: %.1f°C, Temperature2: %.1f°C",
          initial_snap.temps.t1, initial_snap.temps.t2);
        RCLCPP_INFO(this->get_logger(), "  Status bits: 0x%08X", initial_snap.status_bits);
      } else {
        RCLCPP_WARN(this->get_logger(), "Failed to read initial device state: %s", err.c_str());
      }

    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize RoboClaw device: %s", e.what());
      roboclaw_dev_.reset();
      throw; // Re-throw to prevent node from starting with invalid device
    }

    if (publish_odom_) odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    if (publish_joint_)
      joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  }

  void DriverNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    last_cmd_time_ = nowSec();
    double v = msg->linear.x;
    double w = msg->angular.z;

    // Apply velocity limits
    double max_linear = get_parameter(params::kMaxLinearVel).as_double();
    double max_angular = get_parameter(params::kMaxAngularVel).as_double();
    v = std::clamp(v, -max_linear, max_linear);
    w = std::clamp(w, -max_angular, max_angular);

    double v_left = v - w * wheel_separation_ / 2.0;
    double v_right = v + w * wheel_separation_ / 2.0;
    int32_t m1_qpps = 0;
    int32_t m2_qpps = 0;
    if (pulses_per_meter_ > 0) {
      m1_qpps = static_cast<int32_t>(v_left * pulses_per_meter_);
      m2_qpps = static_cast<int32_t>(v_right * pulses_per_meter_);
    }

    // Calculate maximum safe distance based on timeout and current speed
    double cmd_timeout = get_parameter(params::kCmdTimeout).as_double();
    double accel_qpps = get_parameter(params::kAccelQpps).as_double();

    // Calculate maximum distance robot can travel in timeout period
    // Distance = current_speed * time + 0.5 * accel * time^2 (assuming deceleration to stop)
    double max_speed_mps = std::max(std::abs(v_left), std::abs(v_right));
    uint32_t max_distance_pulses = 0;
    if (pulses_per_meter_ > 0 && max_speed_mps > 0) {
      // Distance to stop: v²/(2*a) + safety margin
      double stopping_distance = (max_speed_mps * max_speed_mps) / (2.0 * (accel_qpps / pulses_per_meter_));
      // Add distance for timeout period
      double timeout_distance = max_speed_mps * cmd_timeout;
      max_distance_pulses = static_cast<uint32_t>((stopping_distance + timeout_distance) * pulses_per_meter_);
    }

    // Minimum reasonable distance to prevent immediate stops
    if (max_distance_pulses < 100) {
      max_distance_pulses = 100;
    }

    // Send buffered command with distance limiting for safety
    std::string err;
    if (roboclaw_dev_) {
      bool ok = roboclaw_dev_->driveSpeedsAccelDistance(
        m1_qpps, m2_qpps,
        static_cast<uint32_t>(accel_qpps),
        max_distance_pulses,
        err);
      if (!ok) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "Failed to send buffered drive command: %s", err.c_str());
      } else {
        RCLCPP_DEBUG(this->get_logger(),
          "Sent buffered cmd: M1=%d, M2=%d qpps, accel=%u, max_dist=%u pulses",
          m1_qpps, m2_qpps, static_cast<uint32_t>(accel_qpps), max_distance_pulses);
      }
    }
  }

  void DriverNode::sensorTimer() {
    double age = nowSec() - last_cmd_time_;
    double timeout = get_parameter(params::kCmdTimeout).as_double();
    if (age > timeout) {
      std::string err;
      if (roboclaw_dev_) {
        bool ok = roboclaw_dev_->driveSpeeds(0, 0, err);
        if (!ok) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Failed to send stop command: %s", err.c_str());
        }
      }
    }

    std::string err;
    if (!roboclaw_dev_) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "RoboClaw device not initialized");
      return;
    }

    Snapshot snap{};
    if (!roboclaw_dev_->readSnapshot(snap, err)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Failed to read RoboClaw snapshot: %s", err.c_str());
      return;
    }

    // Update EMA current values (simplified - no actual EMA for now)
    current_ema_m1_ = snap.currents.m1_inst;
    current_ema_m2_ = snap.currents.m2_inst;
    latest_snapshot_ = snap;

    // Log device data occasionally for debugging
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "RoboClaw data - M1 enc: %u, M2 enc: %u, Main V: %.1f, Logic V: %.1f",
      snap.m1_enc.value, snap.m2_enc.value, snap.volts.main, snap.volts.logic);
  }

  void DriverNode::statusTimer() {
    auto snap = latest_snapshot_;
    ros2_roboclaw_driver::msg::RoboClawStatus msg;

    // Motor 1 data
    msg.m1_p = snap.m1_pid.p;
    msg.m1_i = snap.m1_pid.i;
    msg.m1_d = snap.m1_pid.d;
    msg.m1_qpps = snap.m1_pid.qpps;
    msg.m1_speed_command = roboclaw_dev_ ? roboclaw_dev_->getLastCommand1() : 0;
    msg.m1_speed_measured = snap.m1_enc.speed_qpps;
    msg.m1_current_instant = snap.currents.m1_inst;
    msg.m1_current_avg = current_ema_m1_; // Use the EMA value
    msg.m1_encoder_value = static_cast<uint32_t>(snap.m1_enc.value & 0xFFFFFFFF);
    msg.m1_encoder_status = snap.m1_enc.status;

    // Motor 2 data
    msg.m2_p = snap.m2_pid.p;
    msg.m2_i = snap.m2_pid.i;
    msg.m2_d = snap.m2_pid.d;
    msg.m2_qpps = snap.m2_pid.qpps;
    msg.m2_speed_command = roboclaw_dev_ ? roboclaw_dev_->getLastCommand2() : 0;
    msg.m2_speed_measured = snap.m2_enc.speed_qpps;
    msg.m2_current_instant = snap.currents.m2_inst;
    msg.m2_current_avg = current_ema_m2_; // Use the EMA value
    msg.m2_encoder_value = static_cast<uint32_t>(snap.m2_enc.value & 0xFFFFFFFF);
    msg.m2_encoder_status = snap.m2_enc.status;

    // Board electrical & thermal
    msg.main_battery_voltage = snap.volts.main;
    msg.logic_battery_voltage = snap.volts.logic;
    msg.temperature1 = snap.temps.t1;
    msg.temperature2 = snap.temps.t2;

    // Error reporting
    msg.error_bits = snap.status_bits;
    msg.error_json = ""; // TODO: Decode error bits to JSON if needed

    // Safety & estop (assuming these are managed elsewhere)
    msg.safety_enabled = get_parameter(params::kSafetyEnabled).as_bool();
    msg.safety_state = 0; // TODO: Get actual safety state
    msg.active_estop_sources = estop_mgr_.activeSources();
    msg.estop_reasons = {}; // TODO: Get estop reasons if available

    // Driver health & statistics
    msg.crc_error_count = 0; // TODO: Track CRC errors
    msg.io_error_count = 0; // TODO: Track I/O errors
    msg.retry_count = 0; // TODO: Track retry attempts
    msg.last_command_age = nowSec() - last_cmd_time_;
    msg.avg_loop_period = 0.0; // TODO: Calculate loop period if needed

    // Timestamp
    msg.stamp = now();

    status_pub_->publish(msg);
  }

  void DriverNode::odomTimer() {}

  void DriverNode::estopMsgCallback(const ros2_roboclaw_driver::msg::RoboClawEStop::SharedPtr msg) {
    if (msg->estop_active)
      estop_mgr_.set(msg->source, msg->reason);
    else
      estop_mgr_.clear(msg->source);
    ros2_roboclaw_driver::msg::RoboClawEStop state;
    state.estop_active = estop_mgr_.hasAny();
    state.all_active_sources = estop_mgr_.activeSources();
    estop_pub_->publish(state);
  }

  void DriverNode::onResetEncoders(
    const std::shared_ptr<ros2_roboclaw_driver::srv::ResetEncoders::Request> req,
    std::shared_ptr<ros2_roboclaw_driver::srv::ResetEncoders::Response> resp) {
    (void)req;
    std::string err; bool ok = false; if (roboclaw_dev_) ok = roboclaw_dev_->resetEncoders(err); resp->ok = ok;
  }
  void DriverNode::onClearAllEStops(
    const std::shared_ptr<ros2_roboclaw_driver::srv::ClearAllEStops::Request> req,
    std::shared_ptr<ros2_roboclaw_driver::srv::ClearAllEStops::Response> resp) {
    (void)req;
    estop_mgr_.clearAll();
    resp->success = true;
    resp->message = "cleared";
  }
  void DriverNode::onClearEStopSource(
    const std::shared_ptr<ros2_roboclaw_driver::srv::ClearEStopSource::Request> req,
    std::shared_ptr<ros2_roboclaw_driver::srv::ClearEStopSource::Response> resp) {
    bool existed = estop_mgr_.clear(req->source);
    resp->success = existed;
    resp->message = existed ? "cleared" : "not_found";
  }

  rcl_interfaces::msg::SetParametersResult DriverNode::onParamSet(
    const std::vector<rclcpp::Parameter>& params_vec) {
    rcl_interfaces::msg::SetParametersResult res;
    res.successful = true;
    res.reason = "ok";
    bool reconfig_odom = false;
    bool safety_reconfig = false;
    for (auto& p : params_vec) {
      const std::string& name = p.get_name();
      try {
        if (name == params::kAccelQpps) {
          double v = p.as_double();
          if (v <= 0) {
            res.successful = false;
            res.reason = "accel must be > 0";
            break;
          }
          // Acceleration parameter validated but no longer used for command shaping
          // (let nav2 handle acceleration, we use this for safety distance calculations)
        } else if (name == params::kWheelRadius) {
          double v = p.as_double();
          if (v <= 0 || v > 1.0) {
            res.successful = false;
            res.reason = "wheel_radius out of range";
            break;
          }
          wheel_radius_ = v;
          reconfig_odom = true;
        } else if (name == params::kWheelSeparation) {
          double v = p.as_double();
          if (v <= 0 || v > 2.0) {
            res.successful = false;
            res.reason = "wheel_separation out of range";
            break;
          }
          wheel_separation_ = v;
          reconfig_odom = true;
        } else if (name == params::kMaxLinearVel) {
          double v = p.as_double();
          if (v <= 0 || v > 5.0) {
            res.successful = false;
            res.reason = "max_linear_velocity out of range";
            break;
          }
        } else if (name == params::kMaxAngularVel) {
          double v = p.as_double();
          if (v <= 0 || v > 5.0) {
            res.successful = false;
            res.reason = "max_angular_velocity out of range";
            break;
          }
        } else if (name == params::kCmdTimeout) {
          double v = p.as_double();
          if (v <= 0 || v > 10.0) {
            res.successful = false;
            res.reason = "cmd_timeout out of range";
            break;
          }
        } else if (name == params::kQuadPulsesPerMeter) {
          int v = p.as_int();
          if (v <= 0 || v > 1000000) {
            res.successful = false;
            res.reason = "quad_pulses_per_meter out of range";
            break;
          }
          pulses_per_meter_ = v;
        } else if (name == params::kQuadPulsesPerRev) {
          double v = p.as_double();
          if (v <= 0 || v > 100000) {
            res.successful = false;
            res.reason = "quad_pulses_per_revolution out of range";
            break;
          }
          pulses_per_rev_ = v;
          reconfig_odom = true;
        } else if (name == params::kOdomLinearCov) {
          double v = p.as_double();
          if (v < 0) {
            res.successful = false;
            res.reason = "odom_linear_covariance must be >=0";
            break;
          }
        } else if (name == params::kOdomAngularCov) {
          double v = p.as_double();
          if (v < 0) {
            res.successful = false;
            res.reason = "odom_angular_covariance must be >=0";
            break;
          }
        } else if (name == params::kPublishTF) {
          bool v = p.as_bool();
          publish_tf_ = v;
          if (v && !tf_broadcaster_)
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
          if (!v) tf_broadcaster_.reset();
        } else if (name == params::kPublishOdom) {
          publish_odom_ = p.as_bool();
        } else if (name == params::kPublishJointStates) {
          publish_joint_ = p.as_bool();
        } else if (name == params::kSafetyEnabled || name == params::kOverCurrentLimitM1 ||
          name == params::kOverCurrentLimitM2 || name == params::kOverCurrentDetectTime ||
          name == params::kOverCurrentClearTime || name == params::kOverCurrentHysteresis ||
          name == params::kTemp1Limit || name == params::kTemp2Limit ||
          name == params::kTempClearDelta || name == params::kRunawaySpeedFactor ||
          name == params::kRunawayDetectTime || name == params::kStallSpeedRatio ||
          name == params::kStallMinCommand || name == params::kStallTimeout) {
          safety_reconfig = true;  // validate ranges
          // Basic validation examples
          if (name == params::kOverCurrentLimitM1 || name == params::kOverCurrentLimitM2) {
            double v = p.as_double();
            if (v <= 0 || v > 60.0) {
              res.successful = false;
              res.reason = "overcurrent_limit out of range";
              break;
            }
          } else if (name == params::kTemp1Limit || name == params::kTemp2Limit) {
            double v = p.as_double();
            if (v < 0 || v > 120.0) {
              res.successful = false;
              res.reason = "temp_limit out of range";
              break;
            }
          } else if (name == params::kRunawaySpeedFactor) {
            double v = p.as_double();
            if (v < 1.0 || v > 5.0) {
              res.successful = false;
              res.reason = "runaway_speed_factor out of range";
              break;
            }
          }
        }
      } catch (const std::exception& e) {
        res.successful = false;
        res.reason = std::string("exception validating param: ") + e.what();
        break;
      }
    }
    if (res.successful && reconfig_odom)
      odom_.configure(wheel_radius_, wheel_separation_, pulses_per_rev_ > 0 ? pulses_per_rev_ : 1.0);
    if (res.successful && safety_reconfig) {
      SafetyConfig cfg;
      cfg.enabled = get_parameter(params::kSafetyEnabled).as_bool();
      cfg.overcurrent_limit_m1 = get_parameter(params::kOverCurrentLimitM1).as_double();
      cfg.overcurrent_limit_m2 = get_parameter(params::kOverCurrentLimitM2).as_double();
      cfg.overcurrent_detect_time = get_parameter(params::kOverCurrentDetectTime).as_double();
      cfg.overcurrent_clear_time = get_parameter(params::kOverCurrentClearTime).as_double();
      cfg.overcurrent_hysteresis = get_parameter(params::kOverCurrentHysteresis).as_double();
      cfg.temp1_limit = get_parameter(params::kTemp1Limit).as_double();
      cfg.temp2_limit = get_parameter(params::kTemp2Limit).as_double();
      cfg.temp_clear_delta = get_parameter(params::kTempClearDelta).as_double();
      cfg.runaway_speed_factor = get_parameter(params::kRunawaySpeedFactor).as_double();
      cfg.runaway_detect_time = get_parameter(params::kRunawayDetectTime).as_double();
      cfg.stall_speed_ratio = get_parameter(params::kStallSpeedRatio).as_double();
      cfg.stall_min_command = get_parameter(params::kStallMinCommand).as_double();
      cfg.stall_timeout = get_parameter(params::kStallTimeout).as_double();
      safety_.configure(cfg);
    }
    return res;
  }

}  // namespace roboclaw_driver
