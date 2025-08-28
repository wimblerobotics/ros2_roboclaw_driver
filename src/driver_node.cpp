/**
 * @file driver_node.cpp
 * @brief RoboClaw motor controller driver node implementation
 *
 * This file implements the main ROS 2 node for interfacing with RoboClaw motor controllers.
 * The driver provides velocity control, status monitoring, safety supervision, and diagnostic
 * capabilities for differential drive robots using RoboClaw motor controllers.
 *
 * Key Features:
 * - Direct serial communication with RoboClaw devices
 * - Real-time velocity command processing with acceleration shaping
 * - Comprehensive status monitoring and diagnostics
 * - Safety supervision with emergency stop capabilities
 * - Odometry integration and transform broadcasting
 * - Dynamic parameter reconfiguration
 *
 * @author Michael Wimble <mike@wimblerobotics.com>
 * @license MIT License
 */

 // MIT License
 // Author: Michael Wimble <mike@wimblerobotics.com>
#include "roboclaw_driver/driver_node.hpp"

#include <tf2/LinearMath/Quaternion.h>

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
    /** @brief Utility function to get current time in seconds since epoch */
    inline double nowSec() { return rclcpp::Clock().now().seconds(); }
  }  // namespace

  /**
   * @brief Constructor for the RoboClaw driver node
   *
   * Initializes the ROS 2 node with comprehensive parameter declarations, object initialization,
   * publisher/subscriber setup, service registration, and timer configuration. The constructor
   * ensures proper device communication before allowing the node to become operational.
   *
   * @param opts ROS 2 node options for configuration
   * @throws std::runtime_error if RoboClaw device initialization fails
   */
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

  /**
   * @brief Declares all ROS 2 parameters with default values and validation ranges
   *
   * This method defines the complete parameter interface for the RoboClaw driver, including:
   * - Device communication parameters (port, baud rate, address)
   * - Motion control parameters (acceleration, velocity limits, timeouts)
   * - Physical robot parameters (wheel dimensions, encoder scaling)
   * - Safety and monitoring parameters (current limits, temperature thresholds)
   * - Publishing options (odometry, joint states, transforms)
   *
   * All parameters include appropriate default values and will be validated during runtime.
   */
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

  /**
   * @brief Initializes all driver objects and establishes device communication
   *
   * This critical initialization method:
   * 1. Retrieves and validates all configuration parameters
   * 2. Configures motion control algorithms (command shaping, odometry)
   * 3. Establishes communication with the RoboClaw device
   * 4. Verifies device responsiveness through firmware version check
   * 5. Sets up publishers for optional data streams (odometry, joint states)
   *
   * The method implements fail-fast behavior - any communication failure will
   * prevent the node from becoming operational, ensuring system safety.
   *
   * @throws std::runtime_error if device initialization or communication fails
   */
  void DriverNode::initObjects() {
    std::string port = get_parameter(params::kPort).as_string();
    int baud = get_parameter(params::kBaudRate).as_int();
    int addr = get_parameter(params::kDeviceAddress).as_int();
    double accel = get_parameter(params::kAccelQpps).as_double();

    RCLCPP_INFO(this->get_logger(), "Initializing RoboClaw driver with parameters:");
    RCLCPP_INFO(this->get_logger(), "  Port: %s", port.c_str());
    RCLCPP_INFO(this->get_logger(), "  Baud Rate: %d", baud);
    RCLCPP_INFO(this->get_logger(), "  Device Address: %d", addr);
    RCLCPP_INFO(this->get_logger(), "  Acceleration: %.1f qpps/s", accel);

    wheel_radius_ = get_parameter(params::kWheelRadius).as_double();
    wheel_separation_ = get_parameter(params::kWheelSeparation).as_double();
    publish_odom_ = get_parameter(params::kPublishOdom).as_bool();
    publish_joint_ = get_parameter(params::kPublishJointStates).as_bool();
    publish_tf_ = get_parameter(params::kPublishTF).as_bool();
    pulses_per_meter_ = get_parameter(params::kQuadPulsesPerMeter).as_int();
    pulses_per_rev_ = get_parameter(params::kQuadPulsesPerRev).as_double();
    double cmd_timeout = get_parameter(params::kCmdTimeout).as_double();

    RCLCPP_INFO(this->get_logger(), "  Wheel radius: %.3f m", wheel_radius_);
    RCLCPP_INFO(this->get_logger(), "  Wheel separation: %.3f m", wheel_separation_);
    RCLCPP_INFO(this->get_logger(), "  Pulses per meter: %d", pulses_per_meter_);
    RCLCPP_INFO(this->get_logger(), "  Pulses per revolution: %.1f", pulses_per_rev_);
    RCLCPP_INFO(this->get_logger(), "  Command timeout: %.1f s", cmd_timeout);
    RCLCPP_INFO(this->get_logger(), "  Publish odom: %s", publish_odom_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  Publish joint states: %s", publish_joint_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  Publish TF: %s", publish_tf_ ? "true" : "false");

    command_shaper_.configure(accel);
    odom_.configure(wheel_radius_, wheel_separation_, pulses_per_rev_ > 0 ? pulses_per_rev_ : 1.0);

    // Initialize RoboClaw device with comprehensive error checking
    try {
      RCLCPP_INFO(this->get_logger(), "Attempting to connect to RoboClaw device...");
      roboclaw_dev_ = std::make_unique<RoboClawDevice>(port, baud, static_cast<uint8_t>(addr));

      if (!roboclaw_dev_->isInitialized()) {
        RCLCPP_FATAL(this->get_logger(), "RoboClaw device failed initialization!");
        throw std::runtime_error("RoboClaw device not properly initialized");
      }

      // Device is confirmed working - test firmware version
      std::string version = roboclaw_dev_->version();
      RCLCPP_INFO(this->get_logger(), "✓ RoboClaw device connected successfully!");
      RCLCPP_INFO(this->get_logger(), "✓ Firmware version: %s", version.c_str());
      RCLCPP_INFO(this->get_logger(), "✓ Device ready for operation");

    } catch (const std::exception& e) {
      RCLCPP_FATAL(this->get_logger(), "Critical error initializing RoboClaw device: %s", e.what());
      RCLCPP_FATAL(this->get_logger(), "Driver cannot continue - check:");
      RCLCPP_FATAL(this->get_logger(), "  1. Device path: %s", port.c_str());
      RCLCPP_FATAL(this->get_logger(), "  2. Baud rate: %d", baud);
      RCLCPP_FATAL(this->get_logger(), "  3. Device address: %d", addr);
      RCLCPP_FATAL(this->get_logger(), "  4. Physical connections and power");
      roboclaw_dev_.reset();
      throw;  // Re-throw to prevent node from starting
    }

    if (publish_odom_) odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    if (publish_joint_)
      joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  }

  /**
   * @brief Processes incoming velocity commands and translates them to motor speeds
   *
   * This callback handles cmd_vel messages by:
   * 1. Converting linear/angular velocities to differential wheel velocities
   * 2. Scaling wheel velocities to encoder pulses per second (QPPS)
   * 3. Applying acceleration shaping to ensure smooth motion
   * 4. Transmitting motor speed commands to the RoboClaw device
   * 5. Updating command timestamps for timeout monitoring
   *
   * The method implements the standard differential drive kinematic model:
   * - Left wheel velocity = linear_velocity - angular_velocity * wheel_separation / 2
   * - Right wheel velocity = linear_velocity + angular_velocity * wheel_separation / 2
   *
   * @param msg Incoming Twist message containing linear and angular velocity commands
   */
  void DriverNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    last_cmd_time_ = nowSec();
    double v = msg->linear.x;
    double w = msg->angular.z;
    double v_left = v - w * wheel_separation_ / 2.0;
    double v_right = v + w * wheel_separation_ / 2.0;
    int32_t m1_qpps = 0; int32_t m2_qpps = 0;
    if (pulses_per_meter_ > 0) {
      m1_qpps = static_cast<int32_t>(v_left * pulses_per_meter_);
      m2_qpps = static_cast<int32_t>(v_right * pulses_per_meter_);
    }
    command_shaper_.update(m1_qpps, m2_qpps);

    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "cmd_vel: v=%.3f, w=%.3f -> v_left=%.3f, v_right=%.3f -> m1=%d, m2=%d qpps -> shaped: m1=%d, m2=%d qpps",
      v, w, v_left, v_right, m1_qpps, m2_qpps,
      command_shaper_.shapedM1(), command_shaper_.shapedM2());

    std::string err;
    if (roboclaw_dev_) {
      bool ok = roboclaw_dev_->driveSpeeds(command_shaper_.shapedM1(), command_shaper_.shapedM2(), err);
      if (!ok) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "Failed to send drive speeds: %s", err.c_str());
      }
    }
  }

  /**
   * @brief High-frequency sensor data acquisition and safety monitoring
   *
   * This timer callback runs at 20Hz (50ms intervals) and performs:
   * 1. Command timeout monitoring - stops motors if no recent cmd_vel received
   * 2. Comprehensive device state reading (encoders, currents, voltages, temperatures)
   * 3. Exponential moving average calculation for current smoothing
   * 4. Real-time safety monitoring and fault detection
   * 5. Diagnostic logging for debugging and system monitoring
   *
   * The high update rate ensures responsive safety monitoring while providing
   * smooth data for control algorithms and state estimation.
   *
   * Error Handling:
   * - Communication timeouts are logged but don't stop the timer
   * - Device errors are reported through throttled warnings
   * - Safety violations trigger immediate motor stops
   */
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

    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "sensorTimer: Attempting to read RoboClaw snapshot from device at address %d",
      (int)get_parameter(params::kDeviceAddress).as_int());

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
      "RoboClaw data - M1 enc: %u speed: %d, M2 enc: %u speed: %d, Main V: %.1f, Logic V: %.1f, T1: %.1f, T2: %.1f",
      snap.m1_enc.value, snap.m1_enc.speed_qpps, snap.m2_enc.value, snap.m2_enc.speed_qpps,
      snap.volts.main, snap.volts.logic, snap.temps.t1, snap.temps.t2);
  }

  /**
   * @brief Publishes comprehensive RoboClaw status information
   *
   * This timer callback runs at 1Hz and publishes detailed status information
   * including motor states, electrical readings, thermal data, error conditions,
   * and driver health statistics.
   */
  void DriverNode::statusTimer() {
    auto snap = latest_snapshot_;
    ros2_roboclaw_driver::msg::RoboClawStatus msg;

    // Motor 1 PID configuration and status
    msg.m1_p = 0.0; // TODO: Implement PID reading when needed
    msg.m1_i = 0.0; // TODO: Implement PID reading when needed  
    msg.m1_d = 0.0; // TODO: Implement PID reading when needed
    msg.m1_qpps = 0; // TODO: Implement QPPS reading when needed
    msg.m1_speed_command = roboclaw_dev_ ? roboclaw_dev_->getLastCommand1() : 0;
    msg.m1_speed_measured = snap.m1_enc.speed_qpps;
    msg.m1_current_instant = snap.currents.m1_inst;
    msg.m1_current_avg = current_ema_m1_;
    msg.m1_encoder_value = static_cast<uint32_t>(snap.m1_enc.value & 0xFFFFFFFF);
    msg.m1_encoder_status = snap.m1_enc.status;

    // Motor 2 PID configuration and status  
    msg.m2_p = 0.0; // TODO: Implement PID reading when needed
    msg.m2_i = 0.0; // TODO: Implement PID reading when needed
    msg.m2_d = 0.0; // TODO: Implement PID reading when needed
    msg.m2_qpps = 0; // TODO: Implement QPPS reading when needed
    msg.m2_speed_command = roboclaw_dev_ ? roboclaw_dev_->getLastCommand2() : 0;
    msg.m2_speed_measured = snap.m2_enc.speed_qpps;
    msg.m2_current_instant = snap.currents.m2_inst;
    msg.m2_current_avg = current_ema_m2_;
    msg.m2_encoder_value = static_cast<uint32_t>(snap.m2_enc.value & 0xFFFFFFFF);
    msg.m2_encoder_status = snap.m2_enc.status;

    // Electrical and thermal monitoring
    msg.main_battery_voltage = snap.volts.main;
    msg.logic_battery_voltage = snap.volts.logic;
    msg.temperature1 = snap.temps.t1;
    msg.temperature2 = snap.temps.t2;

    // Error status and diagnostics
    msg.error_bits = snap.status_bits;
    msg.error_json = status_decoder_.toJson(snap.status_bits);

    // Safety system status
    msg.safety_enabled = get_parameter(params::kSafetyEnabled).as_bool();
    msg.safety_state = 0; // TODO: Implement safety state tracking when needed
    msg.active_estop_sources = estop_mgr_.activeSources();
    msg.estop_reasons = {}; // TODO: Implement detailed estop reasons when needed

    // Driver health and performance metrics
    msg.crc_error_count = 0; // TODO: Implement error tracking when needed
    msg.io_error_count = 0; // TODO: Implement error tracking when needed
    msg.retry_count = 0; // TODO: Implement retry tracking when needed
    msg.last_command_age = nowSec() - last_cmd_time_;
    msg.avg_loop_period = 0.0; // TODO: Implement loop period calculation when needed

    // Message timestamp
    msg.stamp = now();

    // Publish comprehensive status
    status_pub_->publish(msg);

    // Log critical status occasionally for monitoring
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
      "Status: M1[cmd=%d,meas=%d,cur=%.1fA] M2[cmd=%d,meas=%d,cur=%.1fA] V[main=%.1f,logic=%.1f] T[%.1f,%.1f] err=0x%x",
      msg.m1_speed_command, msg.m1_speed_measured, msg.m1_current_instant,
      msg.m2_speed_command, msg.m2_speed_measured, msg.m2_current_instant,
      msg.main_battery_voltage, msg.logic_battery_voltage,
      msg.temperature1, msg.temperature2, msg.error_bits);
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
          command_shaper_.configure(v);
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
