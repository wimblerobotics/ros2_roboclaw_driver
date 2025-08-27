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

namespace roboclaw_driver {

namespace {
inline double nowSec() {
  return rclcpp::Clock().now().seconds();
}
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
  std::string port = get_parameter(params::kPort).as_string();
  int baud = get_parameter(params::kBaudRate).as_int();
  int addr = get_parameter(params::kDeviceAddress).as_int();
  double accel = get_parameter(params::kAccelQpps).as_double();
  wheel_radius_ = get_parameter(params::kWheelRadius).as_double();
  wheel_separation_ = get_parameter(params::kWheelSeparation).as_double();
  publish_odom_ = get_parameter(params::kPublishOdom).as_bool();
  publish_joint_ = get_parameter(params::kPublishJointStates).as_bool();
  publish_tf_ = get_parameter(params::kPublishTF).as_bool();
  pulses_per_meter_ = get_parameter(params::kQuadPulsesPerMeter).as_int();
  pulses_per_rev_ = get_parameter(params::kQuadPulsesPerRev).as_double();
  command_shaper_.configure(accel);
  odom_.configure(wheel_radius_, wheel_separation_, pulses_per_rev_ > 0 ? pulses_per_rev_ : 1.0);
  // Configure safety supervisor with current params
  SafetyConfig scfg;
  scfg.enabled = get_parameter(params::kSafetyEnabled).as_bool();
  scfg.overcurrent_limit_m1 = get_parameter(params::kOverCurrentLimitM1).as_double();
  scfg.overcurrent_limit_m2 = get_parameter(params::kOverCurrentLimitM2).as_double();
  scfg.overcurrent_detect_time = get_parameter(params::kOverCurrentDetectTime).as_double();
  scfg.overcurrent_clear_time = get_parameter(params::kOverCurrentClearTime).as_double();
  scfg.overcurrent_hysteresis = get_parameter(params::kOverCurrentHysteresis).as_double();
  scfg.temp1_limit = get_parameter(params::kTemp1Limit).as_double();
  scfg.temp2_limit = get_parameter(params::kTemp2Limit).as_double();
  scfg.temp_clear_delta = get_parameter(params::kTempClearDelta).as_double();
  scfg.runaway_speed_factor = get_parameter(params::kRunawaySpeedFactor).as_double();
  scfg.runaway_detect_time = get_parameter(params::kRunawayDetectTime).as_double();
  scfg.stall_speed_ratio = get_parameter(params::kStallSpeedRatio).as_double();
  scfg.stall_min_command = get_parameter(params::kStallMinCommand).as_double();
  scfg.stall_timeout = get_parameter(params::kStallTimeout).as_double();
  scfg.estop_auto_clear = get_parameter(params::kEstopAutoClear).as_bool();
  safety_.configure(scfg);
  transport_ = std::make_unique<SerialTransport>();
  std::string err;
  if (!transport_->open(port, baud, err)) {
    RCLCPP_ERROR(get_logger(), "Failed to open %s: %s", port.c_str(), err.c_str());
  }
  protocol_ = std::make_unique<Protocol>(*transport_, static_cast<uint8_t>(addr));
  hw_ = std::make_unique<HardwareInterface>(*protocol_, status_decoder_);
  if (!hw_->initialize(err)) {
    RCLCPP_ERROR(get_logger(), "Hardware init failed: %s", err.c_str());
  }
  if (publish_odom_) odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  if (publish_joint_)
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
}

void DriverNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  last_cmd_time_ = nowSec();
  double v = msg->linear.x;
  double w = msg->angular.z;
  double v_left = v - w * wheel_separation_ / 2.0;
  double v_right = v + w * wheel_separation_ / 2.0;
  int32_t m1_qpps = 0;
  int32_t m2_qpps = 0;
  if (pulses_per_meter_ > 0) {
    m1_qpps = static_cast<int32_t>(v_left * pulses_per_meter_);
    m2_qpps = static_cast<int32_t>(v_right * pulses_per_meter_);
  }
  command_shaper_.update(m1_qpps, m2_qpps);
  hw_->setLastCommandSpeeds(command_shaper_.shapedM1(), command_shaper_.shapedM2());
  std::string err;
  if (!hw_->driveSpeeds(command_shaper_.shapedM1(), command_shaper_.shapedM2(), err)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "driveSpeeds failed: %s", err.c_str());
  }
}

void DriverNode::sensorTimer() {
  static double ema_m1 = 0.0, ema_m2 = 0.0;
  static bool ema_init = false;
  const double ema_alpha = 0.2;
  static double last_loop_time = nowSec();
  static double avg_loop = 0.0;  // expose via lambda
  double loop_now = nowSec();
  double loop_dt = loop_now - last_loop_time;
  last_loop_time = loop_now;
  if (avg_loop == 0.0)
    avg_loop = loop_dt;
  else
    avg_loop = 0.9 * avg_loop + 0.1 * loop_dt;

  double age = nowSec() - last_cmd_time_;
  double timeout = get_parameter(params::kCmdTimeout).as_double();
  if (age > timeout) {
    auto snap_latest = hw_->latest();
    if (snap_latest.m1_command_qpps != 0 || snap_latest.m2_command_qpps != 0) {
      command_shaper_.update(0, 0);
      hw_->setLastCommandSpeeds(0, 0);
      std::string err2;
      hw_->driveSpeeds(0, 0, err2);
    }
  }
  std::string err;
  auto snap = hw_->readSnapshot(err);
  if (!snap) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Sensor read failed: %s", err.c_str());
    return;
  }
  // Exponential moving average currents
  if (!ema_init) {
    ema_m1 = snap->currents.m1_inst;
    ema_m2 = snap->currents.m2_inst;
    ema_init = true;
  } else {
    ema_m1 = ema_alpha * snap->currents.m1_inst + (1 - ema_alpha) * ema_m1;
    ema_m2 = ema_alpha * snap->currents.m2_inst + (1 - ema_alpha) * ema_m2;
  }
  // Safety evaluation
  if (get_parameter(params::kSafetyEnabled).as_bool()) {
    SafetySample s;
    s.time_now = nowSec();
    s.m1_cmd_qpps = snap->m1_command_qpps;
    s.m2_cmd_qpps = snap->m2_command_qpps;
    s.m1_meas_qpps = snap->m1_enc.speed_qpps;
    s.m2_meas_qpps = snap->m2_enc.speed_qpps;
    s.m1_current_avg = ema_m1;
    s.m2_current_avg = ema_m2;
    s.temp1 = snap->temps.t1;
    s.temp2 = snap->temps.t2;
    auto result = safety_.evaluate(s);
    if (result.estop) {
      if (!estop_mgr_.hasAny()) {  // replace hasSource (not implemented) with simple any check
                                   // before logging
        estop_mgr_.set(result.source, result.reason);
        RCLCPP_ERROR(get_logger(), "Safety EStop: %s (%s)", result.source.c_str(),
                     result.reason.c_str());
      }
      std::string err_stop;
      hw_->driveSpeeds(0, 0, err_stop);
    }
  }
  // Store ema for statusTimer via static accessors
  current_ema_m1_ = ema_m1;
  current_ema_m2_ = ema_m2;
  avg_loop_period_sec_ = avg_loop;

  if (publish_odom_ && odom_pub_ && pulses_per_rev_ > 0) {
    static double last_time = nowSec();
    double now_s = nowSec();
    double dt = now_s - last_time;
    if (dt <= 0) dt = 1e-3;
    last_time = now_s;
    auto res = odom_.update(snap->m1_enc.value, snap->m2_enc.value, dt);
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = now();
    auto odom_frame = get_parameter(params::kFrameOdom).as_string();
    auto base_frame = get_parameter(params::kFrameBase).as_string();
    odom_msg.header.frame_id = odom_frame;
    odom_msg.child_frame_id = base_frame;
    odom_msg.pose.pose.position.x = res.x;
    odom_msg.pose.pose.position.y = res.y;
    odom_msg.pose.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, res.heading);
    odom_msg.pose.pose.orientation = tf2::toMsg(q);
    odom_msg.twist.twist.linear.x = res.linear;
    odom_msg.twist.twist.angular.z = res.angular;

    // Set simple covariance (diagonal entries)
    double lin_cov = get_parameter(params::kOdomLinearCov).as_double();
    double ang_cov = get_parameter(params::kOdomAngularCov).as_double();
    for (int i = 0; i < 36; ++i) odom_msg.pose.covariance[i] = 0.0;
    odom_msg.pose.covariance[0] = lin_cov;   // x
    odom_msg.pose.covariance[7] = lin_cov;   // y
    odom_msg.pose.covariance[35] = ang_cov;  // yaw
    for (int i = 0; i < 36; ++i) odom_msg.twist.covariance[i] = 0.0;
    odom_msg.twist.covariance[0] = lin_cov;
    odom_msg.twist.covariance[7] = lin_cov;
    odom_msg.twist.covariance[35] = ang_cov;
    odom_pub_->publish(odom_msg);
    if (publish_tf_ && tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header.stamp = odom_msg.header.stamp;
      tf_msg.header.frame_id = odom_frame;
      tf_msg.child_frame_id = base_frame;
      tf_msg.transform.translation.x = res.x;
      tf_msg.transform.translation.y = res.y;
      tf_msg.transform.translation.z = 0.0;
      tf_msg.transform.rotation = odom_msg.pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf_msg);
    }
  }
  if (publish_joint_ && joint_pub_) {
    sensor_msgs::msg::JointState js;
    js.header.stamp = now();
    auto left_name = get_parameter(params::kLeftJointName).as_string();
    auto right_name = get_parameter(params::kRightJointName).as_string();
    js.name = {left_name, right_name};
    if (pulses_per_rev_ > 0) {
      double rev_left = snap->m1_enc.value / pulses_per_rev_;
      double rev_right = snap->m2_enc.value / pulses_per_rev_;
      js.position = {rev_left * 2.0 * M_PI, rev_right * 2.0 * M_PI};
      js.velocity = {snap->m1_enc.speed_qpps / pulses_per_rev_ * 2.0 * M_PI,
                     snap->m2_enc.speed_qpps / pulses_per_rev_ * 2.0 * M_PI};
    } else {
      js.position = {0.0, 0.0};
      js.velocity = {0.0, 0.0};
    }
    joint_pub_->publish(js);
  }
  // cache avg loop period for status
  // (Could store in member if needed)
  // simple approach: store in command_shaper_ unused field? Instead add static local captured via
  // lambda in statusTimer. Use parameter interface to store? We'll keep a static global variable
  // accessible via function static. Provide accessor by lambda inside statusTimer (duplicate logic)
  // if needed.
  (void)avg_loop;  // silence unused for now until statusTimer reads updated static via ODR.
}

void DriverNode::statusTimer() {
  auto snap = hw_->latest();
  ros2_roboclaw_driver::msg::RoboClawStatus msg;
  msg.m1_p = snap.m1_pid.p;
  msg.m1_i = snap.m1_pid.i;
  msg.m1_d = snap.m1_pid.d;
  msg.m1_qpps = snap.m1_pid.qpps;
  msg.m2_p = snap.m2_pid.p;
  msg.m2_i = snap.m2_pid.i;
  msg.m2_d = snap.m2_pid.d;
  msg.m2_qpps = snap.m2_pid.qpps;
  msg.m1_speed_command = snap.m1_command_qpps;
  msg.m2_speed_command = snap.m2_command_qpps;
  msg.m1_speed_measured = snap.m1_enc.speed_qpps;
  msg.m2_speed_measured = snap.m2_enc.speed_qpps;
  msg.m1_current_instant = snap.currents.m1_inst;
  msg.m2_current_instant = snap.currents.m2_inst;
  msg.m1_current_avg = current_ema_m1_;
  msg.m2_current_avg = current_ema_m2_;
  msg.m1_encoder_value = static_cast<uint32_t>(snap.m1_enc.value & 0xFFFFFFFF);
  msg.m1_encoder_status = snap.m1_enc.status;
  msg.m2_encoder_value = static_cast<uint32_t>(snap.m2_enc.value & 0xFFFFFFFF);
  msg.m2_encoder_status = snap.m2_enc.status;
  msg.main_battery_voltage = snap.volts.main;
  msg.logic_battery_voltage = snap.volts.logic;
  msg.temperature1 = snap.temps.t1;
  msg.temperature2 = snap.temps.t2;
  msg.error_bits = snap.status_bits;
  msg.error_json = status_decoder_.toJson(snap.status_bits);
  msg.safety_enabled = get_parameter(params::kSafetyEnabled).as_bool();
  msg.safety_state = estop_mgr_.hasAny() ? 2 : 0;
  msg.active_estop_sources = estop_mgr_.activeSources();
  msg.estop_reasons = estop_mgr_.reasons();
  msg.last_command_age = nowSec() - last_cmd_time_;
  msg.crc_error_count = 0;
  msg.io_error_count = 0;
  msg.retry_count = 0;
  msg.avg_loop_period = static_cast<float>(avg_loop_period_sec_);
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
  std::string err;
  bool ok = hw_->resetEncoders(err);
  resp->ok = ok;
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
