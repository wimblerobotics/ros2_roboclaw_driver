// MIT License
// Author: Michael Wimble <mike@wimblerobotics.com>
#pragma once
#include <string>
namespace roboclaw_driver {
namespace params {
// Parameter name constants
static constexpr const char* kPort = "device_name";           // serial or usb path
static constexpr const char* kBaudRate = "baud_rate";         // ignored for USB virtual ACM
static constexpr const char* kDeviceAddress = "device_port";  // RoboClaw address
static constexpr const char* kAccelQpps = "accel_quad_pulses_per_second";
static constexpr const char* kM1P = "m1_p";
static constexpr const char* kM1I = "m1_i";
static constexpr const char* kM1D = "m1_d";
static constexpr const char* kM1QPPS = "m1_qpps";
static constexpr const char* kM2P = "m2_p";
static constexpr const char* kM2I = "m2_i";
static constexpr const char* kM2D = "m2_d";
static constexpr const char* kM2QPPS = "m2_qpps";
static constexpr const char* kM1MaxCurrent = "m1_max_current";
static constexpr const char* kM2MaxCurrent = "m2_max_current";
static constexpr const char* kMaxAngularVel = "max_angular_velocity";
static constexpr const char* kMaxLinearVel = "max_linear_velocity";
static constexpr const char* kCmdTimeout = "max_seconds_uncommanded_travel";
static constexpr const char* kPublishJointStates = "publish_joint_states";
static constexpr const char* kPublishOdom = "publish_odom";
static constexpr const char* kQuadPulsesPerMeter = "quad_pulses_per_meter";
static constexpr const char* kQuadPulsesPerRev = "quad_pulses_per_revolution";
static constexpr const char* kWheelRadius = "wheel_radius";
static constexpr const char* kWheelSeparation = "wheel_separation";
static constexpr const char* kSensorRate = "sensor_rate_hz";           // legacy name
static constexpr const char* kSensorPollRate = "sensor_poll_rate_hz";  // new
static constexpr const char* kStatusRate = "status_rate_hz";
static constexpr const char* kOdomRate = "odom_rate_hz";
static constexpr const char* kPublishTF = "publish_tf";
static constexpr const char* kLeftJointName = "left_wheel_joint_name";
static constexpr const char* kRightJointName = "right_wheel_joint_name";
static constexpr const char* kSafetyEnabled = "safety_enabled";
static constexpr const char* kOverCurrentLimitM1 = "overcurrent_limit_m1";
static constexpr const char* kOverCurrentLimitM2 = "overcurrent_limit_m2";
static constexpr const char* kOverCurrentDetectTime = "overcurrent_detect_time";
static constexpr const char* kOverCurrentClearTime = "overcurrent_clear_time";
static constexpr const char* kOverCurrentHysteresis = "overcurrent_hysteresis";
static constexpr const char* kTemp1Limit = "temp1_limit";
static constexpr const char* kTemp2Limit = "temp2_limit";
static constexpr const char* kTempClearDelta = "temp_clear_delta";
static constexpr const char* kRunawaySpeedFactor = "runaway_speed_factor";
static constexpr const char* kRunawayDetectTime = "runaway_detect_time";
static constexpr const char* kStallSpeedRatio = "stall_speed_ratio";
static constexpr const char* kStallMinCommand = "stall_min_command";
static constexpr const char* kStallTimeout = "stall_timeout";
static constexpr const char* kEstopAutoClear = "estop_auto_clear_enabled";
static constexpr const char* kEstopRepeatWindow = "estop_repeat_window";
static constexpr const char* kEstopRepeatLimit = "estop_repeat_limit";
static constexpr const char* kLogLevel = "log_level";
static constexpr const char* kDebug = "do_debug";
static constexpr const char* kDebugLowLevel = "do_low_level_debug";
static constexpr const char* kFrameOdom = "odom_frame";
static constexpr const char* kFrameBase = "base_frame";
static constexpr const char* kFrameIMU = "imu_frame";
static constexpr const char* kOdomLinearCov = "odom_linear_covariance";
static constexpr const char* kOdomAngularCov = "odom_angular_covariance";
// Defaults
struct Defaults {
  int baud_rate = 38400;
  int device_address = 128;
  double accel_qpps = 1000.0;
  double max_linear_vel = 0.3;
  double max_angular_vel = 0.07;
  double cmd_timeout = 0.25;
  double wheel_radius = 0.1;
  double wheel_separation = 0.345;
  double sensor_poll_rate = 20.0;
  double status_rate = 1.0;
  double odom_rate = 40.0;
  bool publish_tf = true;
  bool publish_odom = true;
  bool publish_joint_states = false;
  const char* left_joint = "left_wheel_joint";
  const char* right_joint = "right_wheel_joint";
  bool safety_enabled = true;
  double overcurrent_limit_m1 = 6.0;
  double overcurrent_limit_m2 = 6.0;
  double overcurrent_detect_time = 0.2;
  double overcurrent_clear_time = 1.0;
  double overcurrent_hysteresis = 0.5;
  double temp1_limit = 80.0;      // C
  double temp2_limit = 80.0;      // C
  double temp_clear_delta = 5.0;  // C below limit to clear
  double runaway_speed_factor = 1.5;
  double runaway_detect_time = 0.3;
  double stall_speed_ratio = 0.1;
  double stall_min_command = 100.0;  // qpps
  double stall_timeout = 1.0;
  bool estop_auto_clear = true;
  double estop_repeat_window = 30.0;
  int estop_repeat_limit = 5;
  double odom_linear_cov = 0.0025;  // (0.05 m)^2
  double odom_angular_cov = 0.01;   // (0.1 rad)^2
};
}  // namespace params
}  // namespace roboclaw_driver
