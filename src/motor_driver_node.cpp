#include <rclcpp/rclcpp.hpp>
#include <string>

#include "motor_driver.h"
#include "roboclaw.h"
#include "ros2_roboclaw_driver/msg/robo_claw_status.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("ros2_roboclaw_driver_node");
  MotorDriver& motorDriver = MotorDriver::singleton();
  motorDriver.onInit(node);

  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10));
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
  qos.avoid_ros_namespace_conventions(false);

  std::string statusTopicName;
  node->declare_parameter<std::string>("roboclaw_status_topic", "roboclaw_status");
  node->get_parameter("roboclaw_status_topic", statusTopicName);
  RCUTILS_LOG_INFO("[motor_driver_node] roboclaw_status_topic: %s", statusTopicName.c_str());

  auto statusPublisher =
      node->create_publisher<ros2_roboclaw_driver::msg::RoboClawStatus>(statusTopicName, qos);

  ros2_roboclaw_driver::msg::RoboClawStatus roboClawStatus;
  rclcpp::WallRate loop_rate(20);
  while (rclcpp::ok()) {
    try {
      roboClawStatus.logic_battery_voltage = RoboClaw::singleton()->getLogicBatteryLevel();
      roboClawStatus.main_battery_voltage = RoboClaw::singleton()->getMainBatteryLevel();
      RoboClaw::TMotorCurrents motorCurrents = RoboClaw::singleton()->getMotorCurrents();
      // legacy fields removed in new message; map to instant currents
      roboClawStatus.m1_current_instant = motorCurrents.m1Current;
      roboClawStatus.m2_current_instant = motorCurrents.m2Current;
      roboClawStatus.m1_current_avg = motorCurrents.m1Current;
      roboClawStatus.m2_current_avg = motorCurrents.m2Current;

      RoboClaw::TPIDQ pidq = RoboClaw::singleton()->getPIDQM1();
      roboClawStatus.m1_p = pidq.p;
      roboClawStatus.m1_i = pidq.i;
      roboClawStatus.m1_d = pidq.d;
      roboClawStatus.m1_qpps = pidq.qpps;

      pidq = RoboClaw::singleton()->getPIDQM2();
      roboClawStatus.m2_p = pidq.p;
      roboClawStatus.m2_i = pidq.i;
      roboClawStatus.m2_d = pidq.d;
      roboClawStatus.m2_qpps = pidq.qpps;

      // New message has temperature1/temperature2; legacy single temperature -> temperature1
      roboClawStatus.temperature1 = RoboClaw::singleton()->getTemperature();
      roboClawStatus.temperature2 = roboClawStatus.temperature1;

      roboClawStatus.m1_encoder_value = RoboClaw::singleton()->getM1Encoder();
      roboClawStatus.m1_encoder_status = RoboClaw::singleton()->getM1EncoderStatus();
      roboClawStatus.m2_encoder_value = RoboClaw::singleton()->getM2Encoder();
      roboClawStatus.m2_encoder_status = RoboClaw::singleton()->getM2EncoderStatus();

      roboClawStatus.m1_speed_measured = RoboClaw::singleton()->getVelocity(RoboClaw::kM1);
      roboClawStatus.m2_speed_measured = RoboClaw::singleton()->getVelocity(RoboClaw::kM2);

      // Populate error bits/json if available via legacy API (leave zero otherwise)
      roboClawStatus.error_bits = RoboClaw::singleton()->getErrorStatus();
      roboClawStatus.error_json = RoboClaw::singleton()->getErrorString();

      auto now_time = node->now();
      roboClawStatus.stamp = now_time;

      statusPublisher->publish(roboClawStatus);
    } catch (RoboClaw::TRoboClawException* e) {
      RCUTILS_LOG_ERROR("[motor_driver_node] Exception: %s", e->what());
    } catch (...) {
      RCUTILS_LOG_ERROR("[motor_driver_node] Uncaught exception !!!");
    }

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}