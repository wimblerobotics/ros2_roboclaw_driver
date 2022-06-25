#include <rclcpp/rclcpp.hpp>
#include <string>

#include "motor_driver.h"
#include "ros2_roboclaw_driver/msg/robo_claw_status.hpp"
#include "roboclaw.h"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("ros2_roboclaw_driver_node");
  MotorDriver &motorDriver = MotorDriver::singleton();
  motorDriver.onInit(node);

  auto qos = rclcpp::QoS(
      rclcpp::QoSInitialization(
          RMW_QOS_POLICY_HISTORY_KEEP_LAST,
          10));
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
  qos.avoid_ros_namespace_conventions(false);

  std::string statusTopicName;
  node->declare_parameter<std::string>("roboclaw_status_topic", "roboclaw_status");
  node->get_parameter("roboclaw_status_topic", statusTopicName);
  RCUTILS_LOG_INFO("[motor_driver_node] roboclaw_status_topic: %s", statusTopicName.c_str());

  rclcpp::Publisher<ros2_roboclaw_driver::msg::RoboClawStatus>::SharedPtr statusPublisher = node->create_publisher<ros2_roboclaw_driver::msg::RoboClawStatus>(statusTopicName, qos);

  ros2_roboclaw_driver::msg::RoboClawStatus roboClawStatus;
  rclcpp::WallRate loop_rate(20);
  while (rclcpp::ok())
  {
    try
    {
      roboClawStatus.logic_battery_voltage = RoboClaw::singleton()->getLogicBatteryLevel();
      roboClawStatus.main_battery_voltage = RoboClaw::singleton()->getMainBatteryLevel();
      RoboClaw::TMotorCurrents motorCurrents = RoboClaw::singleton()->getMotorCurrents();
      roboClawStatus.m1_motor_current = motorCurrents.m1Current;
      roboClawStatus.m2_motor_current = motorCurrents.m2Current;

      RoboClaw::TPIDQ pidq = RoboClaw::singleton()->getPIDQ(RoboClaw::kGETM1PID);
      roboClawStatus.m1_p = pidq.p / 65536.0;
      roboClawStatus.m1_i = pidq.i / 65536.0;
      roboClawStatus.m1_d = pidq.d / 65536.0;
      roboClawStatus.m1_qpps = pidq.qpps;

      pidq = RoboClaw::singleton()->getPIDQ(RoboClaw::kGETM2PID);
      roboClawStatus.m2_p = pidq.p / 65536.0;
      roboClawStatus.m2_i = pidq.i / 65536.0;
      roboClawStatus.m2_d = pidq.d / 65536.0;
      roboClawStatus.m2_qpps = pidq.qpps;

      roboClawStatus.temperature = RoboClaw::singleton()->getTemperature();

      {
        RoboClaw::EncodeResult encoder = RoboClaw::singleton()->getEncoderCommandResult(RoboClaw::kGETM1ENC);
        roboClawStatus.m1_encoder_value = encoder.value;
        roboClawStatus.m1_encoder_status = encoder.status;
      }

      {
        RoboClaw::EncodeResult encoder = RoboClaw::singleton()->getEncoderCommandResult(RoboClaw::kGETM2ENC);
        roboClawStatus.m2_encoder_value = encoder.value;
        roboClawStatus.m2_encoder_status = encoder.status;
      }

      roboClawStatus.m1_current_speed = RoboClaw::singleton()->getVelocity(RoboClaw::kGETM1SPEED);
      roboClawStatus.m2_current_speed = RoboClaw::singleton()->getVelocity(RoboClaw::kGETM2SPEED);

      roboClawStatus.error_string = RoboClaw::singleton()->getErrorString();

      statusPublisher->publish(roboClawStatus);
    }
    catch (RoboClaw::TRoboClawException *e)
    {
      RCUTILS_LOG_ERROR("[motor_driver_node] Exception: %s", e->what());
    }
    catch (...)
    {
      RCUTILS_LOG_ERROR("[motor_driver_node] Uncaught exception !!!");
    }

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}