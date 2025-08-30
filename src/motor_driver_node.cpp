// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2025 Michael Wimble. https://github.com/wimblerobotics/ros2_roboclaw_driver
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "motor_driver.h"
#include "roboclaw.h"
#include "io_executor.h"
#include "ros2_roboclaw_driver/msg/robo_claw_status.hpp"

// Error bit definitions from RoboClaw manual
#define ERROR_ESTOP            0x00000001
#define ERROR_TEMP             0x00000002
#define ERROR_TEMP2            0x00000004
#define ERROR_LBATHIGH         0x00000010
#define ERROR_LBATLOW          0x00000020
#define ERROR_FAULTM1          0x00000040
#define ERROR_FAULTM2          0x00000080
#define ERROR_SPEED1           0x00000100
#define ERROR_SPEED2           0x00000200
#define ERROR_POS1             0x00000400
#define ERROR_POS2             0x00000800
#define ERROR_CURRENTM1        0x00001000
#define ERROR_CURRENTM2        0x00002000
#define WARN_OVERCURRENTM1     0x00010000
#define WARN_OVERCURRENTM2     0x00020000
#define WARN_MBATHIGH          0x00040000
#define WARN_MBATLOW           0x00080000
#define WARN_TEMP              0x00100000
#define WARN_TEMP2             0x00200000
#define WARN_S4                0x00400000
#define WARN_S5                0x00800000
#define WARN_CAN               0x10000000  // MCP Only
#define WARN_BOOT              0x20000000
#define WARN_OVERREGENM1       0x40000000
#define WARN_OVERREGENM2       0x80000000

std::string decodeErrorBits(uint32_t error_status) {
    if (error_status == 0) {
        return "No errors or warnings";
    }

    std::string result;

    // Check error bits
    if (error_status & ERROR_ESTOP) result += "ERROR_ESTOP ";
    if (error_status & ERROR_TEMP) result += "ERROR_TEMP ";
    if (error_status & ERROR_TEMP2) result += "ERROR_TEMP2 ";
    if (error_status & ERROR_LBATHIGH) result += "ERROR_LBATHIGH ";
    if (error_status & ERROR_LBATLOW) result += "ERROR_LBATLOW ";
    if (error_status & ERROR_FAULTM1) result += "ERROR_FAULTM1 ";
    if (error_status & ERROR_FAULTM2) result += "ERROR_FAULTM2 ";
    if (error_status & ERROR_SPEED1) result += "ERROR_SPEED1 ";
    if (error_status & ERROR_SPEED2) result += "ERROR_SPEED2 ";
    if (error_status & ERROR_POS1) result += "ERROR_POS1 ";
    if (error_status & ERROR_POS2) result += "ERROR_POS2 ";
    if (error_status & ERROR_CURRENTM1) result += "ERROR_CURRENTM1 ";
    if (error_status & ERROR_CURRENTM2) result += "ERROR_CURRENTM2 ";

    // Check warning bits
    if (error_status & WARN_OVERCURRENTM1) result += "WARN_OVERCURRENTM1 ";
    if (error_status & WARN_OVERCURRENTM2) result += "WARN_OVERCURRENTM2 ";
    if (error_status & WARN_MBATHIGH) result += "WARN_MBATHIGH ";
    if (error_status & WARN_MBATLOW) result += "WARN_MBATLOW ";
    if (error_status & WARN_TEMP) result += "WARN_TEMP ";
    if (error_status & WARN_TEMP2) result += "WARN_TEMP2 ";
    if (error_status & WARN_S4) result += "WARN_S4 ";
    if (error_status & WARN_S5) result += "WARN_S5 ";
    if (error_status & WARN_CAN) result += "WARN_CAN ";
    if (error_status & WARN_BOOT) result += "WARN_BOOT ";
    if (error_status & WARN_OVERREGENM1) result += "WARN_OVERREGENM1 ";
    if (error_status & WARN_OVERREGENM2) result += "WARN_OVERREGENM2 ";

    // Remove trailing space
    if (!result.empty() && result.back() == ' ') {
        result.pop_back();
    }

    return result;
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node =
        rclcpp::Node::make_shared("ros2_roboclaw_driver_node");
    MotorDriver& motorDriver = MotorDriver::singleton();
    motorDriver.onInit(node);

    auto qos = rclcpp::QoS(
        rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    qos.avoid_ros_namespace_conventions(false);

    std::string statusTopicName;
    node->declare_parameter<std::string>("roboclaw_status_topic",
        "roboclaw_status");
    node->get_parameter("roboclaw_status_topic", statusTopicName);
    RCUTILS_LOG_INFO("[motor_driver_node] roboclaw_status_topic: %s",
        statusTopicName.c_str());

    rclcpp::Publisher<ros2_roboclaw_driver::msg::RoboClawStatus>::SharedPtr
        statusPublisher =
        node->create_publisher<ros2_roboclaw_driver::msg::RoboClawStatus>(
            statusTopicName, qos);

    ros2_roboclaw_driver::msg::RoboClawStatus roboClawStatus;
    rclcpp::WallRate loop_rate(20);
    while (rclcpp::ok()) {
        try {
            // Use cached sensor data from IoExecutor instead of direct RoboClaw calls
            auto& cache = IoExecutor::instance().getDeviceCache();

            // Battery voltages from cache
            roboClawStatus.logic_battery_voltage = cache.getLogicBatteryVoltage();
            roboClawStatus.main_battery_voltage = cache.getMainBatteryVoltage();

            // Motor currents from cache
            auto currents = cache.getCurrents();
            roboClawStatus.m1_motor_current = currents.first;
            roboClawStatus.m2_motor_current = currents.second;

            // PID values (these don't change often, so direct access is OK)
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

            // Temperature from cache  
            auto temps = cache.getTemperatures();
            roboClawStatus.temperature = temps.first;

            // Encoder values from smart cache (uses cached values unless stale)
            auto encoders = motorDriver.getEncodersForStatus();
            roboClawStatus.m1_encoder_value = encoders.first;
            roboClawStatus.m2_encoder_value = encoders.second;

            // Encoder status (direct access - these are rarely read)
            roboClawStatus.m1_encoder_status = RoboClaw::singleton()->getM1EncoderStatus();
            roboClawStatus.m2_encoder_status = RoboClaw::singleton()->getM2EncoderStatus();

            // Current speeds from cache
            auto velocities = cache.getVelocities();
            roboClawStatus.m1_current_speed = velocities.first;
            roboClawStatus.m2_current_speed = velocities.second;

            // Error status from cache and decoded error string
            roboClawStatus.error_status = cache.getStatus();
            roboClawStatus.error_string = decodeErrorBits(roboClawStatus.error_status);

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