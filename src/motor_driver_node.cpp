// SPDX-License-Identifier: Apache-2.0
/****************************************************************************
 *  Copyright (c) 2025 Michael Wimble. All rights reserved.
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *      http://www.apache.org/licenses/LICENSE-2.0
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 ****************************************************************************/

#include <rclcpp/rclcpp.hpp>
#include <string>

#include "motor_driver.h"
#include "roboclaw.h"
#include "ros2_roboclaw_driver/msg/robo_claw_status.hpp"

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
            roboClawStatus.logic_battery_voltage =
                RoboClaw::singleton()->getLogicBatteryLevel();
            roboClawStatus.main_battery_voltage =
                RoboClaw::singleton()->getMainBatteryLevel();
            RoboClaw::TMotorCurrents motorCurrents =
                RoboClaw::singleton()->getMotorCurrents();
            roboClawStatus.m1_motor_current = motorCurrents.m1Current;
            roboClawStatus.m2_motor_current = motorCurrents.m2Current;

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

            roboClawStatus.temperature = RoboClaw::singleton()->getTemperature();

            roboClawStatus.m1_encoder_value = RoboClaw::singleton()->getM1Encoder();
            roboClawStatus.m1_encoder_status =
                RoboClaw::singleton()->getM1EncoderStatus();
            roboClawStatus.m2_encoder_value = RoboClaw::singleton()->getM2Encoder();
            roboClawStatus.m2_encoder_status =
                RoboClaw::singleton()->getM2EncoderStatus();

            roboClawStatus.m1_current_speed =
                RoboClaw::singleton()->getVelocity(RoboClaw::kM1);
            roboClawStatus.m2_current_speed =
                RoboClaw::singleton()->getVelocity(RoboClaw::kM2);

            roboClawStatus.error_string = RoboClaw::singleton()->getErrorString();

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