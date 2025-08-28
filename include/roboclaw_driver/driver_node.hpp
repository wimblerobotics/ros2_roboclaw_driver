// MIT License
// Author: Michael Wimble <mike@wimblerobotics.com>
#pragma once
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "roboclaw_driver/estop_manager.hpp"
#include "roboclaw_driver/safety_supervisor.hpp"
#include "roboclaw_driver/status_decoder.hpp"
#include "roboclaw_driver/roboclaw_device.hpp"
#include "ros2_roboclaw_driver/msg/robo_claw_e_stop.hpp"
#include "ros2_roboclaw_driver/msg/robo_claw_status.hpp"
#include "ros2_roboclaw_driver/srv/clear_all_e_stops.hpp"
#include "ros2_roboclaw_driver/srv/clear_e_stop_source.hpp"
#include "ros2_roboclaw_driver/srv/reset_encoders.hpp"

namespace roboclaw_driver {
    class DriverNode : public rclcpp::Node {
    public:
        explicit DriverNode(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions());

    private:
        void declareParameters();
        void initObjects();
        void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
        void sensorTimer();
        void statusTimer();
        void odomTimer();
        void estopMsgCallback(const ros2_roboclaw_driver::msg::RoboClawEStop::SharedPtr msg);
        rcl_interfaces::msg::SetParametersResult onParamSet(const std::vector<rclcpp::Parameter>& params);
        void onResetEncoders(const std::shared_ptr<ros2_roboclaw_driver::srv::ResetEncoders::Request> req,
            std::shared_ptr<ros2_roboclaw_driver::srv::ResetEncoders::Response> resp);
        void onClearAllEStops(
            const std::shared_ptr<ros2_roboclaw_driver::srv::ClearAllEStops::Request> req,
            std::shared_ptr<ros2_roboclaw_driver::srv::ClearAllEStops::Response> resp);
        void onClearEStopSource(
            const std::shared_ptr<ros2_roboclaw_driver::srv::ClearEStopSource::Request> req,
            std::shared_ptr<ros2_roboclaw_driver::srv::ClearEStopSource::Response> resp);
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
        rclcpp::Publisher<ros2_roboclaw_driver::msg::RoboClawStatus>::SharedPtr status_pub_;
        rclcpp::Publisher<ros2_roboclaw_driver::msg::RoboClawEStop>::SharedPtr estop_pub_;
        rclcpp::Subscription<ros2_roboclaw_driver::msg::RoboClawEStop>::SharedPtr estop_cmd_sub_;
        rclcpp::TimerBase::SharedPtr sensor_timer_, status_timer_, odom_timer_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
        double last_cmd_time_{ 0 };
        std::unique_ptr<RoboClawDevice> roboclaw_dev_;
        StatusDecoder status_decoder_;
        EStopManager estop_mgr_;
        SafetySupervisor safety_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
        int pulses_per_meter_{ 0 };
        double pulses_per_rev_{ 0 };
        double wheel_radius_{ 0 };
        double wheel_separation_{ 0 };
        bool publish_odom_{ true };
        bool publish_joint_{ false };
        bool publish_tf_{ true };
        int64_t last_enc_left_{ 0 };
        int64_t last_enc_right_{ 0 };
        bool have_last_enc_{ false };
        double current_ema_m1_{ 0.0 };
        double current_ema_m2_{ 0.0 };
        double avg_loop_period_sec_{ 0.0 };
        Snapshot latest_snapshot_{};
        // Odometry state
        double x_{ 0.0 };
        double y_{ 0.0 };
        double yaw_{ 0.0 };
        double last_odom_time_{ 0.0 };
        std::string odom_frame_{ "odom" };
        std::string base_frame_{ "base_link" };
        rclcpp::Service<ros2_roboclaw_driver::srv::ResetEncoders>::SharedPtr reset_enc_srv_;
        rclcpp::Service<ros2_roboclaw_driver::srv::ClearAllEStops>::SharedPtr clear_all_estops_srv_;
        rclcpp::Service<ros2_roboclaw_driver::srv::ClearEStopSource>::SharedPtr clear_estop_source_srv_;
    };
}  // namespace roboclaw_driver
