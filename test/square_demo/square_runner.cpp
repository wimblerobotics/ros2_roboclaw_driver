// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Wimblerobotics
// https://github.com/wimblerobotics/ros2_roboclaw_driver

#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <cmath>
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

/**
 * @brief Test program to drive robot in a square using odometry feedback
 *
 * This program drives the robot in a square pattern by:
 * 1. Moving forward for the specified distance
 * 2. Turning left 90 degrees
 * 3. Repeating for 4 sides to complete the square
 *
 * Uses odometry feedback to determine when to stop moving/turning.
 */
class SquareRunner : public rclcpp::Node {
 public:
  /**
   * @brief Constructor
   * @param square_size Side length of the square in meters (default: 0.5m)
   */
  explicit SquareRunner(double square_size = 0.5)
      : Node("square_runner"),
        square_size_(square_size),
        linear_speed_(0.2),         // m/s
        angular_speed_(0.5),        // rad/s
        position_tolerance_(0.05),  // 5cm tolerance
        angle_tolerance_(0.087),    // ~5 degree tolerance in radians
        current_state_(State::MOVING_FORWARD),
        side_count_(0)
  {
    // Create publishers and subscribers
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&SquareRunner::odom_callback, this, std::placeholders::_1));

    // Create timer for control loop
    timer_ = this->create_wall_timer(std::chrono::milliseconds(50),  // 20Hz control loop
                                     std::bind(&SquareRunner::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "Square Runner initialized with square size: %.2f m", square_size_);
    RCLCPP_INFO(this->get_logger(), "Waiting for initial odometry...");
  }

 private:
  enum class State { WAITING_FOR_ODOM, MOVING_FORWARD, TURNING_LEFT, COMPLETED };

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_odom_ = *msg;
    odom_received_ = true;

    if (current_state_ == State::WAITING_FOR_ODOM) {
      // Initialize start position and orientation
      start_position_ = current_odom_;
      target_position_ = current_odom_;
      current_state_ = State::MOVING_FORWARD;
      RCLCPP_INFO(this->get_logger(), "Starting square pattern - Side %d/4", side_count_ + 1);
    }
  }

  void control_loop() {
    if (!odom_received_) {
      current_state_ = State::WAITING_FOR_ODOM;
      return;
    }

    geometry_msgs::msg::Twist cmd_vel;

    switch (current_state_) {
      case State::WAITING_FOR_ODOM:
        // Do nothing, wait for odometry
        break;

      case State::MOVING_FORWARD:
        if (move_forward(cmd_vel)) {
          current_state_ = State::TURNING_LEFT;
          RCLCPP_INFO(this->get_logger(), "Completed forward movement, starting turn");
        }
        break;

      case State::TURNING_LEFT:
        if (turn_left(cmd_vel)) {
          side_count_++;
          if (side_count_ >= 4) {
            current_state_ = State::COMPLETED;
            RCLCPP_INFO(this->get_logger(), "Square completed successfully!");
          } else {
            current_state_ = State::MOVING_FORWARD;
            RCLCPP_INFO(this->get_logger(), "Completed turn, starting side %d/4", side_count_ + 1);
          }
        }
        break;

      case State::COMPLETED:
        // Stop the robot
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        RCLCPP_INFO_ONCE(this->get_logger(), "Test completed. Robot stopped.");
        break;
    }

    cmd_vel_pub_->publish(cmd_vel);
  }

  /**
   * @brief Move forward for the specified distance
   * @param cmd_vel Command velocity message to populate
   * @return true if movement is complete, false otherwise
   */
  bool move_forward(geometry_msgs::msg::Twist& cmd_vel) {
    // Calculate distance traveled from start of this segment
    double dx = current_odom_.pose.pose.position.x - target_position_.pose.pose.position.x;
    double dy = current_odom_.pose.pose.position.y - target_position_.pose.pose.position.y;
    double distance_traveled = std::sqrt(dx * dx + dy * dy);

    if (distance_traveled >= square_size_ - position_tolerance_) {
      // Movement complete
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;

      // Update target position for next segment
      target_position_ = current_odom_;
      return true;
    } else {
      // Continue moving forward
      cmd_vel.linear.x = linear_speed_;
      cmd_vel.angular.z = 0.0;
      return false;
    }
  }

  /**
   * @brief Turn left 90 degrees
   * @param cmd_vel Command velocity message to populate
   * @return true if turn is complete, false otherwise
   */
  bool turn_left(geometry_msgs::msg::Twist& cmd_vel) {
    // Extract current yaw angle
    tf2::Quaternion current_quat;
    tf2::fromMsg(current_odom_.pose.pose.orientation, current_quat);
    double current_yaw = quaternion_to_yaw(current_quat);

    // Extract target yaw angle
    tf2::Quaternion target_quat;
    tf2::fromMsg(target_position_.pose.pose.orientation, target_quat);
    double target_yaw = quaternion_to_yaw(target_quat);

    // Calculate target yaw (90 degrees left)
    double desired_yaw = target_yaw + M_PI_2;

    // Normalize angles to [-pi, pi]
    desired_yaw = std::atan2(std::sin(desired_yaw), std::cos(desired_yaw));
    current_yaw = std::atan2(std::sin(current_yaw), std::cos(current_yaw));

    // Calculate angular difference
    double angle_diff = desired_yaw - current_yaw;
    angle_diff = std::atan2(std::sin(angle_diff), std::cos(angle_diff));

    if (std::abs(angle_diff) <= angle_tolerance_) {
      // Turn complete
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;

      // Update target orientation for next segment
      target_position_ = current_odom_;
      return true;
    } else {
      // Continue turning
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = (angle_diff > 0) ? angular_speed_ : -angular_speed_;
      return false;
    }
  }

  /**
   * @brief Convert quaternion to yaw angle
   * @param quat Quaternion to convert
   * @return Yaw angle in radians
   */
  double quaternion_to_yaw(const tf2::Quaternion& quat) {
    double siny_cosp = 2 * (quat.w() * quat.z() + quat.x() * quat.y());
    double cosy_cosp = 1 - 2 * (quat.y() * quat.y() + quat.z() * quat.z());
    return std::atan2(siny_cosp, cosy_cosp);
  }

  // Node parameters
  double square_size_;
  double linear_speed_;
  double angular_speed_;
  double position_tolerance_;
  double angle_tolerance_;

  // State machine
  State current_state_;
  int side_count_;

  // ROS interfaces
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Odometry tracking
  bool odom_received_ = false;
  nav_msgs::msg::Odometry current_odom_;
  nav_msgs::msg::Odometry start_position_;
  nav_msgs::msg::Odometry target_position_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  // Parse command line arguments
  double square_size = 0.5;  // Default square size
  if (argc > 1) {
    try {
      square_size = std::stod(argv[1]);
      if (square_size <= 0) {
        std::cerr << "Error: Square size must be positive" << std::endl;
        return 1;
      }
    } catch (const std::exception& e) {
      std::cerr << "Error parsing square size: " << e.what() << std::endl;
      std::cerr << "Usage: " << argv[0] << " [square_size_in_meters]" << std::endl;
      return 1;
    }
  }

  auto node = std::make_shared<SquareRunner>(square_size);

  std::cout << "=== RoboClaw Square Test ===" << std::endl;
  std::cout << "Square size: " << square_size << " meters" << std::endl;
  std::cout << "Press Ctrl+C to stop the test" << std::endl;
  std::cout << "=============================" << std::endl;

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
