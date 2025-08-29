#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ros2_roboclaw_driver/msg/robo_claw_status.hpp>
#include <chrono>
#include <cmath>
#include <vector>

using namespace std::chrono_literals;
using roboclaw_status_t = ros2_roboclaw_driver::msg::RoboClawStatus;

class SquareRunner : public rclcpp::Node {
public:
  SquareRunner() : Node("square_runner") {
    declare_parameter<double>("side_length", 0.5);
    declare_parameter<double>("linear_speed", 0.15); // m/s
    declare_parameter<double>("angular_speed", 0.6); // rad/s
    declare_parameter<double>("stop_margin", 0.01);  // m
    declare_parameter<double>("angle_margin", 0.02); // rad
    get_parameter("side_length", side_);
    get_parameter("linear_speed", v_lin_);
    get_parameter("angular_speed", v_ang_);
    get_parameter("stop_margin", stop_margin_);
    get_parameter("angle_margin", angle_margin_);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odom", 25, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { latest_odom_ = *m; have_odom_ = true; });
    status_sub_ = create_subscription<roboclaw_status_t>("/roboclaw/status", 10, [this](roboclaw_status_t::ConstSharedPtr m) { last_status_ = *m; });

    timer_ = create_wall_timer(50ms, std::bind(&SquareRunner::tick, this));
  }
private:
  enum class Phase { FORWARD, TURN, DONE };
  struct EdgeState { double start_x; double start_y; double start_yaw; };
  Phase phase_{ Phase::FORWARD };
  int edges_done_{ 0 };
  EdgeState current_{};

  void tick() {
    if (!have_odom_) return;
    if (edges_done_ >= 4) { stop(); if (phase_ != Phase::DONE) { phase_ = Phase::DONE; RCLCPP_INFO(get_logger(), "Square complete"); rclcpp::shutdown(); } return; }
    double x = latest_odom_.pose.pose.position.x;
    double y = latest_odom_.pose.pose.position.y;
    double yaw = extractYaw(latest_odom_);
    if (!have_edge_start_) { current_.start_x = x; current_.start_y = y; current_.start_yaw = yaw; have_edge_start_ = true; phase_ = Phase::FORWARD; }
    if (phase_ == Phase::FORWARD) {
      double dx = x - current_.start_x; double dy = y - current_.start_y; double dist = std::sqrt(dx * dx + dy * dy);
      if (dist >= side_ - stop_margin_) { stop(); phase_ = Phase::TURN; turn_start_yaw_ = yaw; } else { sendLinear(); }
    } else if (phase_ == Phase::TURN) {
      double yaw_now = yaw; double delta = normalizeAngle(yaw_now - turn_start_yaw_); if (delta >= M_PI_2 - angle_margin_) { stop(); edges_done_++; have_edge_start_ = false; } else { sendTurn(); }
    }
  }
  void sendLinear() { geometry_msgs::msg::Twist tw; tw.linear.x = v_lin_; cmd_pub_->publish(tw); }
  void sendTurn() { geometry_msgs::msg::Twist tw; tw.angular.z = v_ang_; cmd_pub_->publish(tw); }
  void stop() { geometry_msgs::msg::Twist tw; cmd_pub_->publish(tw); }
  double extractYaw(const nav_msgs::msg::Odometry& o) { auto& q = o.pose.pose.orientation; double s = 2 * (q.w * q.z + q.x * q.y); double c = 1 - 2 * (q.y * q.y + q.z * q.z); return std::atan2(s, c); }
  double normalizeAngle(double a) { while (a < 0) a += 2 * M_PI; while (a >= 2 * M_PI) a -= 2 * M_PI; return a; }
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<roboclaw_status_t>::SharedPtr status_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  nav_msgs::msg::Odometry latest_odom_; roboclaw_status_t last_status_{}; bool have_odom_{ false }; bool have_edge_start_{ false };
  double side_{}, v_lin_{}, v_ang_{}, stop_margin_{}, angle_margin_{}; double turn_start_yaw_{};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<SquareRunner>()); rclcpp::shutdown(); return 0;
}
