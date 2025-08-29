#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ros2_roboclaw_driver/msg/robo_claw_status.hpp>
#include <chrono>
#include <fstream>
#include <filesystem>
#include <iomanip>

namespace fs = std::filesystem;
using roboclaw_status_t = ros2_roboclaw_driver::msg::RoboClawStatus;

class PassiveCaptureNode : public rclcpp::Node {
public:
  PassiveCaptureNode() : Node("roboclaw_passive_capture") {
    declare_parameter<std::string>("output_path", "");
    declare_parameter<int>("sample_limit", 1000);
    declare_parameter<double>("time_limit_sec", 0.0); // 0 disables
    get_parameter("output_path", output_path_);
    get_parameter("sample_limit", sample_limit_);
    get_parameter("time_limit_sec", time_limit_sec_);

    if (output_path_.empty()) {
      auto t = std::chrono::system_clock::now();
      std::time_t tt = std::chrono::system_clock::to_time_t(t);
      std::tm tm = *std::localtime(&tt);
      std::ostringstream oss; oss << "golden_capture_" << std::put_time(&tm, "%Y%m%d_%H%M%S");
      output_path_ = oss.str();
    }
    fs::create_directories(output_path_);
    status_csv_.open(fs::path(output_path_) / "status_samples.csv");
    status_csv_ << "t,odom_x,odom_y,odom_yaw,m1_cmd,m1_meas,m1_enc,m2_cmd,m2_meas,m2_enc,main_v,logic_v,temp1,temp2,error_bits,crc_errors,io_errors,retries\n";

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odom", 50, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { last_odom_ = *m; have_odom_ = true; });
    status_sub_ = create_subscription<roboclaw_status_t>("/roboclaw/status", 50, [this](roboclaw_status_t::ConstSharedPtr m) { onStatus(m); });

    start_time_ = now();
  }
private:
  void onStatus(const roboclaw_status_t::ConstSharedPtr& msg) {
    if (!have_odom_) return;
    double t = this->now().seconds();
    double yaw = extractYaw(last_odom_);
    status_csv_ << std::fixed << std::setprecision(6)
      << t << ','
      << last_odom_.pose.pose.position.x << ','
      << last_odom_.pose.pose.position.y << ','
      << yaw << ','
      << msg->m1_speed_command << ','
      << msg->m1_speed_measured << ','
      << msg->m1_encoder_value << ','
      << msg->m2_speed_command << ','
      << msg->m2_speed_measured << ','
      << msg->m2_encoder_value << ','
      << msg->main_battery_voltage << ','
      << msg->logic_battery_voltage << ','
      << msg->temperature1 << ','
      << msg->temperature2 << ','
      << msg->error_bits << ','
      << msg->crc_error_count << ','
      << msg->io_error_count << ','
      << msg->retry_count << '\n';
    if (++samples_ >= static_cast<size_t>(sample_limit_)) finish();
    if (time_limit_sec_ > 0.0 && (t - start_time_.seconds()) >= time_limit_sec_) finish();
  }
  double extractYaw(const nav_msgs::msg::Odometry& o) {
    auto& q = o.pose.pose.orientation;
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }
  void finish() {
    if (!finished_) {
      finished_ = true;
      RCLCPP_INFO(get_logger(), "Passive capture complete: %zu samples", samples_);
      rclcpp::shutdown();
    }
  }
  std::string output_path_;
  int sample_limit_{}; double time_limit_sec_{}; size_t samples_{ 0 }; bool finished_{ false };
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<roboclaw_status_t>::SharedPtr status_sub_;
  nav_msgs::msg::Odometry last_odom_; bool have_odom_{ false };
  rclcpp::Time start_time_;
  std::ofstream status_csv_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PassiveCaptureNode>());
  rclcpp::shutdown();
  return 0;
}
