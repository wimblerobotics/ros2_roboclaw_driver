#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <filesystem>
#include "roboclaw_driver/roboclaw_device.hpp"
#include "roboclaw_driver/transport.hpp"

using namespace std::chrono_literals;
namespace fs = std::filesystem;

namespace roboclaw_driver {

  class CaptureNode : public rclcpp::Node {
  public:
    CaptureNode() : Node("roboclaw_capture_harness") {
      declare_parameter<std::string>("device", "/dev/ttyACM0");
      declare_parameter<int>("baud", 115200);
      declare_parameter<int>("address", 0x80);
      declare_parameter<int>("repetitions_non_motion", 50);
      declare_parameter<int>("repetitions_motion", 10);
      declare_parameter<double>("speed_linear_test", 0.2);
      declare_parameter<double>("speed_angular_test", 0.6);
      declare_parameter<double>("motion_duration_sec", 2.0);
      declare_parameter<double>("rest_duration_sec", 0.5);
      declare_parameter<std::string>("output_path", "");
      declare_parameter<double>("wheel_separation", 0.40);
      declare_parameter<int>("ticks_per_rev", 2048);

      get_parameter("device", device_);
      get_parameter("baud", baud_);
      get_parameter("address", address_);
      get_parameter("repetitions_non_motion", reps_non_motion_);
      get_parameter("repetitions_motion", reps_motion_);
      get_parameter("speed_linear_test", speed_linear_);
      get_parameter("speed_angular_test", speed_angular_);
      get_parameter("motion_duration_sec", motion_duration_);
      get_parameter("rest_duration_sec", rest_duration_);
      get_parameter("output_path", output_path_);
      get_parameter("wheel_separation", wheel_sep_);
      get_parameter("ticks_per_rev", ticks_per_rev_);

      if (reps_motion_ > 20) { RCLCPP_WARN(get_logger(), "Clamping repetitions_motion from %d to 20", reps_motion_); reps_motion_ = 20; }

      if (output_path_.empty()) {
        auto t = std::chrono::system_clock::now();
        std::time_t tt = std::chrono::system_clock::to_time_t(t);
        std::tm tm = *std::localtime(&tt);
        std::ostringstream oss; oss << "capture_" << std::put_time(&tm, "%Y%m%d_%H%M%S");
        output_path_ = oss.str();
      }
      fs::create_directories(output_path_);

      cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odom", 10,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { latest_odom_ = *msg; });

      transport_ = std::make_shared<SerialTransport>(device_, baud_);
      device_obj_ = std::make_unique<RoboClawDevice>(transport_, static_cast<uint8_t>(address_));

      cmd_log_.open(fs::path(output_path_) / "commands.csv");
      motion_log_.open(fs::path(output_path_) / "motions.csv");
      cmd_log_ << "ts,command,idx,success,latency_ms,tx,rx,detail\n";
      motion_log_ << "segment,trial,type,cmd_m1,cmd_m2,enc_m1_before,enc_m2_before,enc_m1_after,enc_m2_after,delta_left,delta_right,delta_s,delta_theta_enc,delta_theta_odom,odom_x,odom_y,odom_theta,linear_drift,angular_error,sample_speed_m1,sample_speed_m2\n";

      timer_ = create_wall_timer(200ms, std::bind(&CaptureNode::tick, this));
      phase_ = Phase::NON_MOTION;
      startNonMotion();
    }

  private:
    enum class Phase { NON_MOTION, MOTION_SEQUENCE, DONE };
    Phase phase_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    nav_msgs::msg::Odometry latest_odom_;
    std::shared_ptr<SerialTransport> transport_;
    std::unique_ptr<RoboClawDevice> device_obj_;
    std::ofstream cmd_log_, motion_log_;

    std::string device_; int baud_; int address_;
    int reps_non_motion_; int reps_motion_;
    double speed_linear_, speed_angular_, motion_duration_, rest_duration_;
    std::string output_path_;
    double wheel_sep_; int ticks_per_rev_;

    // Non-motion tracking
    std::vector<std::string> commands_ = { "version","encoders","velocities","currents","voltages","temps","status","pid1","pid2" };
    size_t cmd_index_ = 0; int cmd_iter_ = 0;
    rclcpp::Time cmd_start_; std::string last_err_;

    // Motion tracking
    enum class MotionType { FWD, REV, ROTL, ROTR };
    int motion_trial_ = 0; MotionType motion_type_ = MotionType::FWD; rclcpp::Time motion_start_;
    uint32_t enc1_before_ = 0, enc2_before_ = 0; double odom_x_before_ = 0, odom_y_before_ = 0, odom_th_before_ = 0;

    rclcpp::TimerBase::SharedPtr timer_;

    void startNonMotion() { cmd_index_ = 0; cmd_iter_ = 0; phase_ = Phase::NON_MOTION; }

    void tick() {
      if (phase_ == Phase::NON_MOTION) {
        if (cmd_index_ >= commands_.size()) { phase_ = Phase::MOTION_SEQUENCE; motion_trial_ = 0; motion_type_ = MotionType::FWD; startMotionSegment(); return; }
        if (cmd_iter_ >= reps_non_motion_) { cmd_index_++; cmd_iter_ = 0; return; }
        runOneCommand(commands_[cmd_index_], cmd_iter_);
        cmd_iter_++;
        return;
      }
      if (phase_ == Phase::MOTION_SEQUENCE) {
        runMotion();
        return;
      }
    }

    void runOneCommand(const std::string& name, int idx) {
      rclcpp::Time t0 = now();
      bool success = false;
      std::string detail;
      std::string err;
      auto& dev = *device_obj_;
      if (name == "version") {
        auto v = dev.version();
        success = !v.empty();
        detail = v;
      } else if (name == "encoders") {
        Snapshot s; if (dev.readSnapshot(s, err)) { success = true; detail = std::to_string(s.m1_enc.value) + "," + std::to_string(s.m2_enc.value); }
      } else if (name == "velocities") {
        int32_t v1, v2; if (dev.readMotorVelocity(1, v1, err) && dev.readMotorVelocity(2, v2, err)) { success = true; detail = std::to_string(v1) + "," + std::to_string(v2); }
      } else if (name == "currents") {
        Snapshot s; if (dev.readSnapshot(s, err)) { success = true; detail = std::to_string((int)(s.currents.m1_inst * 100)) + "," + std::to_string((int)(s.currents.m2_inst * 100)); }
      } else if (name == "voltages") {
        Snapshot s; if (dev.readSnapshot(s, err)) { success = true; detail = std::to_string((int)(s.volts.main * 10)) + "," + std::to_string((int)(s.volts.logic * 10)); }
      } else if (name == "temps") {
        Snapshot s; if (dev.readSnapshot(s, err)) { success = true; detail = std::to_string((int)(s.temps.t1 * 10)) + "," + std::to_string((int)(s.temps.t2 * 10)); }
      } else if (name == "status") {
        Snapshot s; if (dev.readSnapshot(s, err)) { success = true; detail = std::to_string(s.status_bits); }
      } else if (name == "pid1") {
        PIDSnapshot p; if (dev.readPID(1, p, err)) { success = true; detail = std::to_string(p.p) + "," + std::to_string(p.i) + "," + std::to_string(p.d); }
      } else if (name == "pid2") {
        PIDSnapshot p; if (dev.readPID(2, p, err)) { success = true; detail = std::to_string(p.p) + "," + std::to_string(p.i) + "," + std::to_string(p.d); }
      }
      rclcpp::Time t1 = now();
      double latency_ms = (t1 - t0).seconds() * 1000.0;
      cmd_log_ << std::fixed << std::setprecision(3) << t0.seconds() << "," << name << "," << idx << "," << (success ? 1 : 0) << "," << latency_ms << "," << dev.lastTx() << "," << dev.lastRx() << "," << detail << "\n";
    }

    void startMotionSegment() {
      if (motion_trial_ >= reps_motion_) { phase_ = Phase::DONE; RCLCPP_INFO(get_logger(), "Capture complete"); return; }
      // Determine cmd
      double lin = 0, ang = 0; std::string type_str;
      switch (motion_type_) {
      case MotionType::FWD: lin = speed_linear_; type_str = "FWD"; break;
      case MotionType::REV: lin = -speed_linear_; type_str = "REV"; break;
      case MotionType::ROTL: ang = speed_angular_; type_str = "ROTL"; break;
      case MotionType::ROTR: ang = -speed_angular_; type_str = "ROTR"; break;
      }
      Snapshot s; std::string err; device_obj_->readSnapshot(s, err); enc1_before_ = s.m1_enc.value; enc2_before_ = s.m2_enc.value;
      odom_x_before_ = latest_odom_.pose.pose.position.x; odom_y_before_ = latest_odom_.pose.pose.position.y;
      double qw = latest_odom_.pose.pose.orientation.w, qx = latest_odom_.pose.pose.orientation.x, qy = latest_odom_.pose.pose.orientation.y, qz = latest_odom_.pose.pose.orientation.z;
      double yaw = std::atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)); odom_th_before_ = yaw;
      geometry_msgs::msg::Twist tw; tw.linear.x = lin; tw.angular.z = ang; cmd_pub_->publish(tw);
      motion_start_ = now();
      current_type_str_ = type_str;
    }

    void runMotion() {
      if (phase_ != Phase::MOTION_SEQUENCE) return;
      auto elapsed = (now() - motion_start_).seconds();
      if (elapsed < motion_duration_) {
        if (!sampled_speed_ && elapsed > motion_duration_ * 0.25) {
          int32_t v1, v2; std::string err; if (device_obj_->readMotorVelocity(1, v1, err) && device_obj_->readMotorVelocity(2, v2, err)) { sample_v1_ = v1; sample_v2_ = v2; sampled_speed_ = true; }
        }
        return;
      }
      geometry_msgs::msg::Twist zero; cmd_pub_->publish(zero);
      rclcpp::sleep_for(std::chrono::milliseconds((int)(rest_duration_ * 1000.0)));
      Snapshot s; std::string err; device_obj_->readSnapshot(s, err); uint32_t enc1_after = s.m1_enc.value, enc2_after = s.m2_enc.value;
      double qw = latest_odom_.pose.pose.orientation.w, qx = latest_odom_.pose.pose.orientation.x, qy = latest_odom_.pose.pose.orientation.y, qz = latest_odom_.pose.pose.orientation.z;
      double yaw = std::atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));
      double delta_left = (double)((int64_t)enc1_after - (int64_t)enc1_before_);
      double delta_right = (double)((int64_t)enc2_after - (int64_t)enc2_before_);
      double delta_s = (delta_left + delta_right) / 2.0;
      double delta_theta_enc = (delta_right - delta_left) / wheel_sep_;
      double delta_theta_odom = yaw - odom_th_before_;
      double odom_x = latest_odom_.pose.pose.position.x; double odom_y = latest_odom_.pose.pose.position.y;
      double linear_drift = std::hypot(odom_x - odom_x_before_, odom_y - odom_y_before_);
      double angular_error = std::fabs(delta_theta_enc - delta_theta_odom);
      motion_log_ << current_type_str_ << "," << motion_trial_ << "," << current_type_str_ << "," << device_obj_->getLastCommand1() << "," << device_obj_->getLastCommand2() << "," << enc1_before_ << "," << enc2_before_ << "," << enc1_after << "," << enc2_after << "," << delta_left << "," << delta_right << "," << delta_s << "," << delta_theta_enc << "," << delta_theta_odom << "," << odom_x << "," << odom_y << "," << yaw << "," << linear_drift << "," << angular_error << "," << sample_v1_ << "," << sample_v2_ << "\n";
      // prepare next type
      sampled_speed_ = false; sample_v1_ = 0; sample_v2_ = 0;
      switch (motion_type_) {
      case MotionType::FWD: motion_type_ = MotionType::REV; break;
      case MotionType::REV: motion_type_ = MotionType::ROTL; break;
      case MotionType::ROTL: motion_type_ = MotionType::ROTR; break;
      case MotionType::ROTR: motion_type_ = MotionType::FWD; motion_trial_++; break;
      }
      startMotionSegment();
    }

    std::string current_type_str_;
    bool sampled_speed_ = false; int32_t sample_v1_ = 0, sample_v2_ = 0;
  };

}  // namespace roboclaw_driver

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<roboclaw_driver::CaptureNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
