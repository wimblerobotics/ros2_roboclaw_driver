// MIT License
// Author: Michael Wimble <mike@wimblerobotics.com>
#pragma once
#include <chrono>
#include <cmath>
#include <cstdint>
namespace roboclaw_driver {
  class OdometryIntegrator {
  public:
    void configure(double wheel_radius, double wheel_separation, double pulses_per_rev) {
      wheel_radius_ = wheel_radius;
      wheel_separation_ = wheel_separation;
      pulses_per_rev_ = pulses_per_rev;
    }
    void reset() {
      x_ = 0;
      y_ = 0;
      heading_ = 0;
      have_prev_ = false;
    }
    struct Result {
      double x;
      double y;
      double heading;
      double linear;
      double angular;
    };
    Result update(int64_t enc_left, int64_t enc_right, double dt) {
      if (!have_prev_) {
        prev_left_ = enc_left;
        prev_right_ = enc_right;
        have_prev_ = true;
        return { x_, y_, heading_, 0, 0 };
      }
      int64_t dleft = enc_left - prev_left_;
      prev_left_ = enc_left;
      int64_t dright = enc_right - prev_right_;
      prev_right_ = enc_right;
      double rev_left = static_cast<double>(dleft) / pulses_per_rev_;
      double rev_right = static_cast<double>(dright) / pulses_per_rev_;
      double dist_left = rev_left * 2.0 * M_PI * wheel_radius_;
      double dist_right = rev_right * 2.0 * M_PI * wheel_radius_;
      double dist = (dist_left + dist_right) / 2.0;
      double dtheta = (dist_right - dist_left) / wheel_separation_;
      heading_ += dtheta;
      x_ += dist * cos(heading_);
      y_ += dist * sin(heading_);
      double linear = dist / dt;
      double angular = dtheta / dt;
      return { x_, y_, heading_, linear, angular };
    }

  private:
    double wheel_radius_{ 0.1 };
    double wheel_separation_{ 0.3 };
    double pulses_per_rev_{ 1000.0 };
    double x_{ 0 };
    double y_{ 0 };
    double heading_{ 0 };
    int64_t prev_left_{ 0 };
    int64_t prev_right_{ 0 };
    bool have_prev_{ false };
  };
}  // namespace roboclaw_driver
