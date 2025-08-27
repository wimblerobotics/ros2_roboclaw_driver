// MIT License
// Author: Michael Wimble <mike@wimblerobotics.com>
#pragma once
#include <algorithm>
#include <chrono>
#include <cstdint>
namespace roboclaw_driver {
class CommandShaper {
 public:
  void configure(double accel_qpps_per_sec) {
    accel_qpps_per_sec_ = accel_qpps_per_sec;
  }
  void reset() {
    current_m1_qpps_ = 0;
    current_m2_qpps_ = 0;
    last_update_ = Clock::now();
  }
  // target qpps for each wheel; dt derived internally
  void update(int32_t target_m1, int32_t target_m2) {
    auto now = Clock::now();
    double dt = std::chrono::duration<double>(now - last_update_).count();
    last_update_ = now;
    double max_step = accel_qpps_per_sec_ * dt;
    shapeOne(target_m1, current_m1_qpps_, max_step);
    shapeOne(target_m2, current_m2_qpps_, max_step);
  }
  int32_t shapedM1() const {
    return current_m1_qpps_;
  }
  int32_t shapedM2() const {
    return current_m2_qpps_;
  }

 private:
  using Clock = std::chrono::steady_clock;
  void shapeOne(int32_t target, int32_t& cur, double max_step) {
    double delta = static_cast<double>(target - cur);
    if (delta > max_step)
      delta = max_step;
    else if (delta < -max_step)
      delta = -max_step;
    cur += static_cast<int32_t>(delta);
  }
  double accel_qpps_per_sec_{1000.0};
  int32_t current_m1_qpps_{0};
  int32_t current_m2_qpps_{0};
  Clock::time_point last_update_{Clock::now()};
};
}  // namespace roboclaw_driver
