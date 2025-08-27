// MIT License
// Author: Michael Wimble <mike@wimblerobotics.com>
#include "roboclaw_driver/safety_supervisor.hpp"
#include <cmath>
namespace roboclaw_driver {
    void SafetySupervisor::reset() {
        m1_overcurrent_start_ = m2_overcurrent_start_ = 0;
        m1_overcurrent_clear_start_ = m2_overcurrent_clear_start_ = 0;
        temp1_over_start_ = temp2_over_start_ = 0;
        runaway_start_ = 0; stall_start_ = 0;
    }
    SafetySupervisor::Result SafetySupervisor::evaluate(const SafetySample& s) {
        Result r; if (!cfg_.enabled) return r; // nothing
        // Overcurrent detection
        auto oc_check = [&](double current, double limit, double& start_over, double& start_clear, const char* src_current, const char* src_over) {
            if (current > limit) {
                start_clear = 0; // reset clear timer
                if (start_over == 0) start_over = s.time_now;
                else if ((s.time_now - start_over) >= cfg_.overcurrent_detect_time) {
                    r.estop = true; r.source = src_over; r.reason = "overcurrent"; return;
                }
            }
            else if (current < (limit - cfg_.overcurrent_hysteresis)) {
                start_over = 0;
                if (start_clear == 0) start_clear = s.time_now;
                else if ((s.time_now - start_clear) >= cfg_.overcurrent_clear_time) {
                    // auto clear implied
                }
            }
            };
        oc_check(s.m1_current_avg, cfg_.overcurrent_limit_m1, m1_overcurrent_start_, m1_overcurrent_clear_start_, "m1_current", "estop_overcurrent_m1");
        if (r.estop) return r;
        oc_check(s.m2_current_avg, cfg_.overcurrent_limit_m2, m2_overcurrent_start_, m2_overcurrent_clear_start_, "m2_current", "estop_overcurrent_m2");
        if (r.estop) return r;
        // Temperature
        if (s.temp1 > cfg_.temp1_limit) {
            if (temp1_over_start_ == 0) temp1_over_start_ = s.time_now; else if ((s.time_now - temp1_over_start_) > 0.0) { r.estop = true; r.source = "estop_temp1"; r.reason = "temp1_over"; return r; }
        }
        else if (s.temp1 < cfg_.temp1_limit - cfg_.temp_clear_delta) temp1_over_start_ = 0;
        if (s.temp2 > cfg_.temp2_limit) {
            if (temp2_over_start_ == 0) temp2_over_start_ = s.time_now; else if ((s.time_now - temp2_over_start_) > 0.0) { r.estop = true; r.source = "estop_temp2"; r.reason = "temp2_over"; return r; }
        }
        else if (s.temp2 < cfg_.temp2_limit - cfg_.temp_clear_delta) temp2_over_start_ = 0;
        // Runaway (if measured >> commanded)
        int32_t abs_cmd = std::max(std::abs(s.m1_cmd_qpps), std::abs(s.m2_cmd_qpps));
        int32_t abs_meas = std::max(std::abs(s.m1_meas_qpps), std::abs(s.m2_meas_qpps));
        if (abs_meas > static_cast<int32_t>(cfg_.runaway_speed_factor * std::max(abs_cmd, 100))) {
            if (runaway_start_ == 0) runaway_start_ = s.time_now; else if ((s.time_now - runaway_start_) >= cfg_.runaway_detect_time) { r.estop = true; r.source = "estop_runaway"; r.reason = "runaway_speed"; return r; }
        }
        else runaway_start_ = 0;
        // Stall (commanded high but measured low)
        if (abs_cmd > cfg_.stall_min_command && abs_meas < static_cast<int32_t>(cfg_.stall_speed_ratio * abs_cmd)) {
            if (stall_start_ == 0) stall_start_ = s.time_now; else if ((s.time_now - stall_start_) >= cfg_.stall_timeout) { r.estop = true; r.source = "estop_stall"; r.reason = "stall"; return r; }
        }
        else stall_start_ = 0;
        return r;
    }
} // namespace roboclaw_driver
