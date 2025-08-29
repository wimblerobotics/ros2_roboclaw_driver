// MIT License
// Author: Michael Wimble <mike@wimblerobotics.com>
#pragma once
#include <cstdint>
#include <string>
#include <chrono>
#include <optional>
namespace roboclaw_driver {
    struct SafetyConfig {
        bool enabled{ true };
        double overcurrent_limit_m1{ 6.0 };
        double overcurrent_limit_m2{ 6.0 };
        double overcurrent_detect_time{ 0.2 };
        double overcurrent_clear_time{ 1.0 };
        double overcurrent_hysteresis{ 0.5 };
        double temp1_limit{ 80.0 };
        double temp2_limit{ 80.0 };
        double temp_clear_delta{ 5.0 };
        double runaway_speed_factor{ 1.5 };
        double runaway_detect_time{ 0.3 };
        // Stall detection fields retained for backward compatibility but unused.
        double stall_speed_ratio{ 0.1 }; // deprecated
        double stall_min_command{ 100.0 }; // deprecated
        double stall_timeout{ 1.0 }; // deprecated
    };
    struct SafetySample {
        double time_now; // seconds
        int32_t m1_cmd_qpps; int32_t m2_cmd_qpps;
        int32_t m1_meas_qpps; int32_t m2_meas_qpps;
        double m1_current_avg; double m2_current_avg;
        double temp1; double temp2;
    };
    class SafetySupervisor {
    public:
        struct Result { bool estop = false; bool decel = false; std::string source; std::string reason; };
        void configure(const SafetyConfig& cfg) { cfg_ = cfg; reset(); }
        void reset();
        Result evaluate(const SafetySample& s);
    private:
        SafetyConfig cfg_{};
        double m1_overcurrent_start_{ 0 }, m2_overcurrent_start_{ 0 };
        double m1_overcurrent_clear_start_{ 0 }, m2_overcurrent_clear_start_{ 0 };
        double temp1_over_start_{ 0 }, temp2_over_start_{ 0 };
        double runaway_start_{ 0 };
        double stall_start_{ 0 }; // deprecated
    };
} // namespace roboclaw_driver
