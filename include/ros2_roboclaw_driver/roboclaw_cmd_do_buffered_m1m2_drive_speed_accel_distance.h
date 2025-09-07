// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Wimblerobotics
// https://github.com/wimblerobotics/ros2_roboclaw_driver

#pragma once

#include <chrono>

#include "roboclaw_cmd.h"

class CmdDoBufferedM1M2DriveSpeedAccelDistance : public Cmd {
 public:
  CmdDoBufferedM1M2DriveSpeedAccelDistance(RoboClaw& roboclaw, uint32_t accel_quad_pulses_per_second,
                                           int32_t m1_speed_quad_pulses_per_second,
                                           uint32_t m1_max_distance_quad_pulses,
                                           int32_t m2_speed_quad_pulses_per_second,
                                           uint32_t m2_max_distance_quad_pulses)
      : Cmd(roboclaw, "DoBufferedDriveSpeedAccelDistance", RoboClaw::kNone),
        accel_quad_pulses_per_second_(accel_quad_pulses_per_second),
        m1_speed_quad_pulses_per_second_(m1_speed_quad_pulses_per_second),
        m1_max_distance_quad_pulses_(m1_max_distance_quad_pulses),
        m2_speed_quad_pulses_per_second_(m2_speed_quad_pulses_per_second),
        m2_max_distance_quad_pulses_(m2_max_distance_quad_pulses) {}

 private:
  void send() override {
    try {
      // auto send_start = std::chrono::steady_clock::now();
      // Unthrottled log so we can validate one command per cmd_vel (was
      // RCUTILS_LOG_INFO_THROTTLE) RCUTILS_LOG_INFO(
      //     "CmdDoBufferedM1M2DriveSpeedAccelDistance::send starting at %ld ms
      //     (accel: %d, m1Speed:
      //     "
      //     "%d, m1Distance: %d, m2Speed: %d, m2Distance: %d)",
      //     std::chrono::duration_cast<std::chrono::milliseconds>(send_start.time_since_epoch())
      //         .count(),
      //     accel_quad_pulses_per_second_, m1_speed_quad_pulses_per_second_,
      //     m1_max_distance_quad_pulses_, m2_speed_quad_pulses_per_second_,
      //     m2_max_distance_quad_pulses_);
      roboclaw_.appendToWriteLog(
          "BufferedM1M2WithSignedSpeedAccelDist: accel: %d, m1Speed: %d, "
          "m1Distance: %d, m2Speed: %d, m2Distance: %d, WROTE: ",
          accel_quad_pulses_per_second_, m1_speed_quad_pulses_per_second_, m1_max_distance_quad_pulses_,
          m2_speed_quad_pulses_per_second_, m2_max_distance_quad_pulses_);
      roboclaw_.writeN2(true, 23, roboclaw_.portAddress_, RoboClaw::MIXEDSPEEDACCELDIST,
                        SetDWORDval(accel_quad_pulses_per_second_), SetDWORDval(m1_speed_quad_pulses_per_second_),
                        SetDWORDval(m1_max_distance_quad_pulses_), SetDWORDval(m2_speed_quad_pulses_per_second_),
                        SetDWORDval(m2_max_distance_quad_pulses_), 1 /* Cancel any previous command. */
      );

      // auto send_end = std::chrono::steady_clock::now();
      // auto send_duration =
      //     std::chrono::duration_cast<std::chrono::milliseconds>(send_end -
      //     send_start).count();
      // RCUTILS_LOG_INFO(
      //     "CmdDoBufferedM1M2DriveSpeedAccelDistance::send completed at %ld ms
      //     (duration: %ld ms)",
      //     std::chrono::duration_cast<std::chrono::milliseconds>(send_end.time_since_epoch())
      //         .count(),
      //     send_duration);
    } catch (RoboClaw::TRoboClawException* e) {
      RCUTILS_LOG_ERROR(
          "[RoboClaw::CmdDoBufferedM1M2DriveSpeedAccelDistance] RoboClaw "
          "exception: %s",
          e->what());
    } catch (std::exception& e) {
      RCUTILS_LOG_ERROR(
          "[RoboClaw::CmdDoBufferedM1M2DriveSpeedAccelDistance] Standard "
          "exception: %s",
          e.what());
    } catch (...) {
      RCUTILS_LOG_ERROR(
          "[RoboClaw::CmdDoBufferedM1M2DriveSpeedAccelDistance] Uncaught "
          "exception !!!");
    }
  }

  uint32_t accel_quad_pulses_per_second_;
  int32_t m1_speed_quad_pulses_per_second_;
  uint32_t m1_max_distance_quad_pulses_;
  int32_t m2_speed_quad_pulses_per_second_;
  uint32_t m2_max_distance_quad_pulses_;
};
