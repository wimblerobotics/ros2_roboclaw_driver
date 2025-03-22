#pragma once

#include "roboclaw_cmd.h"

class CmdDoBufferedM1M2DriveSpeedAccelDistance : public Cmd {
 public:
  CmdDoBufferedM1M2DriveSpeedAccelDistance(
      RoboClaw &roboclaw, uint32_t accel_quad_pulses_per_second,
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
  void send() override {
    try {
      roboclaw_.appendToWriteLog(
          "BufferedM1M2WithSignedSpeedAccelDist: accel: %d, m1Speed: %d, "
          "m1Distance: %d, m2Speed: %d, m2Distance: %d, WROTE: ",
          accel_quad_pulses_per_second_, m1_speed_quad_pulses_per_second_,
          m1_max_distance_quad_pulses_, m2_speed_quad_pulses_per_second_,
          m2_max_distance_quad_pulses_);
      roboclaw_.writeN2(true, 23, roboclaw_.portAddress_,
                        RoboClaw::kMIXEDSPEEDACCELDIST,
                        SetDWORDval(accel_quad_pulses_per_second_),
                        SetDWORDval(m1_speed_quad_pulses_per_second_),
                        SetDWORDval(m1_max_distance_quad_pulses_),
                        SetDWORDval(m2_speed_quad_pulses_per_second_),
                        SetDWORDval(m2_max_distance_quad_pulses_),
                        1 /* Cancel any previous command. */
      );
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
