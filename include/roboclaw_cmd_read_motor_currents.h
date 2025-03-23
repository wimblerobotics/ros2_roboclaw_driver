#pragma once

#include "roboclaw_cmd.h"

class CmdReadMotorCurrents : public Cmd {
 public:
  CmdReadMotorCurrents(RoboClaw &roboclaw,
                       RoboClaw::TMotorCurrents &motorCurrents)
      : Cmd(roboclaw, "ReadMotorCurrents", RoboClaw::kNone),
        motorCurrents_(motorCurrents) {}
  void send() override {
    try {
      roboclaw_.appendToWriteLog("ReadMotorCurrents: WROTE: ");

      unsigned long currentPair =
          roboclaw_.getUlongCommandResult2(RoboClaw::GETCURRENTS);
      motorCurrents_.m1Current = ((int16_t)(currentPair >> 16)) * 0.010;
      motorCurrents_.m2Current = ((int16_t)(currentPair & 0xFFFF)) * 0.010;
      roboclaw_.appendToReadLog(", RESULT m1 current: %3.4f, m2 current: %3.4f",
                                motorCurrents_.m1Current,
                                motorCurrents_.m2Current);

      if (motorCurrents_.m1Current > roboclaw_.maxM1Current_) {
        roboclaw_.motorAlarms_ |= RoboClaw::kM1_OVER_CURRENT_ALARM;
        RCUTILS_LOG_ERROR(
            "[RoboClaw::CmdReadMotorCurrents] Motor 1 over current. Max "
            "allowed: %6.3f, found: %6.3f",
            roboclaw_.maxM1Current_, motorCurrents_.m1Current);
        roboclaw_.stop();
      } else {
        roboclaw_.motorAlarms_ &= ~RoboClaw::kM1_OVER_CURRENT_ALARM;
      }

      if (motorCurrents_.m2Current > roboclaw_.maxM2Current_) {
        roboclaw_.motorAlarms_ |= RoboClaw::kM2_OVER_CURRENT_ALARM;
        RCUTILS_LOG_ERROR(
            "[RoboClaw::CmdReadMotorCurrents] Motor 2 over current. Max "
            "allowed: %6.3f, found: %6.3f",
            roboclaw_.maxM2Current_, motorCurrents_.m2Current);
        roboclaw_.stop();
      } else {
        roboclaw_.motorAlarms_ &= ~RoboClaw::kM2_OVER_CURRENT_ALARM;
      }
    } catch (...) {
      RCUTILS_LOG_ERROR(
          "[RoboClaw::CmdReadMotorCurrents] Uncaught exception !!!");
    }
  }

 private:
  RoboClaw::TMotorCurrents &motorCurrents_;
};
