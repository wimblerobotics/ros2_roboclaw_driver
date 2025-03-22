#pragma once

#include "roboclaw_cmd.h"

class CmdReadEncoderSpeed : public Cmd {
 public:
  CmdReadEncoderSpeed(RoboClaw &roboclaw, RoboClaw::kMotor motor,
                      int32_t &speed)
      : Cmd(roboclaw, "ReadEncoderSpeed", motor), speed_(speed) {}
  void send() override {
    try {
      roboclaw_.appendToWriteLog("ReadEncoderSpeed: motor: %d (%s), WROTE: ",
                                 motor_, motor_ == RoboClaw::kM1 ? "M1" : "M2");
      speed_ = roboclaw_.getVelocityResult(motor_ == RoboClaw::kM1
                                               ? RoboClaw::kGETM1SPEED
                                               : RoboClaw::kGETM2SPEED);
      roboclaw_.appendToReadLog(", RESULT: %d", speed_);
      return;
    } catch (...) {
      RCUTILS_LOG_ERROR(
          "[RoboClaw::CmdReadEncoderSpeed] Uncaught exception "
          "!!!");
    }
  }

  int32_t &speed_;
};
