#pragma once

#include "roboclaw_cmd.h"

class CmdSetEncoderValue : public Cmd {
 public:
  CmdSetEncoderValue(RoboClaw &roboclaw, RoboClaw::kMotor motor, long value)
      : Cmd(roboclaw, "SetEncoderValue", motor), value_(value) {}

  void send() override {
    roboclaw_.appendToWriteLog(
        "SetEncoderValue: motor: %d (%s) value: %ld, WROTE: ", motor_,
        RoboClaw::motorNames_[motor_], value_);
    try {
      roboclaw_.writeN2(true, 6, roboclaw_.portAddress_,
                        motor_ == RoboClaw::kM1 ? RoboClaw::kSETM1ENCODER
                                                : RoboClaw::kSETM2ENCODER,
                        SetDWORDval(value_));
      return;
    } catch (...) {
      RCUTILS_LOG_ERROR(
          "[RoboClaw::CmdSetEncoderValue] Uncaught exception !!!");
    }
  }

  long value_;
};
