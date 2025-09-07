// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Wimblerobotics
// https://github.com/wimblerobotics/ros2_roboclaw_driver

#pragma once

#include "roboclaw_cmd.h"

class CmdReadEncoderSpeed : public Cmd {
 public:
  CmdReadEncoderSpeed(RoboClaw& roboclaw, RoboClaw::kMotor motor, int32_t& speed)
      : Cmd(roboclaw, "ReadEncoderSpeed", motor), speed_(speed) {}

 private:
  void send() override {
    try {
      roboclaw_.appendToWriteLog("ReadEncoderSpeed: motor: %d (%s), WROTE: ", motor_,
                                 motor_ == RoboClaw::kM1 ? "M1" : "M2");
      speed_ = roboclaw_.getVelocityResult(motor_ == RoboClaw::kM1 ? kGETM1SPEED : kGETM2SPEED);
      roboclaw_.appendToReadLog(", RESULT: %d", speed_);
      return;
    } catch (...) {
      RCUTILS_LOG_ERROR(
          "[RoboClaw::CmdReadEncoderSpeed] Uncaught exception "
          "!!!");
    }
  }

  typedef enum WHICH_VELOCITY {
    kGETM1SPEED = 18,
    kGETM2SPEED = 19,
  } WHICH_VELOCITY;

  int32_t& speed_;
};
