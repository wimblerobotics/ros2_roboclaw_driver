// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Wimblerobotics
// https://github.com/wimblerobotics/ros2_roboclaw_driver

#pragma once

#include "roboclaw_cmd.h"

class CmdReadMainBatteryVoltage : public Cmd {
 public:
  CmdReadMainBatteryVoltage(RoboClaw& roboclaw, float& voltage)
      : Cmd(roboclaw, "ReadLMainBatteryVoltage", RoboClaw::kNone), voltage_(voltage) {}

 private:
  void send() override {
    try {
      roboclaw_.appendToWriteLog("ReadLMainBatteryVoltage: WROTE: ");
      float result = ((float)roboclaw_.get2ByteCommandResult2(RoboClaw::GETMBATT)) / 10.0;
      voltage_ = result;
      roboclaw_.appendToReadLog(", RESULT: %f", result);
      return;
    } catch (...) {
      RCUTILS_LOG_ERROR("[RoboClaw::CmdReadMainBatteryVoltage] Uncaught exception !!!");
    }
  }

  float& voltage_;
};
