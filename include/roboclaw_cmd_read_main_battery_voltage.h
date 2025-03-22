#pragma once

#include "roboclaw_cmd.h"

class CmdReadMainBatteryVoltage : public Cmd {
 public:
  CmdReadMainBatteryVoltage(RoboClaw &roboclaw, float &voltage)
      : Cmd(roboclaw, "ReadLMainBatteryVoltage", RoboClaw::kNone),
        voltage_(voltage) {}
  void send() override {
    try {
      roboclaw_.appendToWriteLog("ReadLMainBatteryVoltage: WROTE: ");
      float result =
          ((float)roboclaw_.get2ByteCommandResult2(RoboClaw::kGETMBATT)) / 10.0;
      voltage_ = result;
      roboclaw_.appendToReadLog(", RESULT: %f", result);
      return;
    } catch (...) {
      RCUTILS_LOG_ERROR(
          "[RoboClaw::CmdReadMainBatteryVoltage] Uncaught exception !!!");
    }
  }

  float &voltage_;
};
