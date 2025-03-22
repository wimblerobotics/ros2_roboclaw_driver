#pragma once

#include "roboclaw_cmd.h"

class CmdReadLogicBatteryVoltage : public Cmd {
 public:
  CmdReadLogicBatteryVoltage(RoboClaw &roboclaw, float &voltage)
      : Cmd(roboclaw, "ReadLogicBatteryVoltage", RoboClaw::kNone),
        voltage_(voltage) {}
  void send() override {
    try {
      roboclaw_.appendToWriteLog("ReadLogicBatteryVoltage: WROTE: ");
      float result =
          ((float)roboclaw_.get2ByteCommandResult2(RoboClaw::kGETLBATT)) / 10.0;
      voltage_ = result;
      roboclaw_.appendToReadLog(", RESULT: %f", result);
      return;
    } catch (...) {
      RCUTILS_LOG_ERROR(
          "[RoboClaw::CmdReadLogicBatteryVoltage] Uncaught exception !!!");
    }
  }

  float &voltage_;
};
