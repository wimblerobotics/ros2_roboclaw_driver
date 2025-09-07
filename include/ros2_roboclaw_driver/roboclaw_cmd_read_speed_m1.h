// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Wimblerobotics
// https://github.com/wimblerobotics/ros2_roboclaw_driver

#pragma once

#include "roboclaw_cmd.h"

class CmdReadSpeedM1 : public Cmd {
 public:
  CmdReadSpeedM1(RoboClaw& roboclaw, int32_t& speed) : Cmd(roboclaw, "CmdReadSpeedM1", RoboClaw::kM1), speed_(speed) {}

 private:
  void send() override {
    try {
      uint16_t crc = 0;
      roboclaw_.updateCrc(crc, roboclaw_.portAddress_);
      roboclaw_.updateCrc(crc, RoboClaw::GETM1ISPEED);

      roboclaw_.appendToWriteLog("CmdReadSpeedM1: WROTE: ");
      roboclaw_.writeN2(false, 2, roboclaw_.portAddress_, RoboClaw::GETM1ISPEED);
      int32_t result = (int32_t)roboclaw_.getULongCont2(crc);
      uint8_t direction = roboclaw_.readByteWithTimeout2();
      roboclaw_.updateCrc(crc, direction);
      speed_ = result * (direction == 0 ? 1 : -1);
      uint16_t responseCrc = 0;
      uint16_t datum = roboclaw_.readByteWithTimeout2();
      responseCrc = datum << 8;
      datum = roboclaw_.readByteWithTimeout2();
      responseCrc |= datum;
      roboclaw_.appendToReadLog(", RESULT: %d", result);
      if (responseCrc == crc) {
        return;
      } else {
        RCUTILS_LOG_ERROR(
            "[RoboClaw::CmdReadSpeedM1] invalid CRC "
            "expected: 0x%2X, got: "
            "0x%2X",
            crc, responseCrc);
      }
      return;
    } catch (...) {
      RCUTILS_LOG_ERROR("[RoboClaw::CmdReadSpeedM1] Uncaught exception !!!");
    }
  }

  int32_t& speed_;
};
