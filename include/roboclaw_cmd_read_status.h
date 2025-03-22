#pragma once

#include "roboclaw_cmd.h"

class CmdReadStatus : public Cmd {
 public:
  CmdReadStatus(RoboClaw &roboclaw, uint16_t &status)
      : Cmd(roboclaw, "ReadStatus", RoboClaw::kNone), status_(status) {}
  void send() override {
    try {
      uint16_t crc = 0;
      roboclaw_.updateCrc(crc, roboclaw_.portAddress_);
      roboclaw_.updateCrc(crc, RoboClaw::kGETERROR);
      roboclaw_.appendToWriteLog("ReadStatus: WROTE: ");
      roboclaw_.writeN2(false, 2, roboclaw_.portAddress_, RoboClaw::kGETERROR);
      status_ = (unsigned short)roboclaw_.getULongCont2(crc);
      uint16_t responseCrc = 0;
      uint16_t datum = roboclaw_.readByteWithTimeout2();
      responseCrc = datum << 8;
      datum = roboclaw_.readByteWithTimeout2();
      responseCrc |= datum;
      if (responseCrc == crc) {
        roboclaw_.appendToReadLog(", RESULT: %04X", status_);
        return;
      } else {
        RCUTILS_LOG_ERROR(
            "[RoboClaw::CmdReadStatus] invalid CRC expected: 0x%02X, "
            "got: "
            "0x%02X",
            crc, responseCrc);
      }
    } catch (...) {
      RCUTILS_LOG_ERROR("[RoboClaw::CmdReadStatus] Uncaught exception !!!");
    }
  };

  uint16_t &status_;
};
