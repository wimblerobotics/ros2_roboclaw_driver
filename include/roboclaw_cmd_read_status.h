#pragma once

#include "roboclaw_cmd.h"

class CmdReadStatus : public Cmd {
 public:
  CmdReadStatus(RoboClaw &roboclaw, uint32_t &status)
      : Cmd(roboclaw, "ReadStatus", RoboClaw::kNone), status_(status) {}
  void send() override {
    try {
      uint16_t crc = 0;
      roboclaw_.updateCrc(crc, roboclaw_.portAddress_);
      roboclaw_.updateCrc(crc, RoboClaw::GETERROR);
      roboclaw_.appendToWriteLog("ReadStatus: WROTE: ");
      roboclaw_.writeN2(false, 2, roboclaw_.portAddress_, RoboClaw::GETERROR);
      // Read full 32-bit status (4 bytes) updating CRC on each byte.
      status_ = roboclaw_.getULongCont2(crc);
      // Read and validate 16-bit CRC
      uint16_t responseCrc = 0;
      uint8_t datum = roboclaw_.readByteWithTimeout2();
      responseCrc = static_cast<uint16_t>(datum) << 8;
      datum = roboclaw_.readByteWithTimeout2();
      responseCrc |= datum;
      if (responseCrc == crc) {
        roboclaw_.appendToReadLog(", RESULT: %08X", static_cast<unsigned int>(status_));
        return;
      } else {
        RCUTILS_LOG_ERROR(
            "[RoboClaw::CmdReadStatus] invalid CRC expected: 0x%04X, got: 0x%04X",
            crc, responseCrc);
      }
    } catch (...) {
      RCUTILS_LOG_ERROR("[RoboClaw::CmdReadStatus] Uncaught exception !!!");
    }
  };

 private:
  uint32_t &status_;
};
