#pragma once

#include "roboclaw_cmd.h"

class CmdReadEncoder : public Cmd {
 public:
  CmdReadEncoder(RoboClaw &roboclaw, RoboClaw::kMotor motor,
                 RoboClaw::EncodeResult &encoder)
      : Cmd(roboclaw, "ReadEncoder", motor), encoder_(encoder) {}
  void send() override {
    try {
      roboclaw_.appendToWriteLog("ReadEncoder: encoder: %d (%s), WROTE: ",
                                 motor_, motor_ == RoboClaw::kM1 ? "M1" : "M2");

      uint16_t crc = 0;
      roboclaw_.updateCrc(crc, roboclaw_.portAddress_);
      roboclaw_.updateCrc(crc, motor_ == RoboClaw::kM1
                                   ? RoboClaw::kGETM1ENC
                                   : RoboClaw::RoboClaw::kGETM2ENC);

      roboclaw_.writeN2(
          false, 2, roboclaw_.portAddress_,
          motor_ == RoboClaw::kM1 ? RoboClaw::kGETM1ENC : RoboClaw::kGETM2ENC);

      uint8_t datum = roboclaw_.readByteWithTimeout2();
      encoder_.value |= datum << 24;
      roboclaw_.updateCrc(crc, datum);

      datum = roboclaw_.readByteWithTimeout2();
      encoder_.value |= datum << 16;
      roboclaw_.updateCrc(crc, datum);

      datum = roboclaw_.readByteWithTimeout2();
      encoder_.value |= datum << 8;
      roboclaw_.updateCrc(crc, datum);

      datum = roboclaw_.readByteWithTimeout2();
      encoder_.value |= datum;
      roboclaw_.updateCrc(crc, datum);

      datum = roboclaw_.readByteWithTimeout2();
      encoder_.status |= datum;
      roboclaw_.updateCrc(crc, datum);

      uint16_t responseCrc = 0;
      datum = roboclaw_.readByteWithTimeout2();
      responseCrc = datum << 8;
      datum = roboclaw_.readByteWithTimeout2();
      responseCrc |= datum;
      if (responseCrc != crc) {
        RCUTILS_LOG_ERROR(
            "[RoboClaw::CmdReadEncoder] Expected "
            "CRC of: 0x%02X, but "
            "got: 0x%02X",
            int(crc), int(responseCrc));
        throw new RoboClaw::TRoboClawException(
            "[RoboClaw::CmdReadEncoder] INVALID CRC");
      }

      roboclaw_.appendToReadLog(", RESULT value: %d, status: %d",
                                encoder_.value, encoder_.status);
      return;
    } catch (...) {
      RCUTILS_LOG_ERROR("[RoboClaw::CmdReadEncoder] Uncaught exception !!!");
    }
  }

  RoboClaw::EncodeResult &encoder_;
};
