#pragma once

#include "roboclaw_cmd.h"

class CmdReadTemperature : public Cmd {
 public:
  CmdReadTemperature(RoboClaw &roboclaw, float &temperature)
      : Cmd(roboclaw, "ReadTemperature", RoboClaw::kNone),
        temperature_(temperature) {}
  void send() override {
    try {
      roboclaw_.appendToWriteLog("ReadTemperature: WROTE: ");
      uint16_t crc = 0;
      roboclaw_.updateCrc(crc, roboclaw_.portAddress_);
      roboclaw_.updateCrc(crc, RoboClaw::GETTEMPERATURE);
      roboclaw_.writeN2(false, 2, roboclaw_.portAddress_,
                        RoboClaw::GETTEMPERATURE);
      uint16_t result = 0;
      uint8_t datum = roboclaw_.readByteWithTimeout2();
      roboclaw_.updateCrc(crc, datum);
      result = datum << 8;
      datum = roboclaw_.readByteWithTimeout2();
      roboclaw_.updateCrc(crc, datum);
      result |= datum;

      uint16_t responseCrc = 0;
      datum = roboclaw_.readByteWithTimeout2();
      responseCrc = datum << 8;
      datum = roboclaw_.readByteWithTimeout2();
      responseCrc |= datum;
      if (responseCrc != crc) {
        RCUTILS_LOG_ERROR(
            "[RoboClaw::CmdReadTemperature] invalid CRC expected: 0x%2X, "
            "got: 0x%2X",
            crc, responseCrc);
        result = 0.0;
      }

      temperature_ = result / 10.0;
    } catch (...) {
      RCUTILS_LOG_ERROR(
          "[RoboClaw::CmdReadTemperature] Uncaught exception !!!");
    }

    roboclaw_.appendToReadLog(", RESULT: %f", temperature_);
  }

 private:
  float &temperature_;
};
