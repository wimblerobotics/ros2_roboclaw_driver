#pragma once

#include "roboclaw_cmd.h"

class CmdReadFirmwareVersion : public Cmd {
 public:
  CmdReadFirmwareVersion(RoboClaw &roboclaw, std::string &version)
      : Cmd(roboclaw, "ReadFirmwareVersion", RoboClaw::kNone),
        version_(version) {}

  void send() override {
    roboclaw_.appendToWriteLog("ReadFirmwareVersion: WROTE: ");
    roboclaw_.writeN2(false, 2, roboclaw_.portAddress_, RoboClaw::GETVERSION);

    try {
      uint16_t crc = 0;
      uint8_t i;
      uint8_t datum;
      std::stringstream version;

      roboclaw_.updateCrc(crc, roboclaw_.portAddress_);
      roboclaw_.updateCrc(crc, RoboClaw::GETVERSION);
      for (i = 0; i < 48; i++) {
        datum = roboclaw_.readByteWithTimeout2();
        roboclaw_.updateCrc(crc, datum);
        if (datum == 0) {
          uint16_t responseCrc = 0;
          datum = roboclaw_.readByteWithTimeout2();
          responseCrc = datum << 8;
          datum = roboclaw_.readByteWithTimeout2();
          responseCrc |= datum;
          if (responseCrc == crc) {
            version_ = version.str();
            roboclaw_.appendToReadLog(", RESULT: '%s'", version_.c_str());
            return;
          } else {
            RCUTILS_LOG_ERROR(
                "[RoboClaw::CmdReadFirmwareVersion] invalid "
                "CRC expected: 0x%02X, "
                "got: 0x%02X",
                crc, responseCrc);
            throw new RoboClaw::TRoboClawException(
                "[RoboClaw::getVersion] Invalid CRC, expected 0x%02X, "
                "got 0x%02X",
                crc, responseCrc);
          }
        } else {
          version << (char)datum;
        }
      }

      RCUTILS_LOG_ERROR(
          "[RoboClaw::CmdReadFirmwareVersion] unexpected long string");
      throw new RoboClaw::TRoboClawException(
          "[RoboClaw::getVersion] unexpected long string");
    } catch (...) {
      RCUTILS_LOG_ERROR(
          "[RoboClaw::CmdReadFirmwareVersion] Uncaught exception !!!");
    }
    RCUTILS_LOG_ERROR(
        "[RoboClaw::CmdReadFirmwareVersion] RETRY COUNT EXCEEDED");
  }

 private:
  std::string &version_;
};
