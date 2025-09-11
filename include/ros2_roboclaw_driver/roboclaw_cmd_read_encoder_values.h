// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Wimblerobotics
// https://github.com/wimblerobotics/ros2_roboclaw_driver

#pragma once

#include "roboclaw_cmd.h"

class CmdReadEncoderValues : public Cmd {
 public:
  CmdReadEncoderValues(RoboClaw& roboclaw, uint32_t& m1_encoder, uint32_t& m2_encoder)
      : Cmd(roboclaw, "ReadEncoderValues", RoboClaw::kNone), m1_encoder_(m1_encoder), m2_encoder_(m2_encoder) {}

 private:
  void send() override {
    try {
      roboclaw_.appendToWriteLog("ReadEncoderValues), WROTE: ");

      uint16_t crc = 0;
      roboclaw_.updateCrc(crc, roboclaw_.portAddress_);
      roboclaw_.updateCrc(crc, RoboClaw::GETENCODERVALUES);

      roboclaw_.writeN2(false, 2, roboclaw_.portAddress_, RoboClaw::GETENCODERVALUES);

      m1_encoder_ = 0;
      m2_encoder_ = 0;

      uint8_t datum = roboclaw_.readByteWithTimeout2();
      m1_encoder_ |= datum << 24;
      roboclaw_.updateCrc(crc, datum);

      datum = roboclaw_.readByteWithTimeout2();
      m1_encoder_ |= datum << 16;
      roboclaw_.updateCrc(crc, datum);

      datum = roboclaw_.readByteWithTimeout2();
      m1_encoder_ |= datum << 8;
      roboclaw_.updateCrc(crc, datum);

      datum = roboclaw_.readByteWithTimeout2();
      m1_encoder_ |= datum;
      roboclaw_.updateCrc(crc, datum);

      datum = roboclaw_.readByteWithTimeout2();
      m2_encoder_ |= datum << 24;
      roboclaw_.updateCrc(crc, datum);

      datum = roboclaw_.readByteWithTimeout2();
      m2_encoder_ |= datum << 16;
      roboclaw_.updateCrc(crc, datum);

      datum = roboclaw_.readByteWithTimeout2();
      m2_encoder_ |= datum << 8;
      roboclaw_.updateCrc(crc, datum);

      datum = roboclaw_.readByteWithTimeout2();
      m2_encoder_ |= datum;
      roboclaw_.updateCrc(crc, datum);

      uint16_t responseCrc = 0;
      datum = roboclaw_.readByteWithTimeout2();
      responseCrc = datum << 8;
      datum = roboclaw_.readByteWithTimeout2();
      responseCrc |= datum;
      if (responseCrc != crc) {
        RCUTILS_LOG_ERROR(
            "[RoboClaw::CmdReadEncoderValues] Expected "
            "CRC of: 0x%02X, but "
            "got: 0x%02X",
            int(crc), int(responseCrc));
        throw new RoboClaw::TRoboClawException("[RoboClaw::CmdReadEncoderValues] INVALID CRC");
      }

      roboclaw_.appendToReadLog(", RESULT m1_encoder: %d, m2_encoder: %d", m1_encoder_, m2_encoder_);
      return;
    } catch (...) {
      RCUTILS_LOG_ERROR("[RoboClaw::CmdReadEncoderValues] Uncaught exception !!!");
    }
  }

  uint32_t& m1_encoder_;
  uint32_t& m2_encoder_;
};
