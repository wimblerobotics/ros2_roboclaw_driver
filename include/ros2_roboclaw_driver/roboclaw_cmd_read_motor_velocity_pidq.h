#pragma once

#include "roboclaw_cmd.h"

class CmdReadMotorVelocityPIDQ : public Cmd {
 public:
  CmdReadMotorVelocityPIDQ(RoboClaw &roboclaw, RoboClaw::kMotor motor,
                           RoboClaw::TPIDQ &pidq)
      : Cmd(roboclaw, "ReadMotorVelocityPIDQ", motor), pidq_(pidq) {}
  void send() override {
    try {
      roboclaw_.appendToWriteLog(
          "ReadMotorVelocityPIDQ: motor: %d (%s), WROTE: ", motor_,
          motor_ == RoboClaw::kM1 ? "M1" : "M2");
      RoboClaw::TPIDQ result;
      uint16_t crc = 0;
      roboclaw_.updateCrc(crc, roboclaw_.portAddress_);
      roboclaw_.updateCrc(crc, motor_ == RoboClaw::kM1 ? kGETM1PID : kGETM2PID);

      roboclaw_.writeN2(false, 2, roboclaw_.portAddress_,
                        motor_ == RoboClaw::kM1 ? kGETM1PID : kGETM2PID);
      result.p = (int32_t)roboclaw_.getULongCont2(crc) / 65536.0;
      result.i = (int32_t)roboclaw_.getULongCont2(crc) / 65536.0;
      result.d = (int32_t)roboclaw_.getULongCont2(crc) / 65536.0;
      result.qpps = (int32_t)roboclaw_.getULongCont2(crc);

      uint16_t responseCrc = 0;
      uint16_t datum = roboclaw_.readByteWithTimeout2();
      responseCrc = datum << 8;
      datum = roboclaw_.readByteWithTimeout2();
      responseCrc |= datum;
      roboclaw_.appendToReadLog(
          ", RESULT: p: %3.4f, i: %3.4f, d: %3.4f, qpps: %d", result.p,
          result.i, result.d, result.qpps);
      if (responseCrc == crc) {
        pidq_ = result;
        return;
      } else {
        RCUTILS_LOG_ERROR(
            "[RoboClaw::CmdReadMotorVelocityPIDQ] invalid CRC "
            "expected: 0x%2X, got: "
            "0x%2X",
            crc, responseCrc);
      }
    } catch (...) {
      RCUTILS_LOG_ERROR(
          "[RoboClaw::CmdReadMotorVelocityPIDQ] Uncaught exception !!!");
    }
  }

 private:
  // Referencing which motor in the RoboClaw
  typedef enum WHICH_MOTOR {
    kGETM1PID = 55,
    kGETM2PID = 56,
  } WHICH_MOTOR;

  RoboClaw::TPIDQ &pidq_;
};