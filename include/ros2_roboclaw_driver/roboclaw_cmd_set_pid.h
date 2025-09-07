#pragma once

#include "roboclaw_cmd.h"

class CmdSetPid : public Cmd {
 public:
  CmdSetPid(RoboClaw &roboclaw, RoboClaw::kMotor motor, float p, float i,
            float d, uint32_t qpps)
      : Cmd(roboclaw, "SetPid", motor), p_(p), i_(i), d_(d), qpps_(qpps) {}

  void send() override {
    roboclaw_.appendToWriteLog(
        "SetPid: motor: %d (%s) p: %f, i: %f, d: %f, "
        "qpps: %d, WROTE: ",
        motor_, RoboClaw::motorNames_[motor_], p_, i_, d_, qpps_);
    uint32_t kp = int(p_ * 65536.0);
    uint32_t ki = int(i_ * 65536.0);
    uint32_t kd = int(d_ * 65536.0);
    roboclaw_.writeN2(
        true, 18, roboclaw_.portAddress_,
        motor_ == RoboClaw::kM1 ? RoboClaw::SETM1PID : RoboClaw::SETM2PID,
        SetDWORDval(kd), SetDWORDval(kp), SetDWORDval(ki), SetDWORDval(qpps_));
  }

 private:
  float p_;
  float i_;
  float d_;
  uint32_t qpps_;
};
