#pragma once

#include "roboclaw.h"

class Cmd {
 public:
  void execute() {
    for (int retry = 0; retry < 3 /*### maxCommandRetries_*/; retry++) {
      try {
        std::lock_guard<std::mutex> lock(
            RoboClaw::buffered_command_mutex_);  // Lock the mutex
        send();
        roboclaw_.debug_log_.showLog();
        return;
      } catch (RoboClaw::TRoboClawException *e) {
        roboclaw_.debug_log_.showLog();
        RCUTILS_LOG_ERROR(
            "[RoboClaw::Cmd::execute] Exception: %s, retry number: %d",
            e->what(), retry);
      } catch (...) {
        roboclaw_.debug_log_.showLog();
        RCUTILS_LOG_ERROR("[RoboClaw::Cmd::execute] Uncaught exception !!!");
      }
    }

    roboclaw_.debug_log_.showLog();
    RCUTILS_LOG_ERROR("[RoboClaw::Cmd::execute] RETRY COUNT EXCEEDED");
    throw new RoboClaw::TRoboClawException(
        "[RoboClaw::Cmd::execute] RETRY COUNT EXCEEDED");
  }

  virtual void send() = 0;  // Declare send as a pure virtual function

 protected:
  Cmd(RoboClaw &roboclaw, const char *name, const RoboClaw::kMotor motor)
      : motor_(motor), roboclaw_(roboclaw) {
    strncpy(name_, name, sizeof(name_));
    name_[sizeof(name_) - 1] = '\0';  // Ensure null-termination
  }

  RoboClaw::kMotor motor_;
  RoboClaw &roboclaw_;
  char name_[32];

 private:
  Cmd() = delete;  // Disallow default constructor
};
