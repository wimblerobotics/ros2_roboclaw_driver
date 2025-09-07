// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Wimblerobotics
// https://github.com/wimblerobotics/ros2_roboclaw_driver

#pragma once

#include <termios.h>

#include "RoboClaw.h"

class Cmd {
 public:
  void execute() {
    int max_attempts = 3;
    int quiet_ms = 0;
    max_attempts = roboclaw_.getRetryCount();
    quiet_ms = roboclaw_.getRetryQuietMs();

    for (int attempt = 0; attempt < max_attempts; ++attempt) {
      if (attempt == 0) {
      }

      try {
        std::lock_guard<std::mutex> lock(RoboClaw::buffered_command_mutex_);
        send();
        roboclaw_.debug_log_.showLog();
        return;
      } catch (RoboClaw::TRoboClawException* e) {
        roboclaw_.debug_log_.showLog();
        RCUTILS_LOG_ERROR("[RoboClaw::Cmd::execute] Exception: %s, attempt: %d", e->what(), attempt);
      } catch (...) {
        roboclaw_.debug_log_.showLog();
        RCUTILS_LOG_ERROR("[RoboClaw::Cmd::execute] Uncaught exception !!! attempt: %d", attempt);
      }

      if (attempt == max_attempts - 1) {
        RCUTILS_LOG_ERROR("[RoboClaw::Cmd::execute] RETRY COUNT EXCEEDED (%d)", max_attempts);
        throw new RoboClaw::TRoboClawException("[RoboClaw::Cmd::execute] RETRY COUNT EXCEEDED");
      }

      if (quiet_ms > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(quiet_ms));
      }
    }
  }

 protected:
  Cmd(RoboClaw& roboclaw, const char* name, const RoboClaw::kMotor motor) : motor_(motor), roboclaw_(roboclaw) {
    strncpy(name_, name, sizeof(name_));
    name_[sizeof(name_) - 1] = '\0';  // Ensure null-termination
  }

  RoboClaw::kMotor motor_;
  RoboClaw& roboclaw_;
  char name_[32];

 private:
  virtual void send() = 0;

  Cmd() = delete;  // Disallow default constructor
};
