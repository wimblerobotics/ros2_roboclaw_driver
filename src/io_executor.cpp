// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2025 Michael Wimble. https://github.com/wimblerobotics/ros2_roboclaw_driver

#include "io_executor.h"
#include "roboclaw.h"

#include <chrono>
#include <iostream>
#include <thread>
#include <algorithm>

IoExecutor& IoExecutor::instance() {
  static IoExecutor exec;
  exec.start();
  return exec;
}

void IoExecutor::start() {
  std::lock_guard<std::mutex> lk(mutex_);
  if (running_) return;

  stop_ = false;
  running_ = true;
  next_incremental_read_ = 0;
  last_incremental_time_ = std::chrono::steady_clock::now();

  worker_ = std::thread(&IoExecutor::run, this);
}

std::future<void> IoExecutor::enqueue(std::function<void()> fn, Priority priority,
  bool fire_and_forget) {
  Op op{ std::move(fn), std::promise<void>(), priority,
         std::chrono::steady_clock::now(), 0 };
  auto fut = op.promise.get_future();

  {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!running_) start();

    switch (priority) {
    case Priority::HIGH:
      high_queue_.push_back(std::move(op));
      break;
    case Priority::NORMAL:
      normal_queue_.push_back(std::move(op));
      break;
    case Priority::LOW:
      low_queue_.push_back(std::move(op));
      break;
    }
  }

  cv_.notify_one();
  return fut;
}

std::future<void> IoExecutor::enqueueMotorCommand(std::function<void()> cmd) {
  std::lock_guard<std::mutex> lk(mutex_);
  if (!running_) start();

  // Replace any pending high-priority command (latest wins)
  if (!high_queue_.empty()) {
    high_queue_.back().promise.set_exception(
      std::make_exception_ptr(std::runtime_error("Superseded by newer command")));
    high_queue_.clear();
  }

  Op op{ std::move(cmd), std::promise<void>(), Priority::HIGH,
         std::chrono::steady_clock::now(), 0 };
  auto fut = op.promise.get_future();
  high_queue_.push_back(std::move(op));

  cv_.notify_one();
  return fut;
}

void IoExecutor::scheduleIncrementalRead() {
  auto fn = [this]() {
    performIncrementalRead();
    };
  enqueue(std::move(fn), Priority::LOW, true);
}

void IoExecutor::performIncrementalRead() {
  auto now = std::chrono::steady_clock::now();

  // Enforce minimum interval between incremental reads per RoboClaw manual
  if (now - last_incremental_time_ < INCREMENTAL_INTERVAL) {
    return;
  }

  last_incremental_time_ = now;

  // Read ALL sensors in one call since RoboClaw uses global cache
  auto* roboclaw = RoboClaw::singleton();
  if (!roboclaw) return;

  try {
    // Refresh the global sensor cache with fresh hardware reads
    roboclaw->readSensorGroup();

    // Now get all the freshly read values from cache
    auto left_enc = roboclaw->getM1Encoder();
    auto right_enc = roboclaw->getM2Encoder();
    device_cache_.updateEncoders(left_enc, right_enc);

    auto left_vel = roboclaw->getVelocity(RoboClaw::kM1);
    auto right_vel = roboclaw->getVelocity(RoboClaw::kM2);
    device_cache_.updateVelocities(left_vel, right_vel);

    auto currents = roboclaw->getMotorCurrents();
    device_cache_.updateCurrents(currents.m1Current, currents.m2Current);

    auto main_voltage = roboclaw->getMainBatteryLevel();
    auto logic_voltage = roboclaw->getLogicBatteryLevel();
    device_cache_.updateVoltages(main_voltage, logic_voltage);

    auto temp = roboclaw->getTemperature();
    device_cache_.updateTemperatures(temp, temp);

    auto error_status = roboclaw->getErrorStatus();
    device_cache_.updateStatus(0, error_status);

  } catch (const std::exception& e) {
    // Log error but continue with incremental reading
    // The cache will retain previous values if read fails
  }
}

void IoExecutor::shutdown() {
  {
    std::lock_guard<std::mutex> lk(mutex_);
    stop_ = true;
  }
  cv_.notify_all();
  if (worker_.joinable()) worker_.join();
  running_ = false;
}

void IoExecutor::updateStats(const Op& op, bool success) {
  std::lock_guard<std::mutex> lk(stats_mutex_);
  if (success) {
    stats_.operations_completed++;
    auto latency_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - op.enqueue_time).count();

    // Update latency statistics
    stats_.max_latency_ms = std::max(stats_.max_latency_ms, latency_ms);

    // Exponential moving average: α = 0.1
    if (stats_.avg_latency_ms == 0.0) {
      stats_.avg_latency_ms = latency_ms;
    } else {
      stats_.avg_latency_ms = 0.9 * stats_.avg_latency_ms + 0.1 * latency_ms;
    }
  } else {
    stats_.operations_failed++;
  }
}

void IoExecutor::executeWithRetry(Op& op) {
  const int MAX_RETRIES = 3;
  const auto RETRY_DELAY = std::chrono::milliseconds(10);

  while (op.attempt <= MAX_RETRIES) {
    try {
      op.fn();
      updateStats(op, true);
      op.promise.set_value();
      return;

    } catch (const std::exception& e) {
      op.attempt++;

      if (op.attempt > MAX_RETRIES) {
        updateStats(op, false);
        op.promise.set_exception(std::current_exception());
        return;
      }

      // Wait before retry, respecting the 10ms quiet period
      std::this_thread::sleep_for(RETRY_DELAY);
    }
  }
}

void IoExecutor::run() {
  while (true) {
    Op op;
    {
      std::unique_lock<std::mutex> lk(mutex_);
      cv_.wait(lk, [this] {
        return stop_ || !high_queue_.empty() || !normal_queue_.empty() || !low_queue_.empty();
        });

      if (stop_ && high_queue_.empty() && normal_queue_.empty() && low_queue_.empty()) {
        break;
      }

      // Priority order: HIGH -> NORMAL -> LOW
      if (!high_queue_.empty()) {
        op = std::move(high_queue_.front());
        high_queue_.pop_front();
      } else if (!normal_queue_.empty()) {
        op = std::move(normal_queue_.front());
        normal_queue_.pop_front();
      } else if (!low_queue_.empty()) {
        op = std::move(low_queue_.front());
        low_queue_.pop_front();
      } else {
        continue;
      }
    }

    // Execute outside the lock with retry logic
    executeWithRetry(op);
  }
}

IoExecutor::Stats IoExecutor::getStats() const {
  std::lock_guard<std::mutex> lk(stats_mutex_);
  return stats_;
}

IoExecutor::~IoExecutor() {
  shutdown();
}
