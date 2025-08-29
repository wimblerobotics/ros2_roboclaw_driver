// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2025 Michael Wimble. https://github.com/wimblerobotics/ros2_roboclaw_driver
#pragma once

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <future>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

// IoExecutor serializes all RoboClaw hardware access (reads & writes)
// onto a single dedicated thread to avoid interleaved transactions
// which previously caused ACK timeouts. Operations can be marked
// high priority (e.g. motion commands) to preempt queued sensor reads.
class IoExecutor {
public:
  static IoExecutor& instance();

  // Starts the worker thread (idempotent).
  void start();

  // Enqueue a hardware operation. If fire_and_forget is true the future is not
  // waited on by caller (still returned if caller wants it). High priority
  // operations are serviced before normal priority ones.
  std::future<void> enqueue(std::function<void()> fn, bool high_priority = false,
    bool fire_and_forget = true);

  // Graceful shutdown (not strictly needed for node lifetime == process).
  void shutdown();

  // Stats (not thread safe; approximate) for debugging / instrumentation.
  struct Stats {
    size_t queued_high{ 0 };
    size_t queued_normal{ 0 };
    uint64_t executed{ 0 };
    double last_latency_ms{ 0.0 };
  };
  Stats getStats() const;

private:
  IoExecutor() = default;
  ~IoExecutor();
  IoExecutor(const IoExecutor&) = delete;
  IoExecutor& operator=(const IoExecutor&) = delete;

  struct Op {
    std::function<void()> fn;
    std::promise<void> promise;
    std::chrono::steady_clock::time_point enqueue_time;
  };

  void run();
  bool popNext(Op& out);

  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<Op> high_q_;
  std::deque<Op> normal_q_;
  bool running_{ false };
  bool stop_{ false };
  std::thread worker_;

  // stats
  uint64_t executed_{ 0 };
  double last_latency_ms_{ 0.0 };
};
