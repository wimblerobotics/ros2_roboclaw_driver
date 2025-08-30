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
#include "device_cache.h"

// IoExecutor serializes all RoboClaw hardware access (reads & writes)
// onto a single dedicated thread to avoid interleaved transactions
// which previously caused ACK timeouts. Operations can be marked
// high priority (e.g. motion commands) to preempt queued sensor reads.
class IoExecutor {
public:
  enum class Priority {
    HIGH,    // Motor commands, emergency stops
    NORMAL,  // Status reads, encoder reads
    LOW      // Temperatures, voltages, infrequent diagnostics
  };

  struct Stats {
    uint64_t operations_completed = 0;
    uint64_t operations_failed = 0;
    uint64_t total_retries = 0;
    double avg_latency_ms = 0.0;
    double max_latency_ms = 0.0;
  };

  static IoExecutor& instance();

  // Starts the worker thread (idempotent).
  void start();

  // Enqueue a hardware operation with priority.
  std::future<void> enqueue(std::function<void()> fn, Priority priority,
    bool fire_and_forget = true);

  // Enqueue motor command (highest priority, replaces pending).
  std::future<void> enqueueMotorCommand(std::function<void()> cmd);

  // Schedule incremental sensor read if data is stale.
  void scheduleIncrementalRead();

  // Get current operation statistics.
  Stats getStats() const;

  // Get device cache for cached sensor access.
  DeviceCache& getDeviceCache() { return device_cache_; }
  const DeviceCache& getDeviceCache() const { return device_cache_; }

  // Graceful shutdown (not strictly needed for node lifetime == process).
  void shutdown();

private:
  IoExecutor() = default;
  ~IoExecutor();
  IoExecutor(const IoExecutor&) = delete;
  IoExecutor& operator=(const IoExecutor&) = delete;

  struct Op {
    std::function<void()> fn;
    std::promise<void> promise;
    Priority priority;
    std::chrono::steady_clock::time_point enqueue_time;
    int attempt = 0;
  };

  void run();
  void executeWithRetry(Op& op);
  void updateStats(const Op& op, bool success);
  void performIncrementalRead();  // Actually reads one sensor type per call

  std::thread worker_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<Op> high_queue_;     // Motor commands (single slot)
  std::deque<Op> normal_queue_;   // Regular operations
  std::deque<Op> low_queue_;      // Background reads

  bool running_{ false };
  bool stop_{ false };

  // Incremental read state
  int next_incremental_read_ = 0;
  std::chrono::steady_clock::time_point last_incremental_time_;

  // Statistics
  mutable std::mutex stats_mutex_;
  Stats stats_;

  // Device cache
  DeviceCache device_cache_;

  // Timing constraints
  static constexpr int MAX_RETRIES = 3;
  static constexpr std::chrono::milliseconds RETRY_QUIET_TIME{ 10 };
  static constexpr std::chrono::milliseconds INCREMENTAL_INTERVAL{ 20 }; // 50Hz
  static constexpr std::chrono::milliseconds ENCODER_STALENESS{ 50 };    // Force refresh
};
