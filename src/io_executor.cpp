// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2025 Michael Wimble. https://github.com/wimblerobotics/ros2_roboclaw_driver

#include "io_executor.h"

#include <chrono>

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
  worker_ = std::thread(&IoExecutor::run, this);
}

std::future<void> IoExecutor::enqueue(std::function<void()> fn, bool high_priority,
  bool fire_and_forget) {
  Op op{ std::move(fn), std::promise<void>(), std::chrono::steady_clock::now() };
  auto fut = op.promise.get_future();
  {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!running_) start();
    if (high_priority) {
      high_q_.push_back(std::move(op));
    } else {
      normal_q_.push_back(std::move(op));
    }
  }
  cv_.notify_one();
  if (fire_and_forget) {
    return std::move(fut);  // caller may ignore
  }
  return fut;
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

bool IoExecutor::popNext(Op& out) {
  if (!high_q_.empty()) {
    out = std::move(high_q_.front());
    high_q_.pop_front();
    return true;
  }
  if (!normal_q_.empty()) {
    out = std::move(normal_q_.front());
    normal_q_.pop_front();
    return true;
  }
  return false;
}

void IoExecutor::run() {
  while (true) {
    Op op;
    {
      std::unique_lock<std::mutex> lk(mutex_);
      cv_.wait(lk, [this] { return stop_ || !high_q_.empty() || !normal_q_.empty(); });
      if (stop_ && high_q_.empty() && normal_q_.empty()) break;
      if (!popNext(op)) continue;
    }
    // execute outside lock
    try {
      op.fn();
      auto end = std::chrono::steady_clock::now();
      last_latency_ms_ = std::chrono::duration<double, std::milli>(end - op.enqueue_time).count();
      executed_++;
      op.promise.set_value();
    } catch (...) {
      op.promise.set_exception(std::current_exception());
    }
  }
}

IoExecutor::Stats IoExecutor::getStats() const {
  std::lock_guard<std::mutex> lk(mutex_);
  return Stats{ high_q_.size(), normal_q_.size(), executed_, last_latency_ms_ };
}

IoExecutor::~IoExecutor() { shutdown(); }
