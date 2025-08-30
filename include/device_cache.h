// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2025 Michael Wimble. https://github.com/wimblerobotics/ros2_roboclaw_driver

#pragma once

#include <chrono>
#include <mutex>

/**
 * @brief Thread-safe cache for RoboClaw device sensor data.
 *
 * Provides atomic access to frequently-read sensor values with timestamps
 * to support incremental acquisition and staleness detection.
 */
class DeviceCache {
public:
  struct SensorData {
    // Encoder values (signed for direction detection)
    int32_t enc_left = 0;
    int32_t enc_right = 0;

    // Velocities in quad pulses per second
    int32_t vel_left_qpps = 0;
    int32_t vel_right_qpps = 0;

    // Motor currents in amps
    float current_m1 = 0.0f;
    float current_m2 = 0.0f;

    // Battery voltages
    float volt_main = 0.0f;
    float volt_logic = 0.0f;

    // Temperatures in celsius
    float temp1 = 0.0f;
    float temp2 = 0.0f;

    // Status and error bits
    uint32_t status_bits = 0;
    uint32_t error_status = 0;

    // PID values (cached after initial set, rarely updated)
    float m1_p = 0.0f, m1_i = 0.0f, m1_d = 0.0f;
    float m2_p = 0.0f, m2_i = 0.0f, m2_d = 0.0f;
  };

  enum class DataCategory {
    ENCODERS = 0,
    VELOCITIES = 1,
    CURRENTS = 2,
    VOLTAGES = 3,
    TEMPERATURES = 4,
    STATUS_BITS = 5,
    PID_VALUES = 6,
    COUNT = 7
  };

  DeviceCache() {
    auto now = std::chrono::steady_clock::now();
    for (int i = 0; i < static_cast<int>(DataCategory::COUNT); ++i) {
      timestamps_[i] = now;
    }
  }

  /**
   * @brief Get snapshot of all sensor data with thread safety.
   */
  SensorData getSnapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return data_;
  }

  /**
   * @brief Get encoder values.
   */
  std::pair<int32_t, int32_t> getEncoders() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return { data_.enc_left, data_.enc_right };
  }

  /**
   * @brief Get velocity values.
   */
  std::pair<int32_t, int32_t> getVelocities() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return { data_.vel_left_qpps, data_.vel_right_qpps };
  }

  /**
   * @brief Get current values.
   */
  std::pair<float, float> getCurrents() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return { data_.current_m1, data_.current_m2 };
  }

  /**
   * @brief Get main battery voltage.
   */
  float getMainBatteryVoltage() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return data_.volt_main;
  }

  /**
   * @brief Get logic battery voltage.
   */
  float getLogicBatteryVoltage() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return data_.volt_logic;
  }

  /**
   * @brief Get temperature values.
   */
  std::pair<float, float> getTemperatures() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return { data_.temp1, data_.temp2 };
  }

  /**
   * @brief Get status information.
   */
  uint32_t getStatus() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return data_.error_status;
  }

  /**
   * @brief Update encoder values atomically.
   */
  void updateEncoders(int32_t left, int32_t right) {
    std::lock_guard<std::mutex> lock(mutex_);
    data_.enc_left = left;
    data_.enc_right = right;
    timestamps_[static_cast<int>(DataCategory::ENCODERS)] =
      std::chrono::steady_clock::now();
  }

  /**
   * @brief Update velocity values atomically.
   */
  void updateVelocities(int32_t left_qpps, int32_t right_qpps) {
    std::lock_guard<std::mutex> lock(mutex_);
    data_.vel_left_qpps = left_qpps;
    data_.vel_right_qpps = right_qpps;
    timestamps_[static_cast<int>(DataCategory::VELOCITIES)] =
      std::chrono::steady_clock::now();
  }

  /**
   * @brief Update current values atomically.
   */
  void updateCurrents(float m1_amps, float m2_amps) {
    std::lock_guard<std::mutex> lock(mutex_);
    data_.current_m1 = m1_amps;
    data_.current_m2 = m2_amps;
    timestamps_[static_cast<int>(DataCategory::CURRENTS)] =
      std::chrono::steady_clock::now();
  }

  /**
   * @brief Update voltage values atomically.
   */
  void updateVoltages(float main_volts, float logic_volts) {
    std::lock_guard<std::mutex> lock(mutex_);
    data_.volt_main = main_volts;
    data_.volt_logic = logic_volts;
    timestamps_[static_cast<int>(DataCategory::VOLTAGES)] =
      std::chrono::steady_clock::now();
  }

  /**
   * @brief Update temperature values atomically.
   */
  void updateTemperatures(float temp1_c, float temp2_c) {
    std::lock_guard<std::mutex> lock(mutex_);
    data_.temp1 = temp1_c;
    data_.temp2 = temp2_c;
    timestamps_[static_cast<int>(DataCategory::TEMPERATURES)] =
      std::chrono::steady_clock::now();
  }

  /**
   * @brief Update status bits atomically.
   */
  void updateStatus(uint32_t status, uint32_t error) {
    std::lock_guard<std::mutex> lock(mutex_);
    data_.status_bits = status;
    data_.error_status = error;
    timestamps_[static_cast<int>(DataCategory::STATUS_BITS)] =
      std::chrono::steady_clock::now();
  }

  /**
   * @brief Update PID values (rare operation).
   */
  void updatePid(float m1_p, float m1_i, float m1_d,
    float m2_p, float m2_i, float m2_d) {
    std::lock_guard<std::mutex> lock(mutex_);
    data_.m1_p = m1_p; data_.m1_i = m1_i; data_.m1_d = m1_d;
    data_.m2_p = m2_p; data_.m2_i = m2_i; data_.m2_d = m2_d;
    timestamps_[static_cast<int>(DataCategory::PID_VALUES)] =
      std::chrono::steady_clock::now();
  }

  /**
   * @brief Check if data category is stale beyond threshold.
   */
  bool isStale(DataCategory category,
    std::chrono::milliseconds threshold) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto now = std::chrono::steady_clock::now();
    auto age = now - timestamps_[static_cast<int>(category)];
    return age > threshold;
  }

  /**
   * @brief Get age of specific data category.
   */
  std::chrono::milliseconds getAge(DataCategory category) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(
      now - timestamps_[static_cast<int>(category)]);
  }

private:
  mutable std::mutex mutex_;
  SensorData data_;
  std::chrono::steady_clock::time_point
    timestamps_[static_cast<int>(DataCategory::COUNT)];
};
