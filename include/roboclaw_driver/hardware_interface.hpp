// MIT License
// Author: Michael Wimble <mike@wimblerobotics.com>
#pragma once
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "roboclaw_driver/protocol.hpp"
#include "roboclaw_driver/status_decoder.hpp"

namespace roboclaw_driver {

struct MotorPIDConfig {
  float p{0}, i{0}, d{0};
  uint32_t qpps{0};
};
struct EncoderState {
  int64_t value{0};
  uint8_t status{0};
  int32_t speed_qpps{0};
};
struct Currents {
  float m1_inst{0}, m2_inst{0};
};
struct Temperatures {
  float t1{0}, t2{0};
};
struct Voltages {
  float main{0}, logic{0};
};

struct Snapshot {
  MotorPIDConfig m1_pid, m2_pid;
  EncoderState m1_enc, m2_enc;
  Currents currents;
  Voltages volts;
  Temperatures temps;
  uint32_t status_bits{0};
  uint16_t last_crc_errors{0};
  uint16_t last_io_errors{0};
  int32_t m1_command_qpps{0};
  int32_t m2_command_qpps{0};
};

class HardwareInterface {
 public:
  HardwareInterface(Protocol& proto, StatusDecoder& decoder) : proto_(proto), decoder_(decoder) {}
  bool initialize(std::string& err);
  bool setPID(int motor_index, const MotorPIDConfig& cfg, std::string& err);  // motor_index 1 or 2
  bool driveMixedAccelDist(int32_t accel_qpps, int32_t m1_qpps, uint32_t m1_dist, int32_t m2_qpps,
                           uint32_t m2_dist, std::string& err);
  bool driveSpeeds(int32_t m1_qpps, int32_t m2_qpps,
                   std::string& err);  // continuous velocity (placeholder impl)
  bool stop(std::string& err);
  bool resetEncoders(std::string& err);
  std::optional<Snapshot> readSnapshot(std::string& err);
  // Access latest cached snapshot (thread-safe)
  Snapshot latest() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return cache_;
  }
  void setLastCommandSpeeds(int32_t m1, int32_t m2) {
    std::lock_guard<std::mutex> lk(mtx_);
    cache_.m1_command_qpps = m1;
    cache_.m2_command_qpps = m2;
  }

 private:
  Protocol& proto_;
  StatusDecoder& decoder_;
  mutable std::mutex mtx_;
  Snapshot cache_{};
  bool readEncoders(Snapshot& snap, std::string& err);
  bool readCurrents(Snapshot& snap, std::string& err);
  bool readVoltages(Snapshot& snap, std::string& err);
  bool readTemps(Snapshot& snap, std::string& err);
  bool readStatusBits(Snapshot& snap, std::string& err);
};

}  // namespace roboclaw_driver
