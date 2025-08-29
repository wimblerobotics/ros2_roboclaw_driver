// MIT License
// Simplified direct RoboClaw device interface (replaces transport/protocol/hardware layers)
#pragma once
#include <cstdint>
#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include "roboclaw_driver/transport.hpp"

namespace roboclaw_driver {

  struct EncoderSnapshot {
    uint32_t value = 0;
    uint8_t status = 0;
    int32_t speed_qpps = 0;
  };

  struct VoltageSnapshot {
    float main = 0.0f;
    float logic = 0.0f;
  };

  struct CurrentSnapshot {
    float m1_inst = 0.0f;
    float m2_inst = 0.0f;
  };

  struct TemperatureSnapshot {
    float t1 = 0.0f;
    float t2 = 0.0f;
  };

  struct PIDSnapshot {
    float p = 0.0f;
    float i = 0.0f;
    float d = 0.0f;
    uint32_t qpps = 0;
  };

  struct Snapshot {
    EncoderSnapshot m1_enc;
    EncoderSnapshot m2_enc;
    VoltageSnapshot volts;
    CurrentSnapshot currents;
    TemperatureSnapshot temps;
    PIDSnapshot m1_pid;
    PIDSnapshot m2_pid;
    uint32_t status_bits = 0;
  };

  class RoboClawDevice {
  public:
    // Existing ctor (creates its own serial transport)
    RoboClawDevice(const std::string& device, int baud_rate, uint8_t address);
    // New ctor for dependency injection (transport already opened/configured)
    RoboClawDevice(std::shared_ptr<ITransport> transport, uint8_t address);
    ~RoboClawDevice();

    bool readSnapshot(Snapshot& out, std::string& err);
    bool driveSpeeds(int32_t m1_qpps, int32_t m2_qpps, std::string& err);
    bool driveSpeedsAccelDistance(int32_t m1_qpps, int32_t m2_qpps, uint32_t accel, uint32_t distance, std::string& err);
    bool setPID(int motor, float p, float i, float d, uint32_t qpps, std::string& err);
    bool readPID(int motor, PIDSnapshot& pid_out, std::string& err);
    bool resetEncoders(std::string& err);
    std::string version();
    // Public helper for harness to sample instantaneous velocity
    bool readMotorVelocity(int motor, int32_t& vel, std::string& err);

    int32_t getLastCommand1() const { return last_cmd_m1_; }
    int32_t getLastCommand2() const { return last_cmd_m2_; }

    void setRetryCount(int c) { retry_count_ = c; }
    // Logging helpers
    std::string lastTx() const { return last_tx_; }
    std::string lastRx() const { return last_rx_; }
    uint8_t lastCommand() const { return last_command_; }

    struct CommandStats { uint32_t attempts = 0; uint32_t crc_fail = 0; uint32_t io_fail = 0; };
    const CommandStats& stats(uint8_t cmd) const { return cmd_stats_[cmd]; }

  private:
    std::shared_ptr<ITransport> transport_; // optional injected transport
    std::string device_;
    int baud_ = 0;
    uint8_t addr_;
    int fd_ = -1; // legacy direct fd path if no injected transport
    int32_t last_cmd_m1_ = 0;
    int32_t last_cmd_m2_ = 0;
    int retry_count_ = 3; // default retries for all transactions
    // Diagnostics
    std::string last_tx_;
    std::string last_rx_;
    uint8_t last_command_ = 0;
    bool last_was_write_ = false;
    CommandStats cmd_stats_[256]{};

    bool openPort(std::string& err);
    void updateCrc(uint16_t& crc, uint8_t byte);
    void flush();
    bool writeBytes(const uint8_t* data, size_t len, std::string& err);
    uint8_t readByte(double timeout_sec, std::string& err);
    bool readBytes(uint8_t* dst, size_t len, double timeout_sec, std::string& err);
    std::mutex io_mutex_;
    bool sendCommandWrite(uint8_t command, const uint8_t* payload, size_t len, std::string& err);
    bool sendCommandRead(uint8_t command, std::string& err);
    bool readU16(uint8_t cmd, uint16_t& val, std::string& err);
    bool readCurrents(uint8_t cmd, int16_t& m1, int16_t& m2, std::string& err);
    bool readU32(uint8_t cmd, uint32_t& val, std::string& err);
    bool readU32WithStatus(uint8_t cmd, uint32_t& value, uint8_t& status, std::string& err);
    bool readVelocity(uint8_t cmd, int32_t& vel, std::string& err);
    bool read4Values(uint8_t cmd, uint32_t& val1, uint32_t& val2, uint32_t& val3, uint32_t& val4, std::string& err);
  };

} // namespace roboclaw_driver
