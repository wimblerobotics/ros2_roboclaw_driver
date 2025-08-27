// MIT License
// Author: Michael Wimble <mike@wimblerobotics.com>
#pragma once
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "roboclaw_driver/transport.hpp"

namespace roboclaw_driver {

// Low level RoboClaw packet protocol helper.
class Protocol {
 public:
  explicit Protocol(ITransport& transport, uint8_t address)
      : transport_(transport), address_(address) {}

  struct Response {
    std::vector<uint8_t> data;
  };

  // Send a command with payload (may be empty). Reads expected_len bytes response (excludes CRC).
  // Returns std::nullopt on error; sets err.
  std::optional<Response> transact(uint8_t command, const uint8_t* payload, size_t payload_len,
                                   size_t expected_len, int retries, double timeout_sec,
                                   std::string& err);

  static uint16_t crc16(const uint8_t* data, size_t len, uint16_t crc = 0);

  // Temporary public send-only helper (will be wrapped by higher-level helpers)
  bool writePacket(uint8_t command, const uint8_t* payload, size_t payload_len, std::string& err);

 private:
  bool readExact(uint8_t* buf, size_t len, double timeout_sec, std::string& err);
  ITransport& transport_;
  uint8_t address_;
};

}  // namespace roboclaw_driver
