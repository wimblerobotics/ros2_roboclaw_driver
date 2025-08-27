// MIT License
// Author: Michael Wimble <mike@wimblerobotics.com>
#include "roboclaw_driver/protocol.hpp"

#include <cstring>
namespace roboclaw_driver {
uint16_t Protocol::crc16(const uint8_t* data, size_t len, uint16_t crc) {
  for (size_t i = 0; i < len; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (int j = 0; j < 8; ++j) {
      if (crc & 0x8000)
        crc = (crc << 1) ^ 0x1021;
      else
        crc <<= 1;
    }
  }
  return crc;
}

bool Protocol::writePacket(uint8_t command, const uint8_t* payload, size_t payload_len,
                           std::string& err) {
  // Packet: address, command, payload..., crc(high), crc(low)
  std::vector<uint8_t> buf;
  buf.reserve(2 + payload_len + 2);
  buf.push_back(address_);
  buf.push_back(command);
  for (size_t i = 0; i < payload_len; ++i) buf.push_back(payload[i]);
  uint16_t crc = crc16(buf.data(), buf.size());
  buf.push_back((crc >> 8) & 0xFF);
  buf.push_back(crc & 0xFF);
  return transport_.writeBytes(buf.data(), buf.size(), err);
}

bool Protocol::readExact(uint8_t* buf, size_t len, double timeout_sec, std::string& err) {
  return transport_.readBytes(buf, len, timeout_sec, err);
}

std::optional<Protocol::Response> Protocol::transact(uint8_t command, const uint8_t* payload,
                                                     size_t payload_len, size_t expected_len,
                                                     int retries, double timeout_sec,
                                                     std::string& err) {
  for (int attempt = 0; attempt < retries; ++attempt) {
    if (!writePacket(command, payload, payload_len, err)) continue;
    // read response (expected_len bytes + 2 crc)
    std::vector<uint8_t> resp(expected_len + 2);
    if (!readExact(resp.data(), resp.size(), timeout_sec, err)) continue;
    uint16_t crc_calc = crc16(resp.data(), expected_len);
    uint16_t crc_rx = ((uint16_t)resp[expected_len] << 8) | resp[expected_len + 1];
    if (crc_calc != crc_rx) {
      err = "crc mismatch";
      continue;
    }
    Response r;
    r.data.assign(resp.begin(), resp.begin() + expected_len);
    return r;
  }
  return std::nullopt;
}

}  // namespace roboclaw_driver
