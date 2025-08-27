// MIT License
// Author: Michael Wimble <mike@wimblerobotics.com>
#pragma once
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace roboclaw_driver {

// Abstract transport interface (allows mocking for tests)
class ITransport {
 public:
  virtual ~ITransport() = default;
  virtual bool open(const std::string& device, int baud, std::string& err) = 0;
  virtual void close() = 0;
  virtual bool isOpen() const = 0;
  virtual bool writeBytes(const uint8_t* data, size_t len, std::string& err) = 0;
  // Read exactly len bytes or return false on timeout / error.
  virtual bool readBytes(uint8_t* data, size_t len, double timeout_sec, std::string& err) = 0;
};

class SerialTransport : public ITransport {
 public:
  SerialTransport();
  ~SerialTransport() override;
  bool open(const std::string& device, int baud, std::string& err) override;
  void close() override;
  bool isOpen() const override;
  bool writeBytes(const uint8_t* data, size_t len, std::string& err) override;
  bool readBytes(uint8_t* data, size_t len, double timeout_sec, std::string& err) override;

 private:
  int fd_{-1};
  std::string device_;
  int baud_{0};
  mutable std::mutex mtx_;
  bool configureTTY(int fd, int baud, std::string& err);
};

}  // namespace roboclaw_driver
