#pragma once
#include <cstdint>
#include <cstddef>
#include <string>
#include <chrono>

namespace roboclaw_driver {

  class ITransport {
  public:
    virtual ~ITransport() = default;
    virtual bool write(const uint8_t* data, size_t len, std::string& err) = 0;
    virtual bool read(uint8_t* data, size_t len, double timeout_sec, std::string& err) = 0;
    virtual uint8_t readByte(double timeout_sec, std::string& err) = 0;
    virtual void flush() = 0;
  };

  class SerialTransport : public ITransport {
  public:
    SerialTransport(const std::string& device, int baud_rate);
    ~SerialTransport();
    bool ok() const { return fd_ >= 0; }
    bool write(const uint8_t* data, size_t len, std::string& err) override;
    bool read(uint8_t* data, size_t len, double timeout_sec, std::string& err) override;
    uint8_t readByte(double timeout_sec, std::string& err) override;
    void flush() override;
  private:
    int fd_ = -1;
    std::string device_;
    int baud_ = 0;
    bool openPort(std::string& err);
  };

} // namespace roboclaw_driver
