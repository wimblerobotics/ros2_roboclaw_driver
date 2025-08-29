#include "roboclaw_driver/transport.hpp"
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <cstring>
#include <cerrno>

namespace roboclaw_driver {

  SerialTransport::SerialTransport(const std::string& device, int baud_rate)
    : device_(device), baud_(baud_rate) {
    std::string err;
    openPort(err);
    (void)err;  // swallow
  }

  SerialTransport::~SerialTransport() {
    if (fd_ >= 0) ::close(fd_);
  }

  bool SerialTransport::openPort(std::string& err) {
    if (fd_ >= 0) return true;
    fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      err = std::string("open failed: ") + strerror(errno);
      return false;
    }
    termios tio{};
    if (tcgetattr(fd_, &tio) != 0) {
      err = "tcgetattr failed";
      return false;
    }
    cfmakeraw(&tio);
    speed_t speed = B38400;
    switch (baud_) {
    case 2400: speed = B2400; break;
    case 4800: speed = B4800; break;
    case 9600: speed = B9600; break;
    case 19200: speed = B19200; break;
    case 38400: speed = B38400; break;
    case 57600: speed = B57600; break;
    case 115200: speed = B115200; break;
    case 230400: speed = B230400; break;
    case 460800: speed = B460800; break;
    case 500000: speed = B500000; break;
    case 576000: speed = B576000; break;
    case 921600: speed = B921600; break;
    case 1000000: speed = B1000000; break;
    case 1152000: speed = B1152000; break;
    case 1500000: speed = B1500000; break;
    case 2000000: speed = B2000000; break;
    default: speed = B38400; break;
    }
    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 5;  // 0.5s
    if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
      err = "tcsetattr failed";
      return false;
    }
    return true;
  }

  void SerialTransport::flush() {
    if (fd_ >= 0) tcflush(fd_, TCIOFLUSH);
  }

  bool SerialTransport::write(const uint8_t* data, size_t len, std::string& err) {
    size_t off = 0;
    while (off < len) {
      ssize_t w = ::write(fd_, data + off, len - off);
      if (w < 0) {
        if (errno == EAGAIN) continue;
        err = strerror(errno);
        return false;
      }
      off += size_t(w);
    }
    return true;
  }

  uint8_t SerialTransport::readByte(double timeout_sec, std::string& err) {
    struct pollfd p { fd_, POLLIN, 0 };
    int to_ms = static_cast<int>(timeout_sec * 1000.0);
    int r = poll(&p, 1, to_ms);
    if (r <= 0) {
      err = r == 0 ? "timeout" : "poll";
      return 0;
    }
    unsigned char b;
    ssize_t n = ::read(fd_, &b, 1);
    if (n != 1) {
      err = "read";
      return 0;
    }
    return b;
  }

  bool SerialTransport::read(uint8_t* dst, size_t len, double timeout_sec, std::string& err) {
    for (size_t i = 0; i < len; ++i) {
      dst[i] = readByte(timeout_sec, err);
      if (!err.empty()) return false;
    }
    return true;
  }

}  // namespace roboclaw_driver
