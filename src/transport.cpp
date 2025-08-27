// MIT License
// Author: Michael Wimble <mike@wimblerobotics.com>
#include "roboclaw_driver/transport.hpp"

#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cstring>

namespace roboclaw_driver {
SerialTransport::SerialTransport() = default;
SerialTransport::~SerialTransport() {
  close();
}

bool SerialTransport::open(const std::string& device, int baud, std::string& err) {
  std::lock_guard<std::mutex> lk(mtx_);
  if (fd_ >= 0) {
    err = "already open";
    return false;
  }
  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    err = std::string("open failed: ") + std::strerror(errno);
    return false;
  }
  device_ = device;
  baud_ = baud;
  if (!configureTTY(fd_, baud_, err)) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }
  return true;
}

void SerialTransport::close() {
  std::lock_guard<std::mutex> lk(mtx_);
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialTransport::isOpen() const {
  std::lock_guard<std::mutex> lk(mtx_);
  return fd_ >= 0;
}

static speed_t mapBaud(int baud) {
  switch (baud) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
#ifdef B230400
    case 230400:
      return B230400;
#endif
#ifdef B460800
    case 460800:
      return B460800;
#endif
#ifdef B921600
    case 921600:
      return B921600;
#endif
    default:
      return B38400;
  }
}

bool SerialTransport::configureTTY(int fd, int baud, std::string& err) {
  struct termios tio{};
  if (tcgetattr(fd, &tio) != 0) {
    err = std::string("tcgetattr: ") + std::strerror(errno);
    return false;
  }
  cfmakeraw(&tio);
  speed_t sp = mapBaud(baud);
  cfsetispeed(&tio, sp);
  cfsetospeed(&tio, sp);
  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cflag &= ~CRTSCTS;  // no hw flow
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 1;  // 0.1s units
  if (tcsetattr(fd, TCSANOW, &tio) != 0) {
    err = std::string("tcsetattr: ") + std::strerror(errno);
    return false;
  }
  return true;
}

bool SerialTransport::writeBytes(const uint8_t* data, size_t len, std::string& err) {
  std::lock_guard<std::mutex> lk(mtx_);
  if (fd_ < 0) {
    err = "not open";
    return false;
  }
  size_t written = 0;
  while (written < len) {
    ssize_t rc = ::write(fd_, data + written, len - written);
    if (rc < 0) {
      if (errno == EINTR) continue;
      err = std::string("write: ") + std::strerror(errno);
      return false;
    }
    written += static_cast<size_t>(rc);
  }
  return true;
}

bool SerialTransport::readBytes(uint8_t* data, size_t len, double timeout_sec, std::string& err) {
  std::lock_guard<std::mutex> lk(mtx_);
  if (fd_ < 0) {
    err = "not open";
    return false;
  }
  size_t got = 0;
  auto start = std::chrono::steady_clock::now();
  while (got < len) {
    ssize_t rc = ::read(fd_, data + got, len - got);
    if (rc < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - start).count();
        if (elapsed > timeout_sec) {
          err = "timeout";
          return false;
        }
        // brief sleep
        ::usleep(1000);
        continue;
      } else if (errno == EINTR) {
        continue;
      } else {
        err = std::string("read: ") + std::strerror(errno);
        return false;
      }
    } else if (rc == 0) {
      auto now = std::chrono::steady_clock::now();
      double elapsed = std::chrono::duration<double>(now - start).count();
      if (elapsed > timeout_sec) {
        err = "timeout";
        return false;
      }
      ::usleep(1000);
      continue;
    } else {
      got += static_cast<size_t>(rc);
    }
  }
  return true;
}

}  // namespace roboclaw_driver
