// MIT License
#include "roboclaw_driver/roboclaw_device.hpp"
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <cstring>
#include <cerrno>
#include <vector>

namespace roboclaw_driver {

#define MAXRETRY 2

  RoboClawDevice::RoboClawDevice(const std::string& device, int baud_rate, uint8_t address)
    : device_(device), baud_(baud_rate), addr_(address) {
    std::string err;
    openPort(err);
  }

  RoboClawDevice::~RoboClawDevice() {
    if (fd_ >= 0) ::close(fd_);
  }

  bool RoboClawDevice::openPort(std::string& err) {
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
    case 2400:
      speed = B2400;
      break;
    case 4800:
      speed = B4800;
      break;
    case 9600:
      speed = B9600;
      break;
    case 19200:
      speed = B19200;
      break;
    case 38400:
      speed = B38400;
      break;
    case 57600:
      speed = B57600;
      break;
    case 115200:
      speed = B115200;
      break;
    case 230400:
      speed = B230400;
      break;
    case 460800:
      speed = B460800;
      break;
    case 500000:
      speed = B500000;
      break;
    case 576000:
      speed = B576000;
      break;
    case 921600:
      speed = B921600;
      break;
    case 1000000:
      speed = B1000000;
      break;
    case 1152000:
      speed = B1152000;
      break;
    case 1500000:
      speed = B1500000;
      break;
    case 2000000:
      speed = B2000000;
      break;
    default:
      // For USB devices, baud rate is often ignored, but use safe default
      speed = B38400;
      break;
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

  void RoboClawDevice::updateCrc(uint16_t& crc, uint8_t byte) {
    crc ^= (uint16_t)byte << 8;
    for (int i = 0; i < 8; ++i) {
      if (crc & 0x8000)
        crc = (crc << 1) ^ 0x1021;
      else
        crc <<= 1;
    }
  }

  void RoboClawDevice::flush() {
    if (fd_ < 0) return;
    tcflush(fd_, TCIOFLUSH);
  }

  bool RoboClawDevice::writeBytes(const uint8_t* data, size_t len, std::string& err) {
    size_t off = 0;
    while (off < len) {
      ssize_t w = ::write(fd_, data + off, len - off);
      if (w < 0) {
        if (errno == EAGAIN) continue;
        err = strerror(errno);
        return false;
      }
      off += (size_t)w;
    }
    return true;
  }

  uint8_t RoboClawDevice::readByte(double timeout_sec, std::string& err) {
    struct pollfd p { fd_, POLLIN, 0 };
    int to_ms = (int)(timeout_sec * 1000.0);

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
  }  bool RoboClawDevice::readBytes(uint8_t* dst, size_t len, double timeout_sec, std::string& err) {
    for (size_t i = 0; i < len; ++i) {
      dst[i] = readByte(timeout_sec, err);
      if (!err.empty()) return false;
    }
    return true;
  }

  bool RoboClawDevice::commandTxNeedsCRC(uint8_t cmd) const {
    switch (cmd) {
    case 22:
    case 23:
    case 24:
    case 25:
    case 28:
    case 29:
    case 30:
    case 31:
    case 37:
    case 46:
    case 49:
    case 82:
      return true;
    default:
      return false;
    }
  }

  bool RoboClawDevice::commandRxHasCRC(uint8_t cmd) const {
    switch (cmd) {
    case 21:
    case 22:
    case 23:
    case 24:
    case 25:
    case 28:
    case 29:
    case 30:
    case 31:
    case 37:
    case 46:
    case 49:
    case 55:  // ReadM1VelocityPID
    case 56:  // ReadM2VelocityPID
    case 82:
    case 90:
      return true;
    default:
      return false;
    }
  }

  bool RoboClawDevice::sendSimple(uint8_t command, const uint8_t* payload, size_t len,
    std::string& err) {
    std::vector<uint8_t> buf;
    buf.reserve(2 + len);
    buf.push_back(addr_);
    buf.push_back(command);
    for (size_t i = 0; i < len; ++i) buf.push_back(payload[i]);
    return writeBytes(buf.data(), buf.size(), err);
  }

  bool RoboClawDevice::sendCrc(uint8_t command, const uint8_t* payload, size_t len,
    std::string& err) {
    std::vector<uint8_t> buf;
    buf.reserve(2 + len + 2);
    buf.push_back(addr_);
    buf.push_back(command);
    for (size_t i = 0; i < len; ++i) buf.push_back(payload[i]);
    uint16_t crc = 0;
    for (auto b : buf) updateCrc(crc, b);
    buf.push_back((crc >> 8) & 0xFF);
    buf.push_back(crc & 0xFF);
    return writeBytes(buf.data(), buf.size(), err);
  }

  bool RoboClawDevice::readU16(uint8_t cmd, uint16_t& val, std::string& err) {
    if (commandTxNeedsCRC(cmd)) {
      if (!sendCrc(cmd, nullptr, 0, err)) return false;
    } else {
      if (!sendSimple(cmd, nullptr, 0, err)) return false;
    }
    bool rx_crc = commandRxHasCRC(cmd);
    uint8_t data[2];
    if (!readBytes(data, 2, 0.05, err)) return false;
    uint16_t crc = 0;
    if (rx_crc) {
      updateCrc(crc, addr_);
      updateCrc(crc, cmd);
      updateCrc(crc, data[0]);
      updateCrc(crc, data[1]);
      uint8_t cr_hi, cr_lo;
      if (!readBytes(&cr_hi, 1, 0.05, err)) return false;
      if (!readBytes(&cr_lo, 1, 0.05, err)) return false;
      uint16_t rx = (uint16_t(cr_hi) << 8) | cr_lo;
      if (rx != crc) {
        err = "crc mismatch";
        return false;
      }
    }
    val = (uint16_t(data[0]) << 8) | data[1];
    return true;
  }

  bool RoboClawDevice::readCurrents(uint8_t cmd, uint16_t& m1, uint16_t& m2, std::string& err) {
    if (commandTxNeedsCRC(cmd)) {
      if (!sendCrc(cmd, nullptr, 0, err)) return false;
    } else {
      if (!sendSimple(cmd, nullptr, 0, err)) return false;
    }
    bool rx_crc = commandRxHasCRC(cmd);
    uint8_t data[4];
    if (!readBytes(data, 4, 0.05, err)) return false;
    uint16_t crc = 0;
    if (rx_crc) {
      updateCrc(crc, addr_);
      updateCrc(crc, cmd);
      for (int i = 0; i < 4; ++i) updateCrc(crc, data[i]);
      uint8_t hi, lo;
      if (!readBytes(&hi, 1, 0.05, err)) return false;
      if (!readBytes(&lo, 1, 0.05, err)) return false;
      uint16_t rx = (uint16_t(hi) << 8) | lo;
      if (rx != crc) {
        err = "crc mismatch";
        return false;
      }
    }
    m1 = (uint16_t(data[0]) << 8) | data[1];
    m2 = (uint16_t(data[2]) << 8) | data[3];
    return true;
  }

  bool RoboClawDevice::readU32(uint8_t cmd, uint32_t& val, std::string& err) {
    if (commandTxNeedsCRC(cmd)) {
      if (!sendCrc(cmd, nullptr, 0, err)) return false;
    } else {
      if (!sendSimple(cmd, nullptr, 0, err)) return false;
    }
    bool rx_crc = commandRxHasCRC(cmd);
    uint8_t data[4];
    if (!readBytes(data, 4, 0.05, err)) return false;
    uint16_t crc = 0;
    if (rx_crc) {
      updateCrc(crc, addr_);
      updateCrc(crc, cmd);
      for (int i = 0; i < 4; ++i) updateCrc(crc, data[i]);
      uint8_t hi, lo;
      if (!readBytes(&hi, 1, 0.05, err)) return false;
      if (!readBytes(&lo, 1, 0.05, err)) return false;
      uint16_t rx = (uint16_t(hi) << 8) | lo;
      if (rx != crc) {
        err = "crc mismatch";
        return false;
      }
    }
    val = (uint32_t(data[0]) << 24) | (uint32_t(data[1]) << 16) | (uint32_t(data[2]) << 8) | data[3];
    return true;
  }

  bool RoboClawDevice::readU32WithStatus(uint8_t cmd, uint32_t& value, uint8_t& status,
    std::string& err) {
    uint8_t trys = MAXRETRY;

    do {
      flush();

      // Send command
      std::vector<uint8_t> cmd_buf = { addr_, cmd };
      if (!writeBytes(cmd_buf.data(), cmd_buf.size(), err)) {
        continue;
      }

      // Read response - 5 bytes (4 for value + 1 for status)
      uint8_t data[5];
      bool read_ok = true;

      for (int i = 0; i < 5; i++) {
        std::string read_err;
        data[i] = readByte(0.1, read_err);
        if (!read_err.empty()) {
          read_ok = false;
          break;
        }
      }

      if (!read_ok) {
        continue;
      }

      // Read CRC if command has one
      if (commandRxHasCRC(cmd)) {
        uint8_t crc_high, crc_low;
        std::string crc_err;
        crc_high = readByte(0.1, crc_err);
        if (!crc_err.empty()) continue;
        crc_low = readByte(0.1, crc_err);
        if (!crc_err.empty()) continue;

        // Calculate expected CRC
        uint16_t calc_crc = 0;
        updateCrc(calc_crc, addr_);
        updateCrc(calc_crc, cmd);
        for (int i = 0; i < 5; i++) {
          updateCrc(calc_crc, data[i]);
        }

        uint16_t recv_crc = (uint16_t(crc_high) << 8) | crc_low;
        if (calc_crc != recv_crc) {
          continue;  // CRC mismatch, retry
        }
      }

      // Decode value and status
      value = (uint32_t(data[0]) << 24) | (uint32_t(data[1]) << 16) | (uint32_t(data[2]) << 8) | data[3];
      status = data[4];
      return true;

    } while (trys--);

    err = "readU32WithStatus failed after retries";
    return false;
  }

  bool RoboClawDevice::readVelocity(uint8_t cmd, int32_t& vel, std::string& err) {
    if (commandTxNeedsCRC(cmd)) {
      if (!sendCrc(cmd, nullptr, 0, err)) return false;
    } else {
      if (!sendSimple(cmd, nullptr, 0, err)) return false;
    }
    bool rx_crc = commandRxHasCRC(cmd);
    uint8_t data[5];
    if (!readBytes(data, 5, 0.05, err)) return false;
    uint16_t crc = 0;
    if (rx_crc) {
      updateCrc(crc, addr_);
      updateCrc(crc, cmd);
      for (int i = 0; i < 5; ++i) updateCrc(crc, data[i]);
      uint8_t hi, lo;
      if (!readBytes(&hi, 1, 0.05, err)) return false;
      if (!readBytes(&lo, 1, 0.05, err)) return false;
      uint16_t rx = (uint16_t(hi) << 8) | lo;
      if (rx != crc) {
        err = "crc mismatch";
        return false;
      }
    }
    int32_t raw = (int32_t(data[0]) << 24) | (int32_t(data[1]) << 16) | (int32_t(data[2]) << 8) |
      data[3];
    if (data[4] != 0) raw = -raw;
    vel = raw;
    return true;
  }

  bool RoboClawDevice::setPID(int motor, float p, float i, float d, uint32_t qpps, std::string& err) {
    uint8_t cmd = (motor == 1) ? 28 : 29;
    uint32_t P = (uint32_t)(p * 65536.0f);
    uint32_t I = (uint32_t)(i * 65536.0f);
    uint32_t D = (uint32_t)(d * 65536.0f);
    uint8_t payload[16] = { (uint8_t)(P >> 24), (uint8_t)(P >> 16), (uint8_t)(P >> 8), (uint8_t)P,
                           (uint8_t)(I >> 24), (uint8_t)(I >> 16), (uint8_t)(I >> 8), (uint8_t)I,
                           (uint8_t)(D >> 24), (uint8_t)(D >> 16), (uint8_t)(D >> 8), (uint8_t)D,
                           (uint8_t)(qpps >> 24), (uint8_t)(qpps >> 16), (uint8_t)(qpps >> 8),
                           (uint8_t)qpps };
    return commandTxNeedsCRC(cmd) ? sendCrc(cmd, payload, 16, err) : sendSimple(cmd, payload, 16, err);
  }

  bool RoboClawDevice::read4Values(uint8_t cmd, uint32_t& val1, uint32_t& val2, uint32_t& val3, uint32_t& val4, std::string& err) {
    uint8_t trys = MAXRETRY;

    do {
      flush();

      // Send command
      std::vector<uint8_t> cmd_buf = { addr_, cmd };
      if (!writeBytes(cmd_buf.data(), cmd_buf.size(), err)) {
        continue;
      }

      // Read response - 16 bytes for 4 uint32 values
      uint8_t data[16];
      bool read_ok = true;
      int16_t byte_val;

      for (int i = 0; i < 16; i++) {
        std::string read_err;
        byte_val = (int16_t)readByte(0.1, read_err);
        if (!read_err.empty()) {
          read_ok = false;
          break;
        }
        data[i] = (uint8_t)byte_val;
      }

      if (!read_ok) {
        continue;
      }

      // Read CRC
      uint8_t crc_high, crc_low;
      std::string crc_err;
      crc_high = readByte(0.1, crc_err);
      if (!crc_err.empty()) continue;
      crc_low = readByte(0.1, crc_err);
      if (!crc_err.empty()) continue;

      // Calculate expected CRC
      uint16_t calc_crc = 0;
      updateCrc(calc_crc, addr_);
      updateCrc(calc_crc, cmd);
      for (int i = 0; i < 16; i++) {
        updateCrc(calc_crc, data[i]);
      }

      uint16_t recv_crc = (uint16_t(crc_high) << 8) | crc_low;
      if (calc_crc == recv_crc) {
        // Decode the 4 uint32 values
        val1 = (uint32_t(data[0]) << 24) | (uint32_t(data[1]) << 16) | (uint32_t(data[2]) << 8) | data[3];
        val2 = (uint32_t(data[4]) << 24) | (uint32_t(data[5]) << 16) | (uint32_t(data[6]) << 8) | data[7];
        val3 = (uint32_t(data[8]) << 24) | (uint32_t(data[9]) << 16) | (uint32_t(data[10]) << 8) | data[11];
        val4 = (uint32_t(data[12]) << 24) | (uint32_t(data[13]) << 16) | (uint32_t(data[14]) << 8) | data[15];
        return true;
      }

    } while (trys--);

    err = "read4Values failed after retries";
    return false;
  }

  bool RoboClawDevice::readPID(int motor, PIDSnapshot& pid_out, std::string& err) {
    uint8_t cmd = (motor == 1) ? 55 : 56;  // ReadM1VelocityPID : ReadM2VelocityPID
    uint32_t P, I, D, qpps;

    if (!read4Values(cmd, P, I, D, qpps, err)) {
      return false;
    }

    // Convert from fixed point (65536 scale factor) to float
    pid_out.p = static_cast<float>(P) / 65536.0f;
    pid_out.i = static_cast<float>(I) / 65536.0f;
    pid_out.d = static_cast<float>(D) / 65536.0f;
    pid_out.qpps = qpps;

    return true;
  }

  bool RoboClawDevice::driveSpeeds(int32_t m1_qpps, int32_t m2_qpps, std::string& err) {
    uint8_t cmd = 37;
    uint8_t payload[8] = { (uint8_t)(m1_qpps >> 24), (uint8_t)(m1_qpps >> 16), (uint8_t)(m1_qpps >> 8),
                          (uint8_t)m1_qpps,          (uint8_t)(m2_qpps >> 24), (uint8_t)(m2_qpps >> 16),
                          (uint8_t)(m2_qpps >> 8),   (uint8_t)m2_qpps };
    bool ok = commandTxNeedsCRC(cmd) ? sendCrc(cmd, payload, 8, err) : sendSimple(cmd, payload, 8, err);
    if (ok) {
      last_cmd_m1_ = m1_qpps;
      last_cmd_m2_ = m2_qpps;
    }
    return ok;
  }

  bool RoboClawDevice::driveSpeedsAccelDistance(int32_t m1_qpps, int32_t m2_qpps, uint32_t accel, uint32_t distance, std::string& err) {
    uint8_t cmd = 46;  // Drive M1/M2 With Signed Speed, Accel and Distance (Buffered)
    uint8_t payload[16] = {
      (uint8_t)(m1_qpps >> 24), (uint8_t)(m1_qpps >> 16), (uint8_t)(m1_qpps >> 8), (uint8_t)m1_qpps,
      (uint8_t)(m2_qpps >> 24), (uint8_t)(m2_qpps >> 16), (uint8_t)(m2_qpps >> 8), (uint8_t)m2_qpps,
      (uint8_t)(accel >> 24), (uint8_t)(accel >> 16), (uint8_t)(accel >> 8), (uint8_t)accel,
      (uint8_t)(distance >> 24), (uint8_t)(distance >> 16), (uint8_t)(distance >> 8), (uint8_t)distance
    };
    bool ok = commandTxNeedsCRC(cmd) ? sendCrc(cmd, payload, 16, err) : sendSimple(cmd, payload, 16, err);
    if (ok) {
      last_cmd_m1_ = m1_qpps;
      last_cmd_m2_ = m2_qpps;
    }
    return ok;
  }

  bool RoboClawDevice::resetEncoders(std::string& err) {
    uint8_t cmd = 20;
    return sendSimple(cmd, nullptr, 0, err);
  }

  std::string RoboClawDevice::version() {
    std::string err;
    uint8_t cmd = 21;

    if (!sendSimple(cmd, nullptr, 0, err)) {
      return "";
    }

    // Read version string - it should be null-terminated  
    std::vector<uint8_t> accum;
    accum.reserve(64);

    // Read up to 32 bytes looking for null terminator
    for (int i = 0; i < 32; i++) {
      uint8_t ch = 0;
      std::string e2;
      ch = readByte(0.1, e2);
      if (!e2.empty()) {
        return "";
      }

      if (ch == 0) {
        break;
      }
      accum.push_back(ch);
    }

    // read CRC
    uint8_t hi = 0, lo = 0;
    std::string e3;
    hi = readByte(0.1, e3);
    if (!e3.empty()) {
      return "";
    }

    lo = readByte(0.1, e3);
    if (!e3.empty()) {
      return "";
    }

    // Calculate CRC including null terminator
    uint16_t crc_calc = 0;
    updateCrc(crc_calc, addr_);
    updateCrc(crc_calc, cmd);
    for (auto b : accum) updateCrc(crc_calc, b);
    updateCrc(crc_calc, 0);  // Include null terminator in CRC
    uint16_t crc_rx = (uint16_t(hi) << 8) | lo;

    if (crc_calc != crc_rx) {
      return "";
    }

    return std::string(accum.begin(), accum.end());
  }  bool RoboClawDevice::readSnapshot(Snapshot& out, std::string& err) {
    uint32_t v = 0;
    uint8_t status = 0;
    if (!readU32WithStatus(16, v, status, err)) return false;  // GETM1ENC - correct command
    out.m1_enc.value = v;
    out.m1_enc.status = status;
    if (!readU32WithStatus(17, v, status, err)) return false;  // GETM2ENC - correct command
    out.m2_enc.value = v;
    out.m2_enc.status = status;
    int32_t vel = 0;
    if (readVelocity(30, vel, err)) out.m1_enc.speed_qpps = vel;
    if (readVelocity(31, vel, err)) out.m2_enc.speed_qpps = vel;
    uint16_t mv = 0;
    if (readU16(24, mv, err)) out.volts.main = mv / 10.0f;
    uint16_t lv = 0;
    if (readU16(25, lv, err)) out.volts.logic = lv / 10.0f;
    uint16_t m1 = 0, m2 = 0;
    if (readCurrents(49, m1, m2, err)) {
      out.currents.m1_inst = m1 / 100.0f;
      out.currents.m2_inst = m2 / 100.0f;
    }
    uint16_t temp_raw = 0;
    if (readU16(82, temp_raw, err)) out.temps.t1 = temp_raw / 10.0f;
    uint32_t status_bits = 0;
    if (readU32(90, status_bits, err)) out.status_bits = status_bits;

    // Read PID values for both motors
    readPID(1, out.m1_pid, err);  // Don't fail snapshot if PID read fails
    readPID(2, out.m2_pid, err);

    return true;
  }

}  // namespace roboclaw_driver
