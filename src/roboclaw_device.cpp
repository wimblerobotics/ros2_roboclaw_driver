// MIT License
#include "roboclaw_driver/roboclaw_device.hpp"
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <cstring>
#include <cerrno>
#include <vector>
#include <stdexcept>
#include <iostream>  // For debug output

namespace roboclaw_driver {

  RoboClawDevice::RoboClawDevice(const std::string& device, int baud_rate, uint8_t address)
    : device_(device), baud_(baud_rate), addr_(address) {
    std::string err;

    // Open and configure serial port
    if (!openPort(err)) {
      throw std::runtime_error("Failed to open RoboClaw device '" + device + "': " + err);
    }

    // Verify device communication by reading firmware version
    std::string version_str = version();
    if (version_str.empty()) {
      ::close(fd_);
      fd_ = -1;
      throw std::runtime_error("RoboClaw device at address " + std::to_string(address) +
        " failed to respond to firmware version request. " +
        "Check device address and baud rate (" + std::to_string(baud_rate) + ").");
    }

    initialized_ = true;
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

    speed_t speed = B38400;  // Default fallback
    switch (baud_) {
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
    default:
      err = "Unsupported baud rate: " + std::to_string(baud_);
      ::close(fd_);
      fd_ = -1;
      return false;
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
      if (r == 0) {
        err = "timeout after " + std::to_string(timeout_sec) + "s";
          } else {
        err = "poll error: " + std::string(strerror(errno));
      }
      return 0;
    }

    unsigned char b;
    ssize_t n = ::read(fd_, &b, 1);
    if (n != 1) {
      err = "read error: " + std::string(strerror(errno));
      return 0;
    }

    return b;
  }

  bool RoboClawDevice::readBytes(uint8_t* dst, size_t len, double timeout_sec, std::string& err) {
    for (size_t i = 0; i < len; ++i) {
      dst[i] = readByte(timeout_sec, err);
      if (!err.empty()) return false;
    }
    return true;
  }

  bool RoboClawDevice::commandTxNeedsCRC(uint8_t cmd) const {
    // Based on reference Arduino implementation analysis:
    // Commands that use write_n() need CRC and expect 0xFF acknowledgment
    switch (cmd) {
    case 0:   // M1FORWARD
    case 1:   // M1BACKWARD  
    case 2:   // SETMINMB
    case 3:   // SETMAXMB
    case 4:   // M2FORWARD
    case 5:   // M2BACKWARD
    case 6:   // M17BIT
    case 7:   // M27BIT
    case 8:   // MIXEDFORWARD
    case 9:   // MIXEDBACKWARD
    case 10:  // MIXEDRIGHT
    case 11:  // MIXEDLEFT
    case 12:  // MIXEDFB
    case 13:  // MIXEDLR
    case 20:  // RESETENC - uses write_n(2, ...)
    case 22:  // SETM1ENCCOUNT - uses write_n(6, ...) - WRITE operation
    case 23:  // SETM2ENCCOUNT - uses write_n(6, ...) - WRITE operation
    case 26:  // SETMINLB
    case 27:  // SETMAXLB
    case 28:  // SETM1PID - uses write_n(18, ...)
    case 29:  // SETM2PID - uses write_n(18, ...)
    case 32:  // M1DUTY
    case 33:  // M2DUTY
    case 34:  // MIXEDDUTY
    case 35:  // M1SPEED
    case 36:  // M2SPEED
    case 37:  // MIXEDSPEED - uses write_n(10, ...)
    case 38:  // M1SPEEDACCEL
    case 39:  // M2SPEEDACCEL
    case 40:  // MIXEDSPEEDACCEL
    case 41:  // M1SPEEDDIST
    case 42:  // M2SPEEDDIST
    case 43:  // MIXEDSPEEDDIST
    case 44:  // M1SPEEDACCELDIST
    case 45:  // M2SPEEDACCELDIST
    case 46:  // MIXEDSPEEDACCELDIST - uses write_n(23, ...)
    case 50:  // MIXEDSPEED2ACCEL
    case 51:  // MIXEDSPEED2ACCELDIST
    case 52:  // M1DUTYACCEL
    case 53:  // M2DUTYACCEL
    case 54:  // MIXEDDUTYACCEL
    case 57:  // SETMAINVOLTAGES
    case 58:  // SETLOGICVOLTAGES
    case 61:  // SETM1POSPID
    case 62:  // SETM2POSPID
    case 65:  // M1SPEEDACCELDECCELPOS
    case 66:  // M2SPEEDACCELDECCELPOS
    case 67:  // MIXEDSPEEDACCELDECCELPOS
    case 68:  // SETM1DEFAULTACCEL
    case 69:  // SETM2DEFAULTACCEL
    case 74:  // SETPINFUNCTIONS
    case 76:  // SETDEADBAND
    case 80:  // RESTOREDEFAULTS
    case 92:  // SETM1ENCODERMODE
    case 93:  // SETM2ENCODERMODE
    case 94:  // WRITENVM
    case 95:  // READNVM
    case 98:  // SETCONFIG
    case 133: // SETM1MAXCURRENT
    case 134: // SETM2MAXCURRENT
    case 148: // SETPWMMODE
      return true;
    default:
      return false;
    }
  }

  bool RoboClawDevice::commandRxHasCRC(uint8_t cmd) const {
    // Based on reference Arduino implementation analysis:
    // Commands that use Read1(), Read2(), Read4(), Read4_1() have CRC in response
    switch (cmd) {
    case 16:  // GETM1ENC - uses Read4_1()
    case 17:  // GETM2ENC - uses Read4_1()
    case 18:  // GETM1SPEED - uses Read4_1()
    case 19:  // GETM2SPEED - uses Read4_1()
    case 21:  // GETVERSION - special case with CRC
    case 24:  // GETMBATT - uses Read2()
    case 25:  // GETLBATT - uses Read2()
    case 30:  // GETM1ISPEED - uses Read4_1()
    case 31:  // GETM2ISPEED - uses Read4_1()
    case 47:  // GETBUFFERS - uses Read2()
    case 48:  // GETPWMS - uses Read4()
    case 49:  // GETCURRENTS - uses Read4()
    case 55:  // READM1PID - complex read with CRC
    case 56:  // READM2PID - complex read with CRC
    case 59:  // GETMINMAXMAINVOLTAGES - uses Read4()
    case 60:  // GETMINMAXLOGICVOLTAGES - uses Read4()
    case 63:  // READM1POSPID - complex read with CRC
    case 64:  // READM2POSPID - complex read with CRC
    case 75:  // GETPINFUNCTIONS - custom read with CRC
    case 77:  // GETDEADBAND - uses Read2()
    case 78:  // GETENCODERS - complex read with CRC
    case 79:  // GETISPEEDS - complex read with CRC
    case 82:  // GETTEMP - uses Read2()
    case 83:  // GETTEMP2 - uses Read2()
    case 90:  // GETERROR - uses Read4()
    case 91:  // GETENCODERMODE - uses Read2()
    case 99:  // GETCONFIG - uses Read2()
    case 135: // GETM1MAXCURRENT - complex read with CRC
    case 136: // GETM2MAXCURRENT - complex read with CRC
    case 149: // GETPWMMODE - uses Read1()
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

    if (!writeBytes(buf.data(), buf.size(), err)) {
      return false;
    }

    // Read acknowledgment (0xFF) like reference implementation
    uint8_t ack = readByte(0.1, err);
    if (!err.empty()) {
      return false;
    }

    if (ack != 0xFF) {
      err = "Invalid ACK response, expected 0xFF but got 0x" +
        std::to_string(ack);
      return false;
    }

    return true;
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
    value = (uint32_t(data[0]) << 24) | (uint32_t(data[1]) << 16) | (uint32_t(data[2]) << 8) |
      data[3];
    status = data[4];
    return true;
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

  bool RoboClawDevice::resetEncoders(std::string& err) {
    uint8_t cmd = 20;
    // RESETENC command needs CRC according to reference implementation
    return sendCrc(cmd, nullptr, 0, err);
  }

  std::string RoboClawDevice::version() {
    std::string err;
    uint8_t cmd = 21;  // GET_VERSION command

    if (!sendSimple(cmd, nullptr, 0, err)) {
      return "";
    }

    std::vector<uint8_t> accum;
    accum.reserve(64);

    // Read until null terminator (like reference implementation)
    for (int i = 0; i < 48; ++i) {  // Reference reads up to 48 chars
      uint8_t ch = 0;
      std::string e2;
      ch = readByte(0.1, e2);  // Increased timeout to 100ms
      if (!e2.empty()) {
        return "";
      }

      accum.push_back(ch);  // Include the byte in accumulator

      if (ch == '\0') {  // Found null terminator - read CRC next
        break;
      }
    }

    // Read CRC
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

    // Verify CRC - include ALL bytes including null terminator
    uint16_t crc_calc = 0;
    updateCrc(crc_calc, addr_);
    updateCrc(crc_calc, cmd);
    for (auto b : accum) updateCrc(crc_calc, b);  // This now includes the null terminator
    uint16_t crc_rx = (uint16_t(hi) << 8) | lo;

    if (crc_calc != crc_rx) {
      return "";
    }

    // Create string without the null terminator
    std::string version_str;
    for (auto b : accum) {
      if (b == '\0') break;
      version_str += (char)b;
    }

    return version_str;
  }

  bool RoboClawDevice::readSnapshot(Snapshot& out, std::string& err) {
    uint32_t v = 0;
    uint8_t status = 0;
    // Use GETM1ENC (16) and GETM2ENC (17) commands, not SETM1ENCCOUNT (22)/SETM2ENCCOUNT (23)
    if (!readU32WithStatus(16, v, status, err)) return false;
    out.m1_enc.value = v;
    out.m1_enc.status = status;
    if (!readU32WithStatus(17, v, status, err)) return false;
    out.m2_enc.value = v;
    out.m2_enc.status = status;
    int32_t vel = 0;
    // Use GETM1ISPEED (30) and GETM2ISPEED (31) for instantaneous speed
    if (readVelocity(30, vel, err)) out.m1_enc.speed_qpps = vel;
    if (readVelocity(31, vel, err)) out.m2_enc.speed_qpps = vel;
    uint16_t mv = 0;
    // Use GETMBATT (24) for main battery voltage
    if (readU16(24, mv, err)) out.volts.main = mv / 10.0f;
    uint16_t lv = 0;
    // Use GETLBATT (25) for logic battery voltage
    if (readU16(25, lv, err)) out.volts.logic = lv / 10.0f;
    uint16_t m1 = 0, m2 = 0;
    // Use GETCURRENTS (49) for motor currents
    if (readCurrents(49, m1, m2, err)) {
      out.currents.m1_inst = m1 / 100.0f;
      out.currents.m2_inst = m2 / 100.0f;
    }
    uint16_t temp_raw = 0;
    // Use GETTEMP (82) for temperature
    if (readU16(82, temp_raw, err)) out.temps.t1 = temp_raw / 10.0f;
    uint32_t status_bits = 0;
    // Use GETERROR (90) for status bits
    if (readU32(90, status_bits, err)) out.status_bits = status_bits;
    return true;
  }

}  // namespace roboclaw_driver
