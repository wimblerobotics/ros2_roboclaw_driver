// MIT License
#include "roboclaw_driver/roboclaw_device.hpp"
#include "roboclaw_driver/transport.hpp"
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <cstring>
#include <cerrno>
#include <vector>
#include <sstream>
#include <chrono>
#include <thread>
#include <mutex>

namespace roboclaw_driver {

#define MAXRETRY 2

  RoboClawDevice::RoboClawDevice(std::shared_ptr<ITransport> transport, uint8_t address)
    : transport_(std::move(transport)), addr_(address) {
  }

  RoboClawDevice::RoboClawDevice(const std::string& device, int baud_rate, uint8_t address)
    : device_(device), baud_(baud_rate), addr_(address) {
    if (!transport_) {
      std::string err; openPort(err);
    }
  }

  RoboClawDevice::~RoboClawDevice() {
    if (!transport_ && fd_ >= 0) ::close(fd_);
  }

  bool RoboClawDevice::openPort(std::string& err) {
    if (transport_) return true; // external transport handles opening

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
    if (transport_) { transport_->flush(); return; }
    if (fd_ < 0) return;
    tcflush(fd_, TCIOFLUSH);
  }

  bool RoboClawDevice::writeBytes(const uint8_t* data, size_t len, std::string& err) {
    if (transport_) return transport_->write(data, len, err);

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
    if (transport_) return transport_->readByte(timeout_sec, err);

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
  }

  bool RoboClawDevice::readBytes(uint8_t* dst, size_t len, double timeout_sec, std::string& err) {
    if (transport_) return transport_->read(dst, len, timeout_sec, err);

    for (size_t i = 0; i < len; ++i) {
      dst[i] = readByte(timeout_sec, err);
      if (!err.empty()) return false;
    }
    return true;
  }

  static std::string toHex(const std::vector<uint8_t>& data) {
    std::ostringstream oss;
    for (size_t i = 0;i < data.size();++i) { if (i) oss << ' '; oss << std::hex << std::uppercase; oss.width(2); oss.fill('0'); oss << int(data[i]); }
    return oss.str();
  }
  static std::string byteToHex(uint8_t b) { std::vector<uint8_t> v{ b }; return toHex(v); }

  bool RoboClawDevice::sendCommandWrite(uint8_t command, const uint8_t* payload, size_t len, std::string& err) {
    auto& st = cmd_stats_[command];
    st.attempts++;
    last_command_ = command;
    last_was_write_ = true;
    std::vector<uint8_t> frame; frame.reserve(2 + len + 2);
    frame.push_back(addr_); frame.push_back(command);
    for (size_t i = 0; i < len; ++i) frame.push_back(payload[i]);
    uint16_t crc = 0; for (uint8_t b : frame) updateCrc(crc, b);
    frame.push_back(uint8_t(crc >> 8)); frame.push_back(uint8_t(crc & 0xFF));
    last_tx_ = toHex(frame);
    std::string werr;
    uint8_t ack = 0;
    {
      // Critical section: prevent read commands (sensor thread) from interleaving and
      // consuming the expected 0xFF ACK byte.
      std::lock_guard<std::mutex> lk(io_mutex_);
      if (!writeBytes(frame.data(), frame.size(), werr)) {
        st.io_fail++; err = werr; flush(); std::this_thread::sleep_for(std::chrono::milliseconds(12)); return false;
      }
      std::string rerr; ack = readByte(0.05, rerr);
      if (!rerr.empty()) { st.io_fail++; err = "ack read failure:" + rerr; flush(); std::this_thread::sleep_for(std::chrono::milliseconds(12)); return false; }
    }
    last_rx_ = byteToHex(ack);
    if (ack != 0xFF) { err = "unexpected ack byte"; return false; }
    return true;
  }

  bool RoboClawDevice::sendCommandRead(uint8_t command, std::string& err) {
    auto& st = cmd_stats_[command];
    st.attempts++;
    last_command_ = command;
    last_was_write_ = false;
    uint8_t frame[2] = { addr_, command };
    last_tx_ = toHex(std::vector<uint8_t>(frame, frame + 2));
    std::string werr; {
      std::lock_guard<std::mutex> lk(io_mutex_);
      if (!writeBytes(frame, 2, werr)) { st.io_fail++; err = werr; flush(); std::this_thread::sleep_for(std::chrono::milliseconds(12)); return false; }
    }
    return true;
  }

  bool RoboClawDevice::readU16(uint8_t cmd, uint16_t& val, std::string& err) {
    if (!sendCommandRead(cmd, err)) return false;
    std::vector<uint8_t> rx;
    uint8_t data[2]; if (!readBytes(data, 2, 0.05, err)) { last_rx_ = "ERR:" + err; return false; } rx.insert(rx.end(), data, data + 2);
    uint8_t crc_extra[2]; if (!readBytes(crc_extra, 2, 0.05, err)) { last_rx_ = toHex(rx) + " | ERR:" + err; return false; } rx.insert(rx.end(), crc_extra, crc_extra + 2);
    uint16_t crc = 0; updateCrc(crc, addr_); updateCrc(crc, cmd); updateCrc(crc, data[0]); updateCrc(crc, data[1]); uint16_t rxcrc = (uint16_t(crc_extra[0]) << 8) | crc_extra[1];
    last_rx_ = toHex(rx);
    if (crc != rxcrc) {
      cmd_stats_[last_command_].crc_fail++;
      last_rx_ = toHex(rx);
      err = "crc mismatch";
      flush();
      std::this_thread::sleep_for(std::chrono::milliseconds(12));
      return false;
    }
    val = (uint16_t(data[0]) << 8) | data[1]; return true;
  }

  bool RoboClawDevice::readCurrents(uint8_t cmd, int16_t& m1, int16_t& m2, std::string& err) {
    if (!sendCommandRead(cmd, err)) return false;
    std::vector<uint8_t> rx;
    uint8_t data[4];
    if (!readBytes(data, 4, 0.05, err)) { last_rx_ = "ERR:" + err; return false; }
    rx.insert(rx.end(), data, data + 4);
    uint8_t crc_extra[2];
    if (!readBytes(crc_extra, 2, 0.05, err)) { last_rx_ = toHex(rx) + " | ERR:" + err; return false; }
    rx.insert(rx.end(), crc_extra, crc_extra + 2);
    uint16_t crc = 0; updateCrc(crc, addr_); updateCrc(crc, cmd);
    for (int i = 0; i < 4; ++i) updateCrc(crc, data[i]);
    uint16_t rxcrc = (uint16_t(crc_extra[0]) << 8) | crc_extra[1];
    last_rx_ = toHex(rx);
    if (crc != rxcrc) {
      cmd_stats_[last_command_].crc_fail++;
      last_rx_ = toHex(rx);
      err = "crc mismatch";
      flush();
      std::this_thread::sleep_for(std::chrono::milliseconds(12));
      return false;
    }
    m1 = (int16_t)((uint16_t)data[0] << 8 | data[1]);
    m2 = (int16_t)((uint16_t)data[2] << 8 | data[3]);
    return true;
  }

  bool RoboClawDevice::readU32(uint8_t cmd, uint32_t& val, std::string& err) {
    if (!sendCommandRead(cmd, err)) return false;
    std::vector<uint8_t> rx;
    uint8_t data[4];
    if (!readBytes(data, 4, 0.05, err)) { last_rx_ = "ERR:" + err; return false; }
    rx.insert(rx.end(), data, data + 4);
    uint8_t crc_extra[2];
    if (!readBytes(crc_extra, 2, 0.05, err)) { last_rx_ = toHex(rx) + " | ERR:" + err; return false; }
    rx.insert(rx.end(), crc_extra, crc_extra + 2);
    uint16_t crc = 0; updateCrc(crc, addr_); updateCrc(crc, cmd);
    for (int i = 0; i < 4; ++i) updateCrc(crc, data[i]);
    uint16_t rxcrc = (uint16_t(crc_extra[0]) << 8) | crc_extra[1];
    last_rx_ = toHex(rx);
    if (crc != rxcrc) {
      cmd_stats_[last_command_].crc_fail++;
      last_rx_ = toHex(rx);
      err = "crc mismatch";
      flush();
      std::this_thread::sleep_for(std::chrono::milliseconds(12));
      return false;
    }
    val = (uint32_t(data[0]) << 24) | (uint32_t(data[1]) << 16) | (uint32_t(data[2]) << 8) | data[3];
    return true;
  }

  bool RoboClawDevice::readU32WithStatus(uint8_t cmd, uint32_t& value, uint8_t& status, std::string& err) {
    if (!sendCommandRead(cmd, err)) return false;
    std::vector<uint8_t> rx;
    uint8_t data[5];
    if (!readBytes(data, 5, 0.05, err)) { last_rx_ = "ERR:" + err; return false; }
    rx.insert(rx.end(), data, data + 5);
    uint8_t crc_extra[2];
    if (!readBytes(crc_extra, 2, 0.05, err)) { last_rx_ = toHex(rx) + " | ERR:" + err; return false; }
    rx.insert(rx.end(), crc_extra, crc_extra + 2);
    uint16_t crc = 0; updateCrc(crc, addr_); updateCrc(crc, cmd);
    for (int i = 0; i < 5; ++i) updateCrc(crc, data[i]);
    uint16_t rxcrc = (uint16_t(crc_extra[0]) << 8) | crc_extra[1];
    last_rx_ = toHex(rx);
    if (crc != rxcrc) {
      cmd_stats_[last_command_].crc_fail++;
      last_rx_ = toHex(rx);
      err = "crc mismatch";
      flush();
      std::this_thread::sleep_for(std::chrono::milliseconds(12));
      return false;
    }
    value = (uint32_t(data[0]) << 24) | (uint32_t(data[1]) << 16) | (uint32_t(data[2]) << 8) | data[3];
    status = data[4];
    return true;
  }

  bool RoboClawDevice::readVelocity(uint8_t cmd, int32_t& vel, std::string& err) {
    if (!sendCommandRead(cmd, err)) return false;
    std::vector<uint8_t> rx;
    uint8_t data[5];
    if (!readBytes(data, 5, 0.05, err)) { last_rx_ = "ERR:" + err; return false; }
    rx.insert(rx.end(), data, data + 5);
    uint8_t crc_extra[2];
    if (!readBytes(crc_extra, 2, 0.05, err)) { last_rx_ = toHex(rx) + " | ERR:" + err; return false; }
    rx.insert(rx.end(), crc_extra, crc_extra + 2);
    uint16_t crc = 0; updateCrc(crc, addr_); updateCrc(crc, cmd);
    for (int i = 0; i < 5; ++i) updateCrc(crc, data[i]);
    uint16_t rxcrc = (uint16_t(crc_extra[0]) << 8) | crc_extra[1];
    last_rx_ = toHex(rx);
    if (crc != rxcrc) {
      cmd_stats_[last_command_].crc_fail++;
      last_rx_ = toHex(rx);
      err = "crc mismatch";
      flush();
      std::this_thread::sleep_for(std::chrono::milliseconds(12));
      return false;
    }
    int32_t raw = (int32_t(data[0]) << 24) | (int32_t(data[1]) << 16) | (int32_t(data[2]) << 8) | data[3];
    if (data[4] != 0) raw = -raw;
    vel = raw;
    return true;
  }

  bool RoboClawDevice::setPID(int motor, float p, float i, float d, uint32_t qpps, std::string& err) {
    uint8_t cmd = (motor == 1) ? 28 : 29; uint32_t P = (uint32_t)(p * 65536.0f); uint32_t I = (uint32_t)(i * 65536.0f); uint32_t D = (uint32_t)(d * 65536.0f); uint8_t payload[16] = { (uint8_t)(D >> 24),(uint8_t)(D >> 16),(uint8_t)(D >> 8),(uint8_t)D,(uint8_t)(P >> 24),(uint8_t)(P >> 16),(uint8_t)(P >> 8),(uint8_t)P,(uint8_t)(I >> 24),(uint8_t)(I >> 16),(uint8_t)(I >> 8),(uint8_t)I,(uint8_t)(qpps >> 24),(uint8_t)(qpps >> 16),(uint8_t)(qpps >> 8),(uint8_t)qpps }; return sendCommandWrite(cmd, payload, 16, err);
  }

  bool RoboClawDevice::read4Values(uint8_t cmd, uint32_t& v1, uint32_t& v2, uint32_t& v3, uint32_t& v4, std::string& err) { if (!sendCommandRead(cmd, err)) return false; std::vector<uint8_t> rx; uint8_t data[16]; if (!readBytes(data, 16, 0.1, err)) { last_rx_ = "ERR:" + err; return false; } rx.insert(rx.end(), data, data + 16); uint8_t crc_extra[2]; if (!readBytes(crc_extra, 2, 0.05, err)) { last_rx_ = toHex(rx) + " | ERR:" + err; return false; } rx.insert(rx.end(), crc_extra, crc_extra + 2); uint16_t crc = 0; updateCrc(crc, addr_); updateCrc(crc, cmd); for (int i = 0;i < 16;++i) updateCrc(crc, data[i]); uint16_t rxcrc = (uint16_t(crc_extra[0]) << 8) | crc_extra[1]; last_rx_ = toHex(rx); if (crc != rxcrc) { err = "crc mismatch"; return false; } v1 = (uint32_t(data[0]) << 24) | (uint32_t(data[1]) << 16) | (uint32_t(data[2]) << 8) | data[3]; v2 = (uint32_t(data[4]) << 24) | (uint32_t(data[5]) << 16) | (uint32_t(data[6]) << 8) | data[7]; v3 = (uint32_t(data[8]) << 24) | (uint32_t(data[9]) << 16) | (uint32_t(data[10]) << 8) | data[11]; v4 = (uint32_t(data[12]) << 24) | (uint32_t(data[13]) << 16) | (uint32_t(data[14]) << 8) | data[15]; return true; }

  bool RoboClawDevice::readPID(int motor, PIDSnapshot& pid_out, std::string& err) {
    uint8_t cmd = (motor == 1) ? 55 : 56; uint32_t P, I, D, qpps; if (!read4Values(cmd, P, I, D, qpps, err)) return false; pid_out.p = static_cast<float>(P) / 65536.0f; pid_out.i = static_cast<float>(I) / 65536.0f; pid_out.d = static_cast<float>(D) / 65536.0f; pid_out.qpps = qpps; return true;
  }

  bool RoboClawDevice::driveSpeeds(int32_t m1_qpps, int32_t m2_qpps, std::string& err) {
    uint8_t cmd = 37;
    uint8_t payload[8] = {
      (uint8_t)(m1_qpps >> 24),(uint8_t)(m1_qpps >> 16),(uint8_t)(m1_qpps >> 8),(uint8_t)m1_qpps,
      (uint8_t)(m2_qpps >> 24),(uint8_t)(m2_qpps >> 16),(uint8_t)(m2_qpps >> 8),(uint8_t)m2_qpps
    };
    bool ok = sendCommandWrite(cmd, payload, 8, err);
    if (ok) { last_cmd_m1_ = m1_qpps; last_cmd_m2_ = m2_qpps; }
    return ok;
  }

  bool RoboClawDevice::driveSpeedsAccelDistance(int32_t m1_qpps, int32_t m2_qpps, uint32_t accel, uint32_t distance, std::string& err) {
    // SPEC Cmd 46 (MIXEDSPEEDACCELDIST): Accel(4) SpeedM1(4) DistanceM1(4) SpeedM2(4) DistanceM2(4) Buffer(1)
    // Payload bytes = 4+4+4+4+4+1 = 21. Frame on wire: addr(1)+cmd(1)+payload(21)+CRC(2)=25 bytes then ACK 0xFF.
    uint8_t cmd = 46;
    uint8_t buffer_mode = 1; // immediate override
    uint8_t payload[21] = {
      // Accel
      (uint8_t)(accel >> 24),(uint8_t)(accel >> 16),(uint8_t)(accel >> 8),(uint8_t)accel,
      // SpeedM1
      (uint8_t)(m1_qpps >> 24),(uint8_t)(m1_qpps >> 16),(uint8_t)(m1_qpps >> 8),(uint8_t)m1_qpps,
      // DistanceM1
      (uint8_t)(distance >> 24),(uint8_t)(distance >> 16),(uint8_t)(distance >> 8),(uint8_t)distance,
      // SpeedM2
      (uint8_t)(m2_qpps >> 24),(uint8_t)(m2_qpps >> 16),(uint8_t)(m2_qpps >> 8),(uint8_t)m2_qpps,
      // DistanceM2
      (uint8_t)(distance >> 24),(uint8_t)(distance >> 16),(uint8_t)(distance >> 8),(uint8_t)distance,
      // Buffer flag
      buffer_mode };
    bool ok = sendCommandWrite(cmd, payload, 21, err);
    if (ok) { last_cmd_m1_ = m1_qpps; last_cmd_m2_ = m2_qpps; }
    return ok;
  }

  bool RoboClawDevice::resetEncoders(std::string& err) { uint8_t cmd = 20; return sendCommandWrite(cmd, nullptr, 0, err); }

  std::string RoboClawDevice::version() {
    // SPEC: Command 21 returns a NULL-terminated ASCII string (max 48 incl terminator) then 16-bit CRC.
    // CRC covers: address, command (0x15), every data byte INCLUDING the terminating 0x00.
    last_command_ = 21; last_was_write_ = false; last_tx_.clear(); last_rx_.clear();
    std::string err; if (!sendCommandRead(21, err)) { last_rx_ = "SEND_FAIL:" + err; return ""; }
    std::vector<uint8_t> data; data.reserve(48); bool got_term = false; std::string rx_err;
    for (int i = 0; i < 48; ++i) {
      std::string e; uint8_t ch = readByte(0.1, e);
      if (!e.empty()) { rx_err = e; break; }
      data.push_back(ch);
      if (ch == 0) { got_term = true; break; }
    }
    if (!rx_err.empty()) { last_rx_ = std::string("RX_ERR bytes=") + toHex(data) + " err=" + rx_err; return ""; }
    if (!got_term) { last_rx_ = std::string("NO_TERM bytes=") + toHex(data); return ""; }
    // Read CRC (2 bytes)
    uint8_t crc_bytes[2]; std::string ecrc;
    crc_bytes[0] = readByte(0.1, ecrc); if (!ecrc.empty()) { last_rx_ = "CRC_HI_ERR bytes=" + toHex(data); return ""; }
    crc_bytes[1] = readByte(0.1, ecrc); if (!ecrc.empty()) { last_rx_ = "CRC_LO_ERR bytes=" + toHex(data); return ""; }
    uint16_t rxcrc = (uint16_t(crc_bytes[0]) << 8) | crc_bytes[1];
    // Compute CRC over address, command, data (including terminator present as last element)
    uint16_t crc = 0; updateCrc(crc, addr_); updateCrc(crc, 21); for (uint8_t b : data) updateCrc(crc, b);
    if (crc != rxcrc) {
      std::ostringstream oss; oss << "CRC_FAIL bytes=" << toHex(data) << " rxcrc=" << std::hex << std::uppercase << rxcrc << " expect=" << crc;
      last_rx_ = oss.str();
      return "";
    }
    // Build printable string excluding terminator
    std::string ver;
    for (uint8_t b : data) { if (b == 0) break; ver.push_back(char(b)); }
    std::ostringstream oss; oss << "OK bytes=" << toHex(data) << " rxcrc=" << std::hex << std::uppercase << rxcrc;
    last_rx_ = oss.str();
    return ver;
  }

  bool RoboClawDevice::readSnapshot(Snapshot& out, std::string& err) {
    // Order matters to avoid stale data; gather encoder, velocity, currents, volts, temps, status, PID.
    if (!readU32WithStatus(16, out.m1_enc.value, out.m1_enc.status, err)) return false;
    if (!readU32WithStatus(17, out.m2_enc.value, out.m2_enc.status, err)) return false;
    if (!readVelocity(30, out.m1_enc.speed_qpps, err)) return false;
    if (!readVelocity(31, out.m2_enc.speed_qpps, err)) return false;
    int16_t c1, c2;
    if (!readCurrents(49, c1, c2, err)) return false;
    out.currents.m1_inst = c1 / 100.0f;
    out.currents.m2_inst = c2 / 100.0f;
    uint16_t mv;
    if (!readU16(24, mv, err)) return false;
    out.volts.main = mv / 10.0f;
    if (!readU16(25, mv, err)) return false;
    out.volts.logic = mv / 10.0f;
    if (!readU16(82, mv, err)) return false;
    out.temps.t1 = mv / 10.0f;
    if (!readU16(83, mv, err)) {
      out.temps.t2 = 0.0f;
    } else {
      out.temps.t2 = mv / 10.0f;
    }
    uint32_t status;
    if (!readU32(90, status, err)) return false;
    out.status_bits = status;
    PIDSnapshot p1, p2;  // optional
    std::string perr;
    if (readPID(1, p1, perr)) out.m1_pid = p1;
    if (readPID(2, p2, perr)) out.m2_pid = p2;
    return true;
  }

  bool RoboClawDevice::readMotorVelocity(int motor, int32_t& vel, std::string& err) {
    uint8_t cmd = motor == 1 ? 30 : 31; return readVelocity(cmd, vel, err);
  }

} // namespace roboclaw_driver
