// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Wimblerobotics
// https://github.com/wimblerobotics/ros2_roboclaw_driver

#include "ros2_roboclaw_driver/RoboClaw.h"

#include <fcntl.h>
#include <math.h>
#include <poll.h>
#include <rcutils/logging_macros.h>
#include <stdio.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include <boost/assign.hpp>
#include <cstdint>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sstream>

#include "ros2_roboclaw_driver/roboclaw_cmd_do_buffered_m1m2_drive_speed_accel_distance.h"
#include "ros2_roboclaw_driver/roboclaw_cmd_read_encoder.h"
#include "ros2_roboclaw_driver/roboclaw_cmd_read_encoder_speed.h"
#include "ros2_roboclaw_driver/roboclaw_cmd_read_firmware_version.h"
#include "ros2_roboclaw_driver/roboclaw_cmd_read_logic_battery_voltage.h"
#include "ros2_roboclaw_driver/roboclaw_cmd_read_main_battery_voltage.h"
#include "ros2_roboclaw_driver/roboclaw_cmd_read_motor_currents.h"
#include "ros2_roboclaw_driver/roboclaw_cmd_read_motor_velocity_pidq.h"
#include "ros2_roboclaw_driver/roboclaw_cmd_read_status.h"
#include "ros2_roboclaw_driver/roboclaw_cmd_read_temperature.h"
#include "ros2_roboclaw_driver/roboclaw_cmd_set_encoder_value.h"
#include "ros2_roboclaw_driver/roboclaw_cmd_set_pid.h"
#include "ros2_roboclaw_driver/srv/reset_encoders.h"

const char* RoboClaw::motorNames_[] = {"M1", "M2", "NONE"};

// Initialize the static mutex
std::mutex RoboClaw::buffered_command_mutex_;

RoboClaw::RoboClaw(const TPIDQ m1Pid, const TPIDQ m2Pid, float m1MaxCurrent, float m2MaxCurrent,
                   std::string device_name, uint8_t device_port, uint32_t baud_rate, bool(do_debug),
                   bool do_low_level_debug)
    : do_debug_(do_debug),
      do_low_level_debug_(do_low_level_debug),
      baud_rate_(baud_rate),
      serial_fd_(device_port),
      maxCommandRetries_(3),
      maxM1Current_(m1MaxCurrent),
      maxM2Current_(m2MaxCurrent),
      device_name_(device_name),
      portAddress_(128),
      debug_log_(this) {
  debug_log_ = DebugLog(this);
  openPort();
  RCUTILS_LOG_INFO("[RoboClaw::RoboClaw] RoboClaw software version: %s", getVersion().c_str());
  setM1PID(m1Pid.p, m1Pid.i, m1Pid.d, m1Pid.qpps);
  setM2PID(m2Pid.p, m2Pid.i, m2Pid.d, m2Pid.qpps);
  CmdSetEncoderValue m1(*this, kM1, 0);
  m1.execute();
  CmdSetEncoderValue m2(*this, kM2, 0);
  m2.execute();
  // ros2_roboclaw_driver::srv::ResetEncoders::Request resetRequest;
  // resetRequest.left = 0;
  // resetRequest.right = 0;
  // ros2_roboclaw_driver::srv::ResetEncoders::Response response;
  // resetEncoders(resetRequest, response);
  g_singleton = this;
}

RoboClaw::~RoboClaw() {}

void RoboClaw::doMixedSpeedAccelDist(uint32_t accel_quad_pulses_per_second, int32_t m1_quad_pulses_per_second,
                                     uint32_t m1_max_distance, int32_t m2_quad_pulses_per_second,
                                     uint32_t m2_max_distance) {
  CmdDoBufferedM1M2DriveSpeedAccelDistance command(*this, accel_quad_pulses_per_second, m1_quad_pulses_per_second,
                                                   m1_max_distance, m2_quad_pulses_per_second, m2_max_distance);
  command.execute();
}

uint32_t RoboClaw::getErrorStatus() {
  uint32_t status = 0;
  CmdReadStatus cmd_read_status(*this, status);
  cmd_read_status.execute();
  return status;
}

void RoboClaw::decodeErrorStatus(uint32_t error_status, char* buffer, size_t size) const {
  if (error_status == 0) {
    strncpy(buffer, "No errors", size);
    buffer[size - 1] = '\0';
    return;
  }

  buffer[0] = '\0';
  size_t current_len = 0;
  bool first = true;

  auto append_error = [&](const char* error_str) {
    if (current_len >= size - 1)
      return;
    size_t needed = strlen(error_str) + (first ? 0 : 2);
    if (current_len + needed < size) {
      if (!first) {
        strcat(buffer, ", ");
        current_len += 2;
      }
      strcat(buffer, error_str);
      current_len += strlen(error_str);
      first = false;
    }
  };

  // Check error flags (bits 0-15)
  if (error_status & static_cast<uint32_t>(RoboClawError::ERROR_ESTOP))
    append_error("ERROR_ESTOP");
  if (error_status & static_cast<uint32_t>(RoboClawError::ERROR_TEMP))
    append_error("ERROR_TEMP");
  if (error_status & static_cast<uint32_t>(RoboClawError::ERROR_TEMP2))
    append_error("ERROR_TEMP2");
  if (error_status & static_cast<uint32_t>(RoboClawError::ERROR_LBATHIGH))
    append_error("ERROR_LBATHIGH");
  if (error_status & static_cast<uint32_t>(RoboClawError::ERROR_LBATLOW))
    append_error("ERROR_LBATLOW");
  if (error_status & static_cast<uint32_t>(RoboClawError::ERROR_FAULTM1))
    append_error("ERROR_FAULTM1");
  if (error_status & static_cast<uint32_t>(RoboClawError::ERROR_FAULTM2))
    append_error("ERROR_FAULTM2");
  if (error_status & static_cast<uint32_t>(RoboClawError::ERROR_SPEED1))
    append_error("ERROR_SPEED1");
  if (error_status & static_cast<uint32_t>(RoboClawError::ERROR_SPEED2))
    append_error("ERROR_SPEED2");
  if (error_status & static_cast<uint32_t>(RoboClawError::ERROR_POS1))
    append_error("ERROR_POS1");
  if (error_status & static_cast<uint32_t>(RoboClawError::ERROR_POS2))
    append_error("ERROR_POS2");
  if (error_status & static_cast<uint32_t>(RoboClawError::ERROR_CURRENTM1))
    append_error("ERROR_CURRENTM1");
  if (error_status & static_cast<uint32_t>(RoboClawError::ERROR_CURRENTM2))
    append_error("ERROR_CURRENTM2");

  // Check warning flags (bits 16-31)
  if (error_status & static_cast<uint32_t>(RoboClawError::WARN_OVERCURRENTM1))
    append_error("WARN_OVERCURRENTM1");
  if (error_status & static_cast<uint32_t>(RoboClawError::WARN_OVERCURRENTM2))
    append_error("WARN_OVERCURRENTM2");
  if (error_status & static_cast<uint32_t>(RoboClawError::WARN_MBATHIGH))
    append_error("WARN_MBATHIGH");
  if (error_status & static_cast<uint32_t>(RoboClawError::WARN_MBATLOW))
    append_error("WARN_MBATLOW");
  if (error_status & static_cast<uint32_t>(RoboClawError::WARN_TEMP))
    append_error("WARN_TEMP");
  if (error_status & static_cast<uint32_t>(RoboClawError::WARN_TEMP2))
    append_error("WARN_TEMP2");
  if (error_status & static_cast<uint32_t>(RoboClawError::WARN_S4))
    append_error("WARN_S4");
  if (error_status & static_cast<uint32_t>(RoboClawError::WARN_S5))
    append_error("WARN_S5");
  if (error_status & static_cast<uint32_t>(RoboClawError::WARN_CAN))
    append_error("WARN_CAN");
  if (error_status & static_cast<uint32_t>(RoboClawError::WARN_BOOT))
    append_error("WARN_BOOT");
  if (error_status & static_cast<uint32_t>(RoboClawError::WARN_OVERREGENM1))
    append_error("WARN_OVERREGENM1");
  if (error_status & static_cast<uint32_t>(RoboClawError::WARN_OVERREGENM2))
    append_error("WARN_OVERREGENM2");

  // Report any unknown bits
  uint32_t known_errors = 0xF000EFFF;  // All defined error and warning bits
  uint32_t unknown_errors = error_status & ~known_errors;
  if (unknown_errors != 0) {
    if (current_len < size - 1 && !first) {
      strcat(buffer, ", ");
      current_len += 2;
    }
    snprintf(buffer + current_len, size - current_len, "UNKNOWN:0x%X", (unsigned int)unknown_errors);
  }
}

// 32-bit status decode per updated RoboClaw documentation.
// Lower 16 bits retain classic meanings. Upper bits (16-31) may include
// extended status such as position/speed error limits or other firmware
// specific flags; placeholders added for known documented bits. Adjust names
// to match exact manual revision you are using.
std::string RoboClaw::getErrorString(uint32_t status) {
  if (status == 0)
    return "normal";
  std::stringstream ss;
  // Bit 0-15 (classic)
  if (status & 0x00000001) {
    ss << "[M1 OverCurrent Warning] ";
    motorAlarms_ |= kM1_OVER_CURRENT_ALARM;
  } else {
    motorAlarms_ &= ~kM1_OVER_CURRENT_ALARM;
  }
  if (status & 0x00000002) {
    ss << "[M2 OverCurrent Warning] ";
    motorAlarms_ |= kM2_OVER_CURRENT_ALARM;
  } else {
    motorAlarms_ &= ~kM2_OVER_CURRENT_ALARM;
  }
  if (status & 0x00000004)
    ss << "[E-Stop] ";
  if (status & 0x00000008)
    ss << "[Temperature Error] ";
  if (status & 0x00000010)
    ss << "[Temperature2 Error] ";
  if (status & 0x00000020)
    ss << "[Main Battery High Error] ";
  if (status & 0x00000040)
    ss << "[Logic Battery High Error] ";
  if (status & 0x00000080)
    ss << "[Logic Battery Low Error] ";
  if (status & 0x00000100)
    ss << "[M2 Driver Fault] ";
  if (status & 0x00000200)
    ss << "[M1 Driver Fault] ";
  if (status & 0x00000400)
    ss << "[Main Battery High Warning] ";
  if (status & 0x00000800)
    ss << "[Main Battery Low Warning] ";
  if (status & 0x00001000)
    ss << "[Temperature Warning] ";
  if (status & 0x00002000)
    ss << "[Temperature2 Warning] ";
  if (status & 0x00004000)
    ss << "[M1 Home] ";
  if (status & 0x00008000)
    ss << "[M2 Home] ";
  // Upper 16 bits (example labels; verify with manual revision)
  if (status & 0x00010000)
    ss << "[Speed Error Limit Warning] ";
  if (status & 0x00020000)
    ss << "[Position Error Limit Warning] ";
  if (status & 0x00040000)
    ss << "[Reserved Bit 18] ";
  if (status & 0x00080000)
    ss << "[Reserved Bit 19] ";
  if (status & 0x00100000)
    ss << "[Reserved Bit 20] ";
  if (status & 0x00200000)
    ss << "[Reserved Bit 21] ";
  if (status & 0x00400000)
    ss << "[Reserved Bit 22] ";
  if (status & 0x00800000)
    ss << "[Reserved Bit 23] ";
  if (status & 0x01000000)
    ss << "[Extended Flag 24] ";
  if (status & 0x02000000)
    ss << "[Extended Flag 25] ";
  if (status & 0x04000000)
    ss << "[Extended Flag 26] ";
  if (status & 0x08000000)
    ss << "[Extended Flag 27] ";
  if (status & 0x10000000)
    ss << "[Extended Flag 28] ";
  if (status & 0x20000000)
    ss << "[Extended Flag 29] ";
  if (status & 0x40000000)
    ss << "[Extended Flag 30] ";
  if (status & 0x80000000)
    ss << "[Extended Flag 31] ";
  return ss.str();
}

float RoboClaw::getLogicBatteryLevel() {
  float logic_battery_level = 0.0;
  CmdReadLogicBatteryVoltage cmd_logic_battery(*this, logic_battery_level);
  cmd_logic_battery.execute();
  return logic_battery_level;
}

float RoboClaw::getMainBatteryLevel() {
  float main_battery_level = 0.0;
  CmdReadMainBatteryVoltage cmd_main_battery(*this, main_battery_level);
  cmd_main_battery.execute();
  return main_battery_level;
}

// ### change result type to uint16_t
unsigned short RoboClaw::get2ByteCommandResult2(uint8_t command) {
  uint16_t crc = 0;
  updateCrc(crc, portAddress_);
  updateCrc(crc, command);

  writeN2(false, 2, portAddress_, command);
  unsigned short result = 0;
  uint8_t datum = readByteWithTimeout2();
  result |= datum << 8;
  updateCrc(crc, datum);

  datum = readByteWithTimeout2();
  result |= datum;
  updateCrc(crc, datum);

  uint16_t responseCrc = 0;
  datum = readByteWithTimeout2();
  responseCrc = datum << 8;
  datum = readByteWithTimeout2();
  responseCrc |= datum;
  if (responseCrc == crc) {
    return result;
  } else {
    RCUTILS_LOG_ERROR(
        "[RoboClaw::get2ByteCommandResult2] invalid CRC expected: "
        "0x%02X, got: 0x%02X",
        crc, responseCrc);
    throw new TRoboClawException("[RoboClaw::get2ByteCommandResult2 INVALID CRC");
    return 0;
  }
}

RoboClaw::TMotorCurrents RoboClaw::getMotorCurrents() {
  TMotorCurrents motor_currents{};
  CmdReadMotorCurrents cmd_read_motor_currents(*this, motor_currents);
  cmd_read_motor_currents.execute();
  return motor_currents;
}

RoboClaw::TPIDQ RoboClaw::getPIDQM1() {
  TPIDQ read_velocity_pidq_result;
  CmdReadMotorVelocityPIDQ cmd_read_motor_velocity_pidq(*this, kM1, read_velocity_pidq_result);
  cmd_read_motor_velocity_pidq.execute();
  return read_velocity_pidq_result;
}

RoboClaw::TPIDQ RoboClaw::getPIDQM2() {
  TPIDQ read_velocity_pidq_result;
  CmdReadMotorVelocityPIDQ cmd_read_motor_velocity_pidq(*this, kM2, read_velocity_pidq_result);
  cmd_read_motor_velocity_pidq.execute();
  return read_velocity_pidq_result;
}

float RoboClaw::getTemperature() {
  float temperature = 0.0;
  CmdReadTemperature cmd_read_temperature(*this, temperature);
  cmd_read_temperature.execute();
  return temperature;
}

unsigned long RoboClaw::getUlongCommandResult2(uint8_t command) {
  uint16_t crc = 0;
  updateCrc(crc, portAddress_);
  updateCrc(crc, command);

  writeN2(false, 2, portAddress_, command);
  unsigned long result = 0;
  uint8_t datum = readByteWithTimeout2();
  result |= datum << 24;
  updateCrc(crc, datum);
  datum = readByteWithTimeout2();
  result |= datum << 16;
  updateCrc(crc, datum);
  datum = readByteWithTimeout2();
  result |= datum << 8;
  updateCrc(crc, datum);
  datum = readByteWithTimeout2();
  result |= datum;
  updateCrc(crc, datum);

  uint16_t responseCrc = 0;
  datum = readByteWithTimeout2();
  responseCrc = datum << 8;
  datum = readByteWithTimeout2();
  responseCrc |= datum;
  if (responseCrc == crc) {
    return result;
  }

  RCUTILS_LOG_ERROR(
      "[RoboClaw::getUlongCommandResult2] Expected CRC of: 0x%02X, but "
      "got: 0x%02X",
      int(crc), int(responseCrc));
  throw new TRoboClawException("[RoboClaw::getUlongCommandResult2] INVALID CRC");
  return 0;
}

uint32_t RoboClaw::getULongCont2(uint16_t& crc) {
  uint32_t result = 0;
  uint8_t datum = readByteWithTimeout2();
  result |= datum << 24;
  updateCrc(crc, datum);
  datum = readByteWithTimeout2();
  result |= datum << 16;
  updateCrc(crc, datum);
  datum = readByteWithTimeout2();
  result |= datum << 8;
  updateCrc(crc, datum);
  datum = readByteWithTimeout2();
  result |= datum;
  updateCrc(crc, datum);
  return result;
}

int32_t RoboClaw::getVelocityResult(uint8_t command) {
  uint16_t crc = 0;
  updateCrc(crc, portAddress_);
  updateCrc(crc, command);

  writeN2(false, 2, portAddress_, command);
  int32_t result = 0;
  uint8_t datum = readByteWithTimeout2();
  result |= datum << 24;
  updateCrc(crc, datum);

  datum = readByteWithTimeout2();
  result |= datum << 16;
  updateCrc(crc, datum);

  datum = readByteWithTimeout2();
  result |= datum << 8;
  updateCrc(crc, datum);

  datum = readByteWithTimeout2();
  result |= datum;
  updateCrc(crc, datum);

  uint8_t direction = readByteWithTimeout2();
  updateCrc(crc, direction);
  if (direction != 0)
    result = -result;

  uint16_t responseCrc = 0;
  datum = readByteWithTimeout2();
  responseCrc = datum << 8;
  datum = readByteWithTimeout2();
  responseCrc |= datum;
  if (responseCrc == crc) {
    return result;
  }

  RCUTILS_LOG_ERROR(
      "[RoboClaw::getVelocityResult] Expected CRC of: 0x%02X, but got: "
      "0x%02X",
      int(crc), int(responseCrc));
  throw new TRoboClawException("[RoboClaw::getVelocityResult] INVALID CRC");
  return 0;
}

uint32_t RoboClaw::getM1Encoder() {
  EncodeResult encoder_command_result{};
  CmdReadEncoder read_encoder_cmd(*this, kM1, encoder_command_result);
  read_encoder_cmd.execute();
  return encoder_command_result.value;
}

uint32_t RoboClaw::getM2Encoder() {
  EncodeResult encoder_command_result{};
  CmdReadEncoder read_encoder_cmd(*this, kM2, encoder_command_result);
  read_encoder_cmd.execute();
  return encoder_command_result.value;
}

std::string RoboClaw::getVersion() {
  std::string version;
  CmdReadFirmwareVersion command(*this, version);
  command.execute();
  return version;
}

void RoboClaw::openPort() {
  RCUTILS_LOG_INFO("[RoboClaw::openPort] about to open port: %s", device_name_.c_str());
  serial_fd_ = open(device_name_.c_str(), O_RDWR | O_NOCTTY);
  if (serial_fd_ < 0) {
    RCUTILS_LOG_ERROR(
        "[RoboClaw::openPort] Unable to open USB port: %s, errno: (%d) "
        "%s",
        device_name_.c_str(), errno, strerror(errno));
    throw new TRoboClawException("[RoboClaw::openPort] Unable to open USB port");
  }

  // Fetch the current port settings.
  struct termios portOptions;
  int ret = 0;

  ret = tcgetattr(serial_fd_, &portOptions);
  if (ret < 0) {
    RCUTILS_LOG_ERROR(
        "[RoboClaw::openPort] Unable to get terminal options "
        "(tcgetattr), error: %d: %s",
        errno, strerror(errno));
    throw new TRoboClawException("[RoboClaw::openPort] Unable to get terminal options (tcgetattr)");
  }

  if (cfsetispeed(&portOptions, B38400) < 0) {
    RCUTILS_LOG_ERROR(
        "[RoboClaw::openPort] Unable to set terminal speed "
        "(cfsetispeed)");
    throw new TRoboClawException(
        "[RoboClaw::openPort] Unable to set terminal speed "
        "(cfsetispeed)");
  }

  speed_t baud;
  switch (baud_rate_) {
    case 9600:
      baud = B9600;
      break;
    case 19200:
      baud = B19200;
      break;
    case 38400:
      baud = B38400;
      break;
    case 57600:
      baud = B57600;
      break;
    case 115200:
      baud = B115200;
      break;
    default:
      RCUTILS_LOG_ERROR("[RoboClaw::openPort] Unsupported baud rate: %u", baud_rate_);
      throw new TRoboClawException("[RoboClaw::openPort] Unsupported baud rate");
  }

  if (cfsetispeed(&portOptions, baud) < 0) {
    RCUTILS_LOG_ERROR(
        "[RoboClaw::openPort] Unable to set terminal input speed "
        "(cfsetispeed)");
    throw new TRoboClawException(
        "[RoboClaw::openPort] Unable to set terminal input speed "
        "(cfsetispeed)");
  }
  if (cfsetospeed(&portOptions, B38400) < 0) {
    RCUTILS_LOG_ERROR(
        "[RoboClaw::openPort] Unable to set terminal speed "
        "(cfsetospeed)");
    throw new TRoboClawException(
        "[RoboClaw::openPort] Unable to set terminal speed "
        "(cfsetospeed)");
  }

  // Configure other settings
  portOptions.c_cflag &= ~PARENB;         // Disable parity.
  portOptions.c_cflag &= ~CSTOPB;         // 1 stop bit
  portOptions.c_cflag &= ~CSIZE;          // Clear data size bits
  portOptions.c_cflag |= CS8;             // 8 data bits
  portOptions.c_cflag &= ~CRTSCTS;        // Disable hardware flow control
  portOptions.c_cflag |= CREAD | CLOCAL;  // Enable read and ignore control lines

  portOptions.c_lflag &= ~ICANON;  // Disable canonical mode
  portOptions.c_lflag &= ~ECHO;    // Disable echo
  portOptions.c_lflag &= ~ECHOE;   // Disable erasure
  portOptions.c_lflag &= ~ECHONL;  // Disable new-line echo
  portOptions.c_lflag &= ~ISIG;    // Disable interpretation of INTR, QUIT and SUSP

  portOptions.c_iflag &= ~(IXON | IXOFF | IXANY);  // Disable software flow control
  portOptions.c_iflag &=
      ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);  // Disable special handling of received bytes

  portOptions.c_oflag &= ~OPOST;  // Disable output processing

  portOptions.c_cc[VMIN] = 0;   // Non-blocking read
  portOptions.c_cc[VTIME] = 0;  // Timeout of 0.5 seconds

  if (tcsetattr(serial_fd_, TCSANOW, &portOptions) != 0) {
    RCUTILS_LOG_ERROR(
        "[RoboClaw::openPort] Unable to set terminal options "
        "(tcsetattr)");
    throw new TRoboClawException(
        "[RoboClaw::openPort] Unable to set terminal options "
        "(tcsetattr)");
  }
}

uint8_t RoboClaw::readByteWithTimeout2() {
  struct pollfd ufd[1];
  ufd[0].fd = serial_fd_;
  ufd[0].events = POLLIN;

  int retval = poll(ufd, 1, 11);
  if (retval < 0) {
    RCUTILS_LOG_ERROR("[RoboClaw::readByteWithTimeout2 Poll failed (%d) %s", errno, strerror(errno));
    throw new TRoboClawException("[RoboClaw::readByteWithTimeout2 Read error");
  } else if (retval == 0) {
    std::stringstream ev;
    ev << "[RoboClaw::readByteWithTimeout2 TIMEOUT revents: " << std::hex << ufd[0].revents;
    RCUTILS_LOG_ERROR(ev.str().c_str());
    throw new TRoboClawException("[RoboClaw::readByteWithTimeout2 TIMEOUT");
  } else if (ufd[0].revents & POLLERR) {
    RCUTILS_LOG_ERROR("[RoboClaw::readByteWithTimeout2 Error on socket");
    restartPort();
    throw new TRoboClawException("[RoboClaw::readByteWithTimeout2 Error on socket");
  } else if (ufd[0].revents & POLLIN) {
    unsigned char buffer[1];
    ssize_t bytesRead = ::read(serial_fd_, buffer, sizeof(buffer));
    if (bytesRead != 1) {
      RCUTILS_LOG_ERROR(
          "[RoboClaw::readByteWithTimeout2 Failed to read 1 byte, read: "
          "%d",
          (int)bytesRead);
      throw TRoboClawException("[RoboClaw::readByteWithTimeout2 Failed to read 1 byte");
    }

    if (do_debug_ || do_low_level_debug_) {
      appendToReadLog("%02X ", buffer[0]);
      if (do_low_level_debug_) {
        RCUTILS_LOG_INFO("Read: %02X", buffer[0]);
      }
    }

    return buffer[0];
  } else {
    RCUTILS_LOG_ERROR("[RoboClaw::readByteWithTimeout2 Unhandled case");
    throw new TRoboClawException("[RoboClaw::readByteWithTimeout2 Unhandled case");
  }

  return 0;
}

/*
bool RoboClaw::resetEncoders(
    ros2_roboclaw_driver::srv::ResetEncoders::Request &request,
    ros2_roboclaw_driver::srv::ResetEncoders::Response &response) {
  try {
    CmdSetEncoderValue m1(*this, kM1, request.left);
    CmdSetEncoderValue m2(*this, kM2, request.right);
    m1.execute();
    m2.execute();
    response.ok = true;
  } catch (...) {
    RCUTILS_LOG_ERROR("[RoboClaw::resetEncoders] uncaught exception");
  }
  return true;
}
*/

void RoboClaw::restartPort() {
  close(serial_fd_);
  usleep(200000);
  openPort();
}

void RoboClaw::setM1PID(float p, float i, float d, uint32_t qpps) {
  CmdSetPid command(*this, kM1, p, i, d, qpps);
  command.execute();
}

void RoboClaw::setM2PID(float p, float i, float d, uint32_t qpps) {
  CmdSetPid command(*this, kM2, p, i, d, qpps);
  command.execute();
}

void RoboClaw::stop() {
  CmdDoBufferedM1M2DriveSpeedAccelDistance stopCommand(*this, 0, 0, 0, 0, 0);
  stopCommand.execute();
}

void RoboClaw::updateCrc(uint16_t& crc, uint8_t data) {
  crc = crc ^ ((uint16_t)data << 8);
  for (int i = 0; i < 8; i++) {
    if (crc & 0x8000)
      crc = (crc << 1) ^ 0x1021;
    else
      crc <<= 1;
  }
}

void RoboClaw::writeByte2(uint8_t byte) {
  ssize_t result;
  do {
    result = ::write(serial_fd_, &byte, 1);
    // RCUTILS_LOG_INFO("--> wrote: 0x%02X, result: %ld", byte, result);  //
    // ####
    if (do_debug_ || do_low_level_debug_) {
      if (result == 1) {
        appendToWriteLog("%02X ", byte);
        if (do_low_level_debug_) {
          RCUTILS_LOG_INFO("Write: %02X", byte);
        }
      } else {
        appendToWriteLog("~%02X ", byte);
        if (do_low_level_debug_) {
          RCUTILS_LOG_ERROR("Write fail: %02X", byte);
        }
      }
    }

  } while (result == -1 && errno == EAGAIN);

  if (result != 1) {
    RCUTILS_LOG_ERROR(
        "[RoboClaw::writeByte2to write one byte, result: %d, "
        "errno: %d)",
        (int)result, errno);
    restartPort();
    throw new TRoboClawException("[RoboClaw::writeByte2 Unable to write one byte");
  }
}

void RoboClaw::writeN2(bool sendCRC, uint8_t cnt, ...) {
  tcflush(serial_fd_, TCIFLUSH);
  uint16_t crc = 0;
  va_list marker;
  va_start(marker, cnt);

  for (uint8_t i = 0; i < cnt; i++) {
    uint8_t byte = va_arg(marker, int);
    updateCrc(crc, byte);
    writeByte2(byte);
  }

  va_end(marker);

  if (sendCRC) {
    writeByte2(crc >> 8);
    writeByte2(crc);
    tcdrain(serial_fd_);

    uint8_t response = readByteWithTimeout2();
    if (response != 0xFF) {
      char msg[128];
      snprintf(msg, sizeof(msg),
               "[RoboClaw::writeN2] Invalid ACK response, expected 0xFF but got "
               "0x%02X",
               response);
      RCUTILS_LOG_ERROR("%s", msg);
      throw new TRoboClawException(msg);
    }
  }
}

RoboClaw* RoboClaw::singleton() {
  return g_singleton;
}

RoboClaw* RoboClaw::g_singleton = nullptr;
