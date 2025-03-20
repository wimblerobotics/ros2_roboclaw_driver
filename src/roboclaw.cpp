#include "roboclaw.h"

#include <cstdint>
#include <fcntl.h>
#include <math.h>
#include <poll.h>
#include <rcutils/logging_macros.h>
#include <stdio.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include <boost/assign.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sstream>

#include "ros2_roboclaw_driver/srv/reset_encoders.h"

const char *RoboClaw::motorNames_[] = {"M1", "M2", "NONE"};

// Initialize the static mutex
std::mutex RoboClaw::buffered_command_mutex_;

RoboClaw::RoboClaw(const TPIDQ m1Pid, const TPIDQ m2Pid, float m1MaxCurrent,
                   float m2MaxCurrent, std::string device_name,
                   uint8_t device_port)
    : doDebug_(true), doLowLevelDebug_(true), device_port_(device_port),
      maxCommandRetries_(3), maxM1Current_(m1MaxCurrent),
      maxM2Current_(m2MaxCurrent), device_name_(device_name),
      portAddress_(128) {
  openPort();
  RCUTILS_LOG_INFO("[RoboClaw::RoboClaw] RoboClaw software version: %s",
                   getVersion().c_str());
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
  readSensorGroup();
}

RoboClaw::~RoboClaw() {}

void RoboClaw::doMixedSpeedAccelDist(uint32_t accel_quad_pulses_per_second,
                                     int32_t m1_quad_pulses_per_second,
                                     uint32_t m1_max_distance,
                                     int32_t m2_quad_pulses_per_second,
                                     uint32_t m2_max_distance) {
  CmdDoBufferedM2M2DriveSpeedAccelDistance command(
      *this, accel_quad_pulses_per_second, m1_quad_pulses_per_second,
      m1_max_distance, m2_quad_pulses_per_second, m2_max_distance);
  command.execute();
}

uint16_t RoboClaw::getErrorStatus() {
  return g_sensor_value_group_.error_status;
}

std::string RoboClaw::getErrorString() {
  return g_sensor_value_group_.error_string;
}

std::string RoboClaw::getErrorString(uint16_t errorStatus) {
  if (errorStatus == 0)
    return "normal";
  else {
    std::stringstream errorMessage;
    if (errorStatus & 0x8000) {
      errorMessage << "[M2 Home] ";
    }

    if (errorStatus & 0x4000) {
      errorMessage << "[M1 Home] ";
    }

    if (errorStatus & 0x2000) {
      errorMessage << "[Temperature2 Warning] ";
    }

    if (errorStatus & 0x1000) {
      errorMessage << "[Temperature Warning] ";
    }

    if (errorStatus & 0x800) {
      errorMessage << "[Main Battery Low Warning] ";
    }

    if (errorStatus & 0x400) {
      errorMessage << "[Main Battery High Warning] ";
    }

    if (errorStatus & 0x200) {
      errorMessage << "[M1 Driver Fault] ";
    }

    if (errorStatus & 0x100) {
      errorMessage << "[M2 Driver Fault] ";
    }

    if (errorStatus & 0x80) {
      errorMessage << "[Logic Battery Low Error] ";
    }

    if (errorStatus & 0x40) {
      errorMessage << "[Logic Battery High Error] ";
    }

    if (errorStatus & 0x20) {
      errorMessage << "[Main Battery High Error] ";
    }

    if (errorStatus & 0x10) {
      errorMessage << "[Temperature2 Error] ";
    }

    if (errorStatus & 0x08) {
      errorMessage << "[Temperature Error] ";
    }

    if (errorStatus & 0x04) {
      errorMessage << "[E-Stop] ";
    }

    if (errorStatus & 0x02) {
      errorMessage << "[M2 OverCurrent Warning] ";
      motorAlarms_ |= kM2_OVER_CURRENT_ALARM;
    } else {
      motorAlarms_ &= ~kM2_OVER_CURRENT_ALARM;
    }

    if (errorStatus & 0x01) {
      errorMessage << "[M1 OverCurrent Warning] ";
      motorAlarms_ |= kM1_OVER_CURRENT_ALARM;
    } else {
      motorAlarms_ &= ~kM1_OVER_CURRENT_ALARM;
    }

    return errorMessage.str();
  }
}

float RoboClaw::getLogicBatteryLevel() {
  return g_sensor_value_group_.logic_battery_level;
}

float RoboClaw::getMainBatteryLevel() {
  return g_sensor_value_group_.main_battery_level;
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
    throw new TRoboClawException(
        "[RoboClaw::get2ByteCommandResult2 INVALID CRC");
    return 0;
  }
}

RoboClaw::TMotorCurrents RoboClaw::getMotorCurrents() {
  return g_sensor_value_group_.motor_currents;
}

RoboClaw::TPIDQ RoboClaw::getPIDQM1() { return g_sensor_value_group_.m1_pidq; }

RoboClaw::TPIDQ RoboClaw::getPIDQM2() { return g_sensor_value_group_.m2_pidq; }

float RoboClaw::getTemperature() { return g_sensor_value_group_.temperature; }

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
  throw new TRoboClawException(
      "[RoboClaw::getUlongCommandResult2] INVALID CRC");
  return 0;
}

uint32_t RoboClaw::getULongCont2(uint16_t &crc) {
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

int32_t RoboClaw::getVelocity(WHICH_VELOCITY whichVelocity) {
  if (whichVelocity == kGETM1SPEED) {
    return g_sensor_value_group_.m1_velocity;
  } else {
    return g_sensor_value_group_.m2_velocity;
  }
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

int32_t RoboClaw::getM1Encoder() {
  return g_sensor_value_group_.m1_encoder_command_result.value;
}

int8_t RoboClaw::getM1EncoderStatus() {
  return g_sensor_value_group_.m1_encoder_command_result.status;
}

int32_t RoboClaw::getM2Encoder() {
  return g_sensor_value_group_.m2_encoder_command_result.value;
}

int8_t RoboClaw::getM2EncoderStatus() {
  return g_sensor_value_group_.m2_encoder_command_result.status;
}

std::string RoboClaw::getVersion() {
  std::string version;
  CmdReadFirmwareVersion command(*this, version);
  command.execute();
  return version;
}

void RoboClaw::openPort() {
  RCUTILS_LOG_INFO("[RoboClaw::openPort] about to open port: %s",
                   device_name_.c_str());
  device_port_ = open(device_name_.c_str(), O_RDWR | O_NOCTTY);
  if (device_port_ < 0) {
    RCUTILS_LOG_ERROR(
        "[RoboClaw::openPort] Unable to open USB port: %s, errno: (%d) "
        "%s",
        device_name_.c_str(), errno, strerror(errno));
    throw new TRoboClawException(
        "[RoboClaw::openPort] Unable to open USB port");
  }

  // Fetch the current port settings.
  struct termios portOptions;
  int ret = 0;

  ret = tcgetattr(device_port_, &portOptions);
  if (ret < 0) {
    RCUTILS_LOG_ERROR("[RoboClaw::openPort] Unable to get terminal options "
                      "(tcgetattr), error: %d: %s",
                      errno, strerror(errno));
    throw new TRoboClawException(
        "[RoboClaw::openPort] Unable to get terminal options (tcgetattr)");
  }

  if (cfsetispeed(&portOptions, B38400) < 0) {
    RCUTILS_LOG_ERROR("[RoboClaw::openPort] Unable to set terminal speed "
                      "(cfsetispeed)");
    throw new TRoboClawException(
        "[RoboClaw::openPort] Unable to set terminal speed "
        "(cfsetispeed)");
  }

  if (cfsetospeed(&portOptions, B38400) < 0) {
    RCUTILS_LOG_ERROR("[RoboClaw::openPort] Unable to set terminal speed "
                      "(cfsetospeed)");
    throw new TRoboClawException(
        "[RoboClaw::openPort] Unable to set terminal speed "
        "(cfsetospeed)");
  }

  // Configure other settings
  portOptions.c_cflag &= ~PARENB;        // Disable parity.
  portOptions.c_cflag &= ~CSTOPB;        // 1 stop bit
  portOptions.c_cflag &= ~CSIZE;         // Clear data size bits
  portOptions.c_cflag |= CS8;            // 8 data bits
  portOptions.c_cflag &= ~CRTSCTS;       // Disable hardware flow control
  portOptions.c_cflag |= CREAD | CLOCAL; // Enable read and ignore control lines

  portOptions.c_lflag &= ~ICANON; // Disable canonical mode
  portOptions.c_lflag &= ~ECHO;   // Disable echo
  portOptions.c_lflag &= ~ECHOE;  // Disable erasure
  portOptions.c_lflag &= ~ECHONL; // Disable new-line echo
  portOptions.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP

  portOptions.c_iflag &=
      ~(IXON | IXOFF | IXANY); // Disable software flow control
  portOptions.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR |
                           ICRNL); // Disable special handling of received bytes

  portOptions.c_oflag &= ~OPOST; // Disable output processing

  portOptions.c_cc[VMIN] = 0;  // Non-blocking read
  portOptions.c_cc[VTIME] = 5; // Timeout of 0.5 seconds

  if (tcsetattr(device_port_, TCSANOW, &portOptions) != 0) {
    RCUTILS_LOG_ERROR("[RoboClaw::openPort] Unable to set terminal options "
                      "(tcsetattr)");
    throw new TRoboClawException(
        "[RoboClaw::openPort] Unable to set terminal options "
        "(tcsetattr)");
  }

  // // c_cflag contains a few important things- CLOCAL and CREAD, to prevent
  // //   this program from "owning" the port and to enable receipt of data.
  // //   Also, it holds the settings for number of data bits, parity, stop
  // bits,
  // //   and hardware flow control.
  // portOptions.c_cflag |=
  //     HUPCL;  // Enable lower control lines on close - hang up.

  // portOptions.c_iflag &= ~(IGNPAR);  // Disable parity checks.
  //                                             // portOptions.c_iflag |=
  //                                             IGNPAR;
  // portOptions.c_iflag &= ~INPCK;              // Disable parity checking.

  // portOptions.c_lflag &= ~IEXTEN;  // Disable input processing
  // portOptions.c_lflag &= ~NOFLSH;  // Disable flushing on SIGINT.

  // portOptions.c_oflag &= ~OFILL;            // Disable fill characters.
  // portOptions.c_oflag &= ~(ONLCR | OCRNL);  // Disable translating NL <-> CR.

  // portOptions.c_cc[VKILL] = 8;
}

uint8_t RoboClaw::readByteWithTimeout2() {
  struct pollfd ufd[1];
  ufd[0].fd = device_port_;
  ufd[0].events = POLLIN;

  int retval = poll(ufd, 1, 11);
  if (retval < 0) {
    RCUTILS_LOG_ERROR("[RoboClaw::readByteWithTimeout2 Poll failed (%d) %s",
                      errno, strerror(errno));
    throw new TRoboClawException("[RoboClaw::readByteWithTimeout2 Read error");
  } else if (retval == 0) {
    std::stringstream ev;
    ev << "[RoboClaw::readByteWithTimeout2 TIMEOUT revents: " << std::hex
       << ufd[0].revents;
    RCUTILS_LOG_ERROR(ev.str().c_str());
    throw new TRoboClawException("[RoboClaw::readByteWithTimeout2 TIMEOUT");
  } else if (ufd[0].revents & POLLERR) {
    RCUTILS_LOG_ERROR("[RoboClaw::readByteWithTimeout2 Error on socket");
    restartPort();
    throw new TRoboClawException(
        "[RoboClaw::readByteWithTimeout2 Error on socket");
  } else if (ufd[0].revents & POLLIN) {
    unsigned char buffer[1];
    ssize_t bytesRead = ::read(device_port_, buffer, sizeof(buffer));
    if (bytesRead != 1) {
      RCUTILS_LOG_ERROR(
          "[RoboClaw::readByteWithTimeout2 Failed to read 1 byte, read: "
          "%d",
          (int)bytesRead);
      throw TRoboClawException(
          "[RoboClaw::readByteWithTimeout2 Failed to read 1 byte");
    }

    static const bool kDEBUG_READBYTE = true;
    if (doLowLevelDebug_) {
      appendToReadLog("%02X ", buffer[0]);
    }
    if (kDEBUG_READBYTE) {
      if ((buffer[0] < 0x21) || (buffer[0] > 0x7F)) {
        // RCUTILS_LOG_INFO("read ..> char: ?? 0x%02X <--", buffer[0]);
        // fprintf(stderr, "..> char: ?? 0x%02X <--", buffer[0]);
      } else {
        // RCUTILS_LOG_INFO("read ..> char: %c  0x%02X <--", buffer[0],
        // buffer[0]); fprintf(stderr, "..> char: %c  0x%02X <--", buffer[0],
        // buffer[0]);
      }
    }

    return buffer[0];
  } else {
    RCUTILS_LOG_ERROR("[RoboClaw::readByteWithTimeout2 Unhandled case");
    throw new TRoboClawException(
        "[RoboClaw::readByteWithTimeout2 Unhandled case");
  }

  return 0;
}

void RoboClaw::readSensorGroup() {
  if (singleton() != nullptr) {
    TPIDQ m1_read_velocity_pidq_result;
    TPIDQ m2_read_velocity_pidq_result;
    CmdReadMotorVelocityPIDQ cmd_m1_read_motor_velocity_pidq(
        *this, kM1, m1_read_velocity_pidq_result);
    cmd_m1_read_motor_velocity_pidq.execute();

    CmdReadMotorVelocityPIDQ cmd_m2_read_motor_velocity_pidq(
        *this, kM2, m2_read_velocity_pidq_result);
    cmd_m2_read_motor_velocity_pidq.execute();

    float logic_battery_level = 0.0;
    CmdReadLogicBatteryVoltage cmd_logic_battery(*this, logic_battery_level);
    cmd_logic_battery.execute();

    float main_battery_level = 0.0;
    CmdReadMainBatteryVoltage cmd_main_battery(*this, main_battery_level);
    cmd_main_battery.execute();

    EncodeResult m1_encoder_command_result;
    EncodeResult m2_encoder_command_result;
    CmdReadEncoder m1_read_encoder_cmd(*this, kM1, m1_encoder_command_result);
    m1_read_encoder_cmd.execute();
    CmdReadEncoder m2_read_encoder_cmd(*this, kM2, m2_encoder_command_result);
    m2_read_encoder_cmd.execute();

    TMotorCurrents motor_currents;
    CmdReadMotorCurrents cmd_read_motor_currents(*this, motor_currents);
    cmd_read_motor_currents.execute();

    int32_t m1_encoder_speed = 0;
    int32_t m2_encoder_speed = 0;
    CmdReadEncoderSpeed cmd_m1_read_encoder_speed(*this, kM1, m1_encoder_speed);
    cmd_m1_read_encoder_speed.execute();
    CmdReadEncoderSpeed cmd_m2_read_encoder_speed(*this, kM2, m2_encoder_speed);
    cmd_m2_read_encoder_speed.execute();

    float temperature = 0.0;
    CmdReadTemperature cmd_read_temperature(*this, temperature);
    cmd_read_temperature.execute();

    unsigned short status = 0;
    CmdReadStatus cmd_read_status(*this, status);
    cmd_read_status.execute();

    g_sensor_value_group_.error_status = status;
    g_sensor_value_group_.error_string = singleton()->getErrorString(status);
    g_sensor_value_group_.logic_battery_level = logic_battery_level;
    g_sensor_value_group_.m1_encoder_command_result = m1_encoder_command_result;
    g_sensor_value_group_.m1_pidq = m1_read_velocity_pidq_result;
    g_sensor_value_group_.m1_velocity = m1_encoder_speed;
    g_sensor_value_group_.m2_encoder_command_result = m2_encoder_command_result;
    g_sensor_value_group_.m2_pidq = m2_read_velocity_pidq_result;
    g_sensor_value_group_.m2_velocity = m2_encoder_speed;
    g_sensor_value_group_.main_battery_level = main_battery_level;

    // Call getMotorCurrents before getMotorAlarms;
    g_sensor_value_group_.motor_currents = motor_currents;
    g_sensor_value_group_.motor_alarms = singleton()->getMotorAlarms();

    g_sensor_value_group_.temperature = temperature;
    g_sensor_value_group_.last_sensor_read_time_ =
        std::chrono::system_clock::now();
  }
}

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

void RoboClaw::restartPort() {
  close(device_port_);
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
  CmdDoBufferedM2M2DriveSpeedAccelDistance stopCommand(*this, 0, 0, 0, 0, 0);
  stopCommand.execute();
}

void RoboClaw::updateCrc(uint16_t &crc, uint8_t data) {
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
    result = ::write(device_port_, &byte, 1);
    // RCUTILS_LOG_INFO("--> wrote: 0x%02X, result: %ld", byte, result);  //
    // ####
    if (doLowLevelDebug_) {
      if (result == 1) {
        appendToWriteLog("%02X ", byte);
      } else {
        appendToWriteLog(">%02X< ", byte);
      }
    }

  } while (result == -1 && errno == EAGAIN);

  if (result != 1) {
    RCUTILS_LOG_ERROR("[RoboClaw::writeByte2to write one byte, result: %d, "
                      "errno: %d)",
                      (int)result, errno);
    restartPort();
    throw new TRoboClawException(
        "[RoboClaw::writeByte2 Unable to write one byte");
  }
}

void RoboClaw::writeN2(bool sendCRC, uint8_t cnt, ...) {
  uint16_t crc = 0;
  va_list marker;
  va_start(marker, cnt);

  // int origFlags = fcntl(device_port_, F_GETFL, 0);
  // fcntl(device_port_, F_SETFL, origFlags & ~O_NONBLOCK);

  for (uint8_t i = 0; i < cnt; i++) {
    uint8_t byte = va_arg(marker, int);
    updateCrc(crc, byte);
    writeByte2(byte);
  }

  va_end(marker);

  if (sendCRC) {
    writeByte2(crc >> 8);
    writeByte2(crc);

    uint8_t response = readByteWithTimeout2();
    if (response != 0xFF) {
      char msg[128];
      snprintf(
          msg, sizeof(msg),
          "[RoboClaw::writeN2] Invalid ACK response, expected 0xFF but got "
          "0x%02X",
          response);
      RCUTILS_LOG_ERROR("%s", msg);
      throw new TRoboClawException(msg);
    }
  }

  // fcntl(device_port_, F_SETFL, origFlags);
}

RoboClaw *RoboClaw::singleton() { return g_singleton; }

RoboClaw *RoboClaw::g_singleton = nullptr;