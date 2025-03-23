#pragma once

#include <rcutils/logging_macros.h>
#include <sys/types.h>

#include <chrono>
#include <cstdarg>  // Include cstdarg for va_list
#include <cstdint>
#include <mutex>  // Include mutex header
#include <rclcpp/logger.hpp>
#include <sstream>
#include <string>

#include "ros2_roboclaw_driver/srv/reset_encoders.hpp"

/* The expected client is motor_driver.cpp
 * The expected node is motor_driver_node.cpp
 *
 * This class provides a C++ interface to the RoboClaw motor controller.
 * It handles communication with the device and provides methods for
 * controlling the motors, reading encoder values, and getting status
 * information.
 *
 * The RoboClaw motor controller is used in various robotics applications,
 * and this class abstracts the low-level details of communication with the
 * device.
 */

#define SetDWORDval(arg) \
  (uint8_t)(arg >> 24), (uint8_t)(arg >> 16), (uint8_t)(arg >> 8), (uint8_t)arg

class RoboClaw {
 public:
  enum kMotor { kM1 = 0, kM2 = 1, kNone = 2 };

  // Bit positions used to build alarms.
  enum {
    kM1_OVER_CURRENT = 0x01,        // Motor 1 current sense is too high.
    kM2_OVER_CURRENT = 0x02,        // Motor 2 current sense is too high.
    kM1_OVER_CURRENT_ALARM = 0x04,  // Motor 1 controller over current alarm.
    kM2_OVER_CURRENT_ALARM = 0x08,  // Motor 2 controller over current alarm.
  };

  // A convenience struction to pass around configuration information.
  typedef struct {
    float p;
    float i;
    float d;
    uint32_t qpps;
    float max_current;
  } TPIDQ;

  // For a custom exception message.
  struct TRoboClawException : public std::exception {
    std::string s;
    TRoboClawException(const char *format, ...) {
      char buffer[256];
      va_list args;
      va_start(args, format);
      vsnprintf(buffer, sizeof(buffer), format, args);
      va_end(args);
      s = std::string(buffer);
    }
    ~TRoboClawException() throw() {}
    const char *what() const throw() { return s.c_str(); }
  };

  // Holds RoboClaw encoder result.
  typedef struct {
    int32_t value;
    uint8_t status;
  } EncodeResult;

  // Constructor.
  RoboClaw(const TPIDQ m1Pid, const TPIDQ m2Pid, float m1MaxCurrent,
           float m2MaxCurrent, std::string device_name, uint8_t device_port,
           uint32_t baud_rate, bool do_debug = false,
           bool do_low_level_debug = false);

  ~RoboClaw();

  void appendToReadLog(const char *format, ...) {
    va_list args;
    va_start(args, format);
    debug_log_.appendToReadLog(format, args);
    va_end(args);
  }

  void appendToWriteLog(const char *format, ...) {
    va_list args;
    va_start(args, format);
    debug_log_.appendToWriteLog(format, args);
    va_end(args);
  }

  void doMixedSpeedAccelDist(uint32_t accel_quad_pulses_per_second,
                             int32_t m1_quad_pulses_per_second,
                             uint32_t m1_max_distance,
                             int32_t m2_quad_pulses_per_second,
                             uint32_t m2_max_distance);

  // Get RoboClaw error status bits.
  uint16_t getErrorStatus();

  // Get RoboClaw error status as a string.
  std::string getErrorString();

  // Get logical battery voltage level.
  // Note: This is the voltage level of the battery powering the logic
  float getLogicBatteryLevel();

  // Get main battery voltage level.
  // Note: This is the voltage level of the battery powering the motors.
  float getMainBatteryLevel();

  // Get the encoder value for motor 1.
  int32_t getM1Encoder();

  // Get the status of the encoder for motor 1.
  // 0 = no error, 1 = encoder command error, 2 = encoder not found.
  int8_t getM1EncoderStatus();

  // Get the encoder value for motor 2.
  int32_t getM2Encoder();

  // Get the status of the encoder for motor 2.
  // 0 = no error, 1 = encoder command error, 2 = encoder not found.
  int8_t getM2EncoderStatus();

  // Convenience structure to  hold a pair of current values.
  typedef struct {
    float m1Current;
    float m2Current;
  } TMotorCurrents;

  // Make sure you call getMotorCurrents before getMotorAlarms.
  int getMotorAlarms() { return motorAlarms_; }

  // Get the value of currents flowig  into each motors.
  TMotorCurrents getMotorCurrents();

  // Get the PIDQ values for motor 1.
  // PIDQ = {p, i, d, qpps}
  TPIDQ getPIDQM1();

  // Get the PIDQ values for motor 2.
  // PIDQ = {p, i, d, qpps}
  TPIDQ getPIDQM2();

  // Get the temperature of the RoboClaw.
  // Note: This is the temperature of the RoboClaw controller board itself.
  float getTemperature();

  // Get velocity (speed) of a motor.
  int32_t getVelocity(kMotor motor);

  // Get RoboClaw software version.
  std::string getVersion();

  // Stop motion.
  void stop();

  // Get singleton instance of class.
  static RoboClaw *singleton();

  // Read a group of sensors from the RoboClaw.
  void readSensorGroup();

 protected:
  // Write a stream of bytes to the device.
  void writeN2(bool sendCRC, uint8_t cnt, ...);

 private:
  // True => print debug messages.
  bool do_debug_;

  // True => print byte values as they are read and written.
  bool do_low_level_debug_;

  // Get RoboClaw error status as a string.
  std::string getErrorString(uint16_t errorStatus);

  // Various values are periodically read from the RoboClaw as a group and
  // stored in this structure. Clients wanting these values will get the latest,
  // cached results from this structure rather than the current values from the
  // RoboClaw. This is to avoid excessive RoboClaw reads which can cause
  // timeouts and other issues. The values are read from the RoboClaw in the
  // readSensorGroup() method. The values are updated in the readSensorGroup()
  // method and can be accessed by clients using the g_sensor_value_group_
  // structure.
  typedef struct {
    uint16_t error_status;
    std::string error_string;
    float logic_battery_level;
    EncodeResult m1_encoder_command_result;
    TPIDQ m1_pidq;
    int32_t m1_velocity;
    EncodeResult m2_encoder_command_result;
    TPIDQ m2_pidq;
    int32_t m2_velocity;
    float main_battery_level;
    int motor_alarms;
    TMotorCurrents motor_currents;
    float temperature;
    std::chrono::system_clock::time_point last_sensor_read_time_;
  } SensorValueGroup;

  // This structure holds the latest values read from the RoboClaw.
  SensorValueGroup g_sensor_value_group_;

  // Enum values without a 'k' prefix have not been used in code yet.
  typedef enum ROBOCLAW_COMMAND {
    M1FORWARD = 0,
    M1BACKWARD = 1,
    SETMINMB = 2,
    SETMAXMB = 3,
    M2FORWARD = 4,
    M2BACKWARD = 5,
    M17BIT = 6,
    M27BIT = 7,
    MIXEDFORWARD = 8,
    MIXEDBACKWARD = 9,
    MIXEDRIGHT = 10,
    MIXEDLEFT = 11,
    MIXEDFB = 12,
    MIXEDLR = 13,
    RESETENC = 20,
    GETVERSION = 21,
    SETM1ENCODER = 22,
    SETM2ENCODER = 23,
    GETMBATT = 24,
    GETLBATT = 25,
    SETMINLB = 26,
    SETMAXLB = 27,
    SETM1PID = 28,
    SETM2PID = 29,
    GETM1ISPEED = 30,
    GETM2ISPEED = 31,
    M1DUTY = 32,
    M2DUTY = 33,
    kMIXEDDUTY = 34,
    M1SPEED = 35,
    M2SPEED = 36,
    kMIXEDSPEED = 37,
    M1SPEEDACCEL = 38,
    M2SPEEDACCEL = 39,
    MIXEDSPEEDACCEL = 40,
    M1SPEEDDIST = 41,
    M2SPEEDDIST = 42,
    kMIXEDSPEEDDIST = 43,
    M1SPEEDACCELDIST = 44,
    M2SPEEDACCELDIST = 45,
    MIXEDSPEEDACCELDIST = 46,
    GETBUFFERS = 47,
    SETPWM = 48,
    GETCURRENTS = 49,
    MIXEDSPEED2ACCEL = 50,
    MIXEDSPEED2ACCELDIST = 51,
    M1DUTYACCEL = 52,
    M2DUTYACCEL = 53,
    MIXEDDUTYACCEL = 54,
    GETTEMPERATURE = 82,
    GETERROR = 90,
    WRITENVM = 94,
    GETM1MAXCURRENT = 135
  } ROBOCLAW_COMMAND;

  int baud_rate_;            // Baud rate for RoboClaw connection.
  int device_port_;          // Unix file descriptor for RoboClaw connection.
  int maxCommandRetries_;    // Maximum number of times to retry a RoboClaw
                             // command.
  float maxM1Current_;       // Maximum allowed M1 current.
  float maxM2Current_;       // Maximum allowed M2 current.
  int motorAlarms_;          // Motors alarms. Bit-wise OR of contributors.
  std::string device_name_;  // Device name of RoboClaw device.
  int portAddress_;          // Port number of RoboClaw device under control

  // Get velocity (speed) result from the RoboClaw controller.
  int32_t getVelocityResult(uint8_t command);

  unsigned long getUlongCommandResult2(uint8_t command);

  uint32_t getULongCont2(uint16_t &crc);

  unsigned short get2ByteCommandResult2(uint8_t command);

  // Open the RoboClaw USB port.
  void openPort();

  // Read one byte from device with timeout.
  uint8_t readByteWithTimeout2();

  // Perform error recovery to re-open a failed device port.
  void restartPort();

  // Reset the encoders.
  bool resetEncoders(
      ros2_roboclaw_driver::srv::ResetEncoders::Request &request,
      ros2_roboclaw_driver::srv::ResetEncoders::Response &response);

  // Set the PID for motor M1.
  void setM1PID(float p, float i, float d, uint32_t qpps);

  // Set the PID for motor M1.
  void setM2PID(float p, float i, float d, uint32_t qpps);

  // Update the running CRC result.
  void updateCrc(uint16_t &crc, uint8_t data);

  // Write one byte to the device.
  void writeByte2(uint8_t byte);

  static RoboClaw *g_singleton;

  class DebugLog {
   public:
    DebugLog(RoboClaw *roboclaw)
        : roboclaw_(roboclaw),
          next_read_log_index_(0),
          next_write_log_index_(0) {}
    ~DebugLog() {}

    void appendToReadLog(const char *format, va_list args) {
      if (roboclaw_->do_debug_) {
        int written =
            vsnprintf(&read_log_[next_read_log_index_],
                      sizeof(read_log_) - next_read_log_index_, format, args);
        if (written > 0) {
          next_read_log_index_ += written;
        }
      }
    }

    void appendToWriteLog(const char *format, va_list args) {
      if (roboclaw_->do_debug_) {
        int written =
            vsnprintf(&write_log_[next_write_log_index_],
                      sizeof(write_log_) - next_write_log_index_, format, args);
        if (written > 0) {
          next_write_log_index_ += written;
        }
        if ((next_write_log_index_ > 0) && (write_log_[0] == '8')) {
          RCUTILS_LOG_INFO("Whoops!");
        }
      }
    }

    void showLog() {
      if (roboclaw_->do_debug_) {
        RCUTILS_LOG_INFO("[RoboClaw::DebugLog] %s, READ: %s", write_log_,
                         read_log_);
        read_log_[0] = '\0';
        next_read_log_index_ = 0;
        write_log_[0] = '\0';
        next_write_log_index_ = 0;
      }
    }

    // private:
    RoboClaw *roboclaw_;  // Pointer to the RoboClaw instance (if needed)
    char read_log_[256];
    char write_log_[256];
    uint16_t next_read_log_index_;
    uint16_t next_write_log_index_;
  };

  friend class Cmd;
  friend class CmdDoBufferedM1M2DriveSpeedAccelDistance;
  friend class CmdReadEncoderSpeed;
  friend class CmdReadEncoder;
  friend class CmdReadFirmwareVersion;
  friend class CmdReadLogicBatteryVoltage;
  friend class CmdReadMainBatteryVoltage;
  friend class CmdReadMotorCurrents;
  friend class CmdReadMotorVelocityPIDQ;
  friend class CmdReadStatus;
  friend class CmdReadTemperature;
  friend class CmdSetEncoderValue;
  friend class CmdSetPid;

 protected:
  DebugLog debug_log_;

  static const char *motorNames_[];
  static std::mutex
      buffered_command_mutex_;  // Global mutex for buffered commands
};
