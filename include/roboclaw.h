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

/* There are possibly several clients that want to read sensor data,
   such as encoder values, motor speeds, etc. Each time an actual read
   from the RoboClaw device is attempted, it potentially conflicts with
   motor commands. And it's possible that clients want sensor data more
   often than reasonably expected.

   As a result, GetXxx methods below that provide sensor data values are
   broken up into a GetXxx and cache_GetXxx methods. The GetXxx methods
   just return that last reading fetched from a periodic loop. The
   cache_GetXxx methods do the real command request to the RoboClaw and
   are only called during the class constructor and a periodic, background
   thread.
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

  // Referencing which encoder in the RoboClaw
  typedef enum WHICH_ENC {
    kGETM1ENC = 16,
    kGETM2ENC = 17,

  } WHICH_ENC;

  // Referencing which motor in the RoboClaw
  typedef enum WHICH_MOTOR {
    kGETM1PID = 55,
    kGETM2PID = 56,
  } WHICH_MOTOR;

  // Referencing which velocity in the RoboClaw
  typedef enum WHICH_VELOCITY {
    kGETM1SPEED = 18,
    kGETM2SPEED = 19,
  } WHICH_VELOCITY;

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
           uint32_t baud_rate);

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

  // void doMixedSpeedDist(int32_t m1_quad_pulses_per_second,
  //                       int32_t m1_max_distance,
  //                       int32_t m2_quad_pulses_per_second,
  //                       int32_t m2_max_distance);

  void doMixedSpeedAccelDist(uint32_t accel_quad_pulses_per_second,
                             int32_t m1_quad_pulses_per_second,
                             uint32_t m1_max_distance,
                             int32_t m2_quad_pulses_per_second,
                             uint32_t m2_max_distance);

  // EncodeResult getEncoderCommandResult(WHICH_ENC command);

  // Get RoboClaw error status bits.
  uint16_t getErrorStatus();

  // Get RoboClaw error status as a string.
  std::string getErrorString();

  float getLogicBatteryLevel();

  float getMainBatteryLevel();

  // Get the encoder value for motor 1.
  int32_t getM1Encoder();

  int8_t getM1EncoderStatus();

  // Get the encoder value for motor 2.
  int32_t getM2Encoder();

  int8_t getM2EncoderStatus();

  // Convenience structure to  hold a pair of current values.
  typedef struct {
    float m1Current;
    float m2Current;
  } TMotorCurrents;

  // Make sure you call getMotorCurrents before getMotorAlarms.
  int getMotorAlarms() { return motorAlarms_; }

  TMotorCurrents getMotorCurrents();

  TPIDQ getPIDQM1();
  TPIDQ getPIDQM2();

  float getTemperature();

  // Get velocity (speed) of a motor.
  int32_t getVelocity(WHICH_VELOCITY whichVelocity);

  // Get RoboClaw software versions.
  std::string getVersion();

  // Stop motion.
  void stop();

  // Get singleton instance of class.
  static RoboClaw *singleton();

  void readSensorGroup();

 protected:
  // Write a stream of bytes to the device.
  void writeN2(bool sendCRC, uint8_t cnt, ...);

 private:
  // True => print debug messages.
  bool doDebug_;

  // True => print byte values as they are read and written.
  bool doLowLevelDebug_;

  // Get RoboClaw error status as a string.
  std::string getErrorString(uint16_t errorStatus);

  static void sensorReadThread();

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

  SensorValueGroup g_sensor_value_group_;

  typedef struct {
    unsigned long p1;
    unsigned long p2;
  } ULongPair;

  enum {
    kERROR_NORMAL = 0x00,
    kM1OVERCURRENT = 0x01,
    kM2OVERCURRENT = 0x02,
    kESTOP = 0x04,
    kTEMPERATURE = 0x08,
    kMAINBATTERYHIGH = 0x10,
    kMAINBATTERYLOW = 0x20,
    kLOGICBATTERYHIGH = 0x40,
    kLOGICBATTERYLOW = 0x80
  };

  // Enum values without a 'k' prefix have not been used in code.
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
    kGETVERSION = 21,
    kSETM1ENCODER = 22,
    kSETM2ENCODER = 23,
    kGETMBATT = 24,
    kGETLBATT = 25,
    SETMINLB = 26,
    SETMAXLB = 27,
    kSETM1PID = 28,
    kSETM2PID = 29,
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
    kMIXEDSPEEDACCELDIST = 46,
    GETBUFFERS = 47,
    SETPWM = 48,
    kGETCURRENTS = 49,
    MIXEDSPEED2ACCEL = 50,
    MIXEDSPEED2ACCELDIST = 51,
    M1DUTYACCEL = 52,
    M2DUTYACCEL = 53,
    MIXEDDUTYACCEL = 54,
    kGETTEMPERATURE = 82,
    kGETERROR = 90,
    WRITENVM = 94,
    GETM1MAXCURRENT = 135
  } ROBOCLAW_COMMAND;

  int baud_rate_;    // Baud rate for RoboClaw connection.
  int device_port_;  // Unix file descriptor for RoboClaw connection.
  float m1p_;
  float m1i_;
  float m1d_;
  int m1qpps_;
  float m2p_;
  float m2i_;
  float m2d_;
  int m2qpps_;
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

  char command_log_[256];
  char response_log_[256];

  class Cmd {
   public:
    void execute() {
      for (int retry = 0; retry < 3 /*### maxCommandRetries_*/; retry++) {
        try {
          std::lock_guard<std::mutex> lock(
              RoboClaw::buffered_command_mutex_);  // Lock the mutex
          send();
          roboclaw_.debug_log_.showLog();
          return;
        } catch (TRoboClawException *e) {
          roboclaw_.debug_log_.showLog();
          RCUTILS_LOG_ERROR(
              "[RoboClaw::Cmd::execute] Exception: %s, retry number: %d",
              e->what(), retry);
        } catch (...) {
          roboclaw_.debug_log_.showLog();
          RCUTILS_LOG_ERROR("[RoboClaw::Cmd::execute] Uncaught exception !!!");
        }
      }

      roboclaw_.debug_log_.showLog();
      RCUTILS_LOG_ERROR("[RoboClaw::Cmd::execute] RETRY COUNT EXCEEDED");
      throw new TRoboClawException(
          "[RoboClaw::Cmd::execute] RETRY COUNT EXCEEDED");
    }

    virtual void send() = 0;  // Declare send as a pure virtual function

   protected:
    Cmd(RoboClaw &roboclaw, const char *name, const kMotor motor)
        : motor_(motor), roboclaw_(roboclaw) {
      strncpy(name_, name, sizeof(name_));
      name_[sizeof(name_) - 1] = '\0';  // Ensure null-termination
    }

    kMotor motor_;
    RoboClaw &roboclaw_;
    char name_[32];

   private:
    Cmd() = delete;  // Disallow default constructor
  };

  class DebugLog {
   public:
    DebugLog() : next_read_log_index_(0), next_write_log_index_(0) {}
    ~DebugLog() {}

    void appendToReadLog(const char *format, va_list args) {
      int written =
          vsnprintf(&read_log_[next_read_log_index_],
                    sizeof(read_log_) - next_read_log_index_, format, args);
      if (written > 0) {
        next_read_log_index_ += written;
      }
    }

    void appendToWriteLog(const char *format, va_list args) {
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

    void showLog() {
      RCUTILS_LOG_INFO("[RoboClaw::DebugLog] %s, READ: %s", write_log_,
                       read_log_);
      read_log_[0] = '\0';
      next_read_log_index_ = 0;
      write_log_[0] = '\0';
      next_write_log_index_ = 0;
    }

    // private:
    char read_log_[256];
    char write_log_[256];
    uint16_t next_read_log_index_;
    uint16_t next_write_log_index_;
  };

  class CmdSetPid : public Cmd {
   public:
    CmdSetPid(RoboClaw &roboclaw, kMotor motor, float p, float i, float d,
              uint32_t qpps)
        : Cmd(roboclaw, "SetPid", motor), p_(p), i_(i), d_(d), qpps_(qpps) {}

    void send() override {
      roboclaw_.appendToWriteLog(
          "SetPid: motor: %d (%s) p: %f, i: %f, d: %f, "
          "qpps: %d, WROTE: ",
          motor_, motorNames_[motor_], p_, i_, d_, qpps_);
      uint32_t kp = int(p_ * 65536.0);
      uint32_t ki = int(i_ * 65536.0);
      uint32_t kd = int(d_ * 65536.0);
      roboclaw_.writeN2(true, 18, roboclaw_.portAddress_,
                        motor_ == kM1 ? kSETM1PID : kSETM2PID, SetDWORDval(kd),
                        SetDWORDval(kp), SetDWORDval(ki), SetDWORDval(qpps_));
    }

    float p_;
    float i_;
    float d_;
    uint32_t qpps_;
  };

  class CmdReadFirmwareVersion : public Cmd {
   public:
    CmdReadFirmwareVersion(RoboClaw &roboclaw, std::string &version)
        : Cmd(roboclaw, "ReadFirmwareVersion", kNone), version_(version) {}

    void send() override {
      roboclaw_.appendToWriteLog("ReadFirmwareVersion: WROTE: ");
      roboclaw_.writeN2(false, 2, roboclaw_.portAddress_, kGETVERSION);

      try {
        uint16_t crc = 0;
        uint8_t i;
        uint8_t datum;
        std::stringstream version;

        roboclaw_.updateCrc(crc, roboclaw_.portAddress_);
        roboclaw_.updateCrc(crc, kGETVERSION);
        for (i = 0; i < 48; i++) {
          datum = roboclaw_.readByteWithTimeout2();
          roboclaw_.updateCrc(crc, datum);
          if (datum == 0) {
            uint16_t responseCrc = 0;
            datum = roboclaw_.readByteWithTimeout2();
            responseCrc = datum << 8;
            datum = roboclaw_.readByteWithTimeout2();
            responseCrc |= datum;
            if (responseCrc == crc) {
              version_ = version.str();
              roboclaw_.appendToReadLog(", RESULT: '%s'", version_.c_str());
              return;
            } else {
              RCUTILS_LOG_ERROR(
                  "[RoboClaw::CmdReadFirmwareVersion] invalid "
                  "CRC expected: 0x%02X, "
                  "got: 0x%02X",
                  crc, responseCrc);
              throw new TRoboClawException(
                  "[RoboClaw::getVersion] Invalid CRC, expected 0x%02X, "
                  "got 0x%02X",
                  crc, responseCrc);
            }
          } else {
            version << (char)datum;
          }
        }

        RCUTILS_LOG_ERROR(
            "[RoboClaw::CmdReadFirmwareVersion] unexpected long string");
        throw new TRoboClawException(
            "[RoboClaw::getVersion] unexpected long string");
      } catch (...) {
        RCUTILS_LOG_ERROR(
            "[RoboClaw::CmdReadFirmwareVersion] Uncaught exception !!!");
      }
      RCUTILS_LOG_ERROR(
          "[RoboClaw::CmdReadFirmwareVersion] RETRY COUNT EXCEEDED");
    }

    std::string &version_;
  };

  class CmdSetEncoderValue : public Cmd {
   public:
    CmdSetEncoderValue(RoboClaw &roboclaw, kMotor motor, long value)
        : Cmd(roboclaw, "SetEncoderValue", motor), value_(value) {}

    void send() override {
      roboclaw_.appendToWriteLog(
          "SetEncoderValue: motor: %d (%s) value: %ld, WROTE: ", motor_,
          motorNames_[motor_], value_);
      try {
        roboclaw_.writeN2(true, 6, roboclaw_.portAddress_,
                          motor_ == kM1 ? kSETM1ENCODER : kSETM2ENCODER,
                          SetDWORDval(value_));
        return;
      } catch (...) {
        RCUTILS_LOG_ERROR(
            "[RoboClaw::CmdSetEncoderValue] Uncaught exception !!!");
      }
    }

    long value_;
  };

  class CmdReadStatus : public Cmd {
   public:
    CmdReadStatus(RoboClaw &roboclaw, uint16_t &status)
        : Cmd(roboclaw, "ReadStatus", kNone), status_(status) {}
    void send() override {
      try {
        uint16_t crc = 0;
        roboclaw_.updateCrc(crc, roboclaw_.portAddress_);
        roboclaw_.updateCrc(crc, kGETERROR);
        roboclaw_.appendToWriteLog("ReadStatus: WROTE: ");
        roboclaw_.writeN2(false, 2, roboclaw_.portAddress_, kGETERROR);
        status_ = (unsigned short)roboclaw_.getULongCont2(crc);
        uint16_t responseCrc = 0;
        uint16_t datum = roboclaw_.readByteWithTimeout2();
        responseCrc = datum << 8;
        datum = roboclaw_.readByteWithTimeout2();
        responseCrc |= datum;
        if (responseCrc == crc) {
          roboclaw_.appendToReadLog(", RESULT: %04X", status_);
          return;
        } else {
          RCUTILS_LOG_ERROR(
              "[RoboClaw::CmdReadStatus] invalid CRC expected: 0x%02X, "
              "got: "
              "0x%02X",
              crc, responseCrc);
        }
      } catch (...) {
        RCUTILS_LOG_ERROR("[RoboClaw::CmdReadStatus] Uncaught exception !!!");
      }
    };

    uint16_t &status_;
  };

  class CmdReadLogicBatteryVoltage : public Cmd {
   public:
    CmdReadLogicBatteryVoltage(RoboClaw &roboclaw, float &voltage)
        : Cmd(roboclaw, "ReadLogicBatteryVoltage", kNone), voltage_(voltage) {}
    void send() override {
      try {
        roboclaw_.appendToWriteLog("ReadLogicBatteryVoltage: WROTE: ");
        float result =
            ((float)roboclaw_.get2ByteCommandResult2(kGETLBATT)) / 10.0;
        voltage_ = result;
        roboclaw_.appendToReadLog(", RESULT: %f", result);
        return;
      } catch (...) {
        RCUTILS_LOG_ERROR(
            "[RoboClaw::CmdReadLogicBatteryVoltage] Uncaught exception !!!");
      }
    }

    float &voltage_;
  };

  class CmdReadMainBatteryVoltage : public Cmd {
   public:
    CmdReadMainBatteryVoltage(RoboClaw &roboclaw, float &voltage)
        : Cmd(roboclaw, "ReadLMainBatteryVoltage", kNone), voltage_(voltage) {}
    void send() override {
      try {
        roboclaw_.appendToWriteLog("ReadLMainBatteryVoltage: WROTE: ");
        float result =
            ((float)roboclaw_.get2ByteCommandResult2(kGETMBATT)) / 10.0;
        voltage_ = result;
        roboclaw_.appendToReadLog(", RESULT: %f", result);
        return;
      } catch (...) {
        RCUTILS_LOG_ERROR(
            "[RoboClaw::CmdReadMainBatteryVoltage] Uncaught exception !!!");
      }
    }

    float &voltage_;
  };

  class CmdReadEncoder : public Cmd {
   public:
    CmdReadEncoder(RoboClaw &roboclaw, kMotor motor, EncodeResult &encoder)
        : Cmd(roboclaw, "ReadEncoder", motor), encoder_(encoder) {}
    void send() override {
      try {
        roboclaw_.appendToWriteLog("ReadEncoder: encoder: %d (%s), WROTE: ",
                                   motor_, motor_ == kM1 ? "M1" : "M2");

        uint16_t crc = 0;
        roboclaw_.updateCrc(crc, roboclaw_.portAddress_);
        roboclaw_.updateCrc(crc, motor_ == kM1 ? kGETM1ENC : kGETM2ENC);

        roboclaw_.writeN2(false, 2, roboclaw_.portAddress_,
                          motor_ == kM1 ? kGETM1ENC : kGETM2ENC);

        uint8_t datum = roboclaw_.readByteWithTimeout2();
        encoder_.value |= datum << 24;
        roboclaw_.updateCrc(crc, datum);

        datum = roboclaw_.readByteWithTimeout2();
        encoder_.value |= datum << 16;
        roboclaw_.updateCrc(crc, datum);

        datum = roboclaw_.readByteWithTimeout2();
        encoder_.value |= datum << 8;
        roboclaw_.updateCrc(crc, datum);

        datum = roboclaw_.readByteWithTimeout2();
        encoder_.value |= datum;
        roboclaw_.updateCrc(crc, datum);

        datum = roboclaw_.readByteWithTimeout2();
        encoder_.status |= datum;
        roboclaw_.updateCrc(crc, datum);

        uint16_t responseCrc = 0;
        datum = roboclaw_.readByteWithTimeout2();
        responseCrc = datum << 8;
        datum = roboclaw_.readByteWithTimeout2();
        responseCrc |= datum;
        if (responseCrc != crc) {
          RCUTILS_LOG_ERROR(
              "[RoboClaw::CmdReadEncoder] Expected "
              "CRC of: 0x%02X, but "
              "got: 0x%02X",
              int(crc), int(responseCrc));
          throw new TRoboClawException(
              "[RoboClaw::CmdReadEncoder] INVALID CRC");
        }

        roboclaw_.appendToReadLog(", RESULT value: %d, status: %d",
                                  encoder_.value, encoder_.status);
        return;
      } catch (...) {
        RCUTILS_LOG_ERROR("[RoboClaw::CmdReadEncoder] Uncaught exception !!!");
      }
    }

    EncodeResult &encoder_;
  };

  class CmdReadMotorVelocityPIDQ : public Cmd {
   public:
    CmdReadMotorVelocityPIDQ(RoboClaw &roboclaw, kMotor motor, TPIDQ &pidq)
        : Cmd(roboclaw, "ReadMotorVelocityPIDQ", motor), pidq_(pidq) {}
    void send() override {
      try {
        roboclaw_.appendToWriteLog(
            "ReadMotorVelocityPIDQ: motor: %d (%s), WROTE: ", motor_,
            motor_ == kM1 ? "M1" : "M2");
        TPIDQ result;
        uint16_t crc = 0;
        roboclaw_.updateCrc(crc, roboclaw_.portAddress_);
        roboclaw_.updateCrc(crc, motor_ == kM1 ? kGETM1PID : kGETM2PID);

        roboclaw_.writeN2(false, 2, roboclaw_.portAddress_,
                          motor_ == kM1 ? kGETM1PID : kGETM2PID);
        result.p = (int32_t)roboclaw_.getULongCont2(crc) / 65536.0;
        result.i = (int32_t)roboclaw_.getULongCont2(crc) / 65536.0;
        result.d = (int32_t)roboclaw_.getULongCont2(crc) / 65536.0;
        result.qpps = (int32_t)roboclaw_.getULongCont2(crc);

        uint16_t responseCrc = 0;
        uint16_t datum = roboclaw_.readByteWithTimeout2();
        responseCrc = datum << 8;
        datum = roboclaw_.readByteWithTimeout2();
        responseCrc |= datum;
        roboclaw_.appendToReadLog(
            ", RESULT: p: %3.4f, i: %3.4f, d: %3.4f, qpps: %d", result.p,
            result.i, result.d, result.qpps);
        if (responseCrc == crc) {
          pidq_ = result;
          return;
        } else {
          RCUTILS_LOG_ERROR(
              "[RoboClaw::CmdReadMotorVelocityPIDQ] invalid CRC "
              "expected: 0x%2X, got: "
              "0x%2X",
              crc, responseCrc);
        }
      } catch (...) {
        RCUTILS_LOG_ERROR(
            "[RoboClaw::CmdReadMotorVelocityPIDQ] Uncaught exception !!!");
      }
    }

    TPIDQ &pidq_;
  };

  class CmdReadEncoderSpeed : public Cmd {
   public:
    CmdReadEncoderSpeed(RoboClaw &roboclaw, kMotor motor, int32_t &speed)
        : Cmd(roboclaw, "ReadEncoderSpeed", motor), speed_(speed) {}
    void send() override {
      try {
        roboclaw_.appendToWriteLog("ReadEncoderSpeed: motor: %d (%s), WROTE: ",
                                   motor_, motor_ == kM1 ? "M1" : "M2");
        speed_ = roboclaw_.getVelocityResult(motor_ == kM1 ? kGETM1SPEED
                                                           : kGETM2SPEED);
        roboclaw_.appendToReadLog(", RESULT: %d", speed_);
        return;
      } catch (...) {
        RCUTILS_LOG_ERROR(
            "[RoboClaw::CmdReadEncoderSpeed] Uncaught exception "
            "!!!");
      }
    }

    int32_t &speed_;
  };

  class CmdReadMotorCurrents : public Cmd {
   public:
    CmdReadMotorCurrents(RoboClaw &roboclaw, TMotorCurrents &motorCurrents)
        : Cmd(roboclaw, "ReadMotorCurrents", kNone),
          motorCurrents_(motorCurrents) {}
    void send() override {
      try {
        roboclaw_.appendToWriteLog("ReadMotorCurrents: WROTE: ");

        unsigned long currentPair =
            roboclaw_.getUlongCommandResult2(kGETCURRENTS);
        motorCurrents_.m1Current = ((int16_t)(currentPair >> 16)) * 0.010;
        motorCurrents_.m2Current = ((int16_t)(currentPair & 0xFFFF)) * 0.010;
        roboclaw_.appendToReadLog(
            ", RESULT m1 current: %3.4f, m2 current: %3.4f",
            motorCurrents_.m1Current, motorCurrents_.m2Current);

        if (motorCurrents_.m1Current > roboclaw_.maxM1Current_) {
          roboclaw_.motorAlarms_ |= kM1_OVER_CURRENT_ALARM;
          RCUTILS_LOG_ERROR(
              "[RoboClaw::CmdReadMotorCurrents] Motor 1 over current. Max "
              "allowed: %6.3f, found: %6.3f",
              roboclaw_.maxM1Current_, motorCurrents_.m1Current);
          roboclaw_.stop();
        } else {
          roboclaw_.motorAlarms_ &= ~kM1_OVER_CURRENT_ALARM;
        }

        if (motorCurrents_.m2Current > roboclaw_.maxM2Current_) {
          roboclaw_.motorAlarms_ |= kM2_OVER_CURRENT_ALARM;
          RCUTILS_LOG_ERROR(
              "[RoboClaw::CmdReadMotorCurrents] Motor 2 over current. Max "
              "allowed: %6.3f, found: %6.3f",
              roboclaw_.maxM2Current_, motorCurrents_.m2Current);
          roboclaw_.stop();
        } else {
          roboclaw_.motorAlarms_ &= ~kM2_OVER_CURRENT_ALARM;
        }
      } catch (...) {
        RCUTILS_LOG_ERROR(
            "[RoboClaw::CmdReadMotorCurrents] Uncaught exception !!!");
      }
    }

    TMotorCurrents &motorCurrents_;
  };

  class CmdReadTemperature : public Cmd {
   public:
    CmdReadTemperature(RoboClaw &roboclaw, float &temperature)
        : Cmd(roboclaw, "ReadTemperature", kNone), temperature_(temperature) {}
    void send() override {
      try {
        roboclaw_.appendToWriteLog("ReadTemperature: WROTE: ");
        uint16_t crc = 0;
        roboclaw_.updateCrc(crc, roboclaw_.portAddress_);
        roboclaw_.updateCrc(crc, kGETTEMPERATURE);
        roboclaw_.writeN2(false, 2, roboclaw_.portAddress_, kGETTEMPERATURE);
        uint16_t result = 0;
        uint8_t datum = roboclaw_.readByteWithTimeout2();
        roboclaw_.updateCrc(crc, datum);
        result = datum << 8;
        datum = roboclaw_.readByteWithTimeout2();
        roboclaw_.updateCrc(crc, datum);
        result |= datum;

        uint16_t responseCrc = 0;
        datum = roboclaw_.readByteWithTimeout2();
        responseCrc = datum << 8;
        datum = roboclaw_.readByteWithTimeout2();
        responseCrc |= datum;
        if (responseCrc != crc) {
          RCUTILS_LOG_ERROR(
              "[RoboClaw::CmdReadTemperature] invalid CRC expected: 0x%2X, "
              "got: 0x%2X",
              crc, responseCrc);
          result = 0.0;
        }

        temperature_ = result / 10.0;
      } catch (...) {
        RCUTILS_LOG_ERROR(
            "[RoboClaw::CmdReadTemperature] Uncaught exception !!!");
      }

      roboclaw_.appendToReadLog(", RESULT: %f", temperature_);
    }

    float &temperature_;
  };

  class CmdDoBufferedM1M2DriveSpeedAccelDistance : public Cmd {
   public:
    CmdDoBufferedM1M2DriveSpeedAccelDistance(
        RoboClaw &roboclaw, uint32_t accel_quad_pulses_per_second,
        int32_t m1_speed_quad_pulses_per_second,
        uint32_t m1_max_distance_quad_pulses,
        int32_t m2_speed_quad_pulses_per_second,
        uint32_t m2_max_distance_quad_pulses)
        : Cmd(roboclaw, "DoBufferedDriveSpeedAccelDistance", kNone),
          accel_quad_pulses_per_second_(accel_quad_pulses_per_second),
          m1_speed_quad_pulses_per_second_(m1_speed_quad_pulses_per_second),
          m1_max_distance_quad_pulses_(m1_max_distance_quad_pulses),
          m2_speed_quad_pulses_per_second_(m2_speed_quad_pulses_per_second),
          m2_max_distance_quad_pulses_(m2_max_distance_quad_pulses) {}
    void send() override {
      try {
        roboclaw_.appendToWriteLog(
            "BufferedM1M2WithSignedSpeedAccelDist: accel: %d, m1Speed: %d, "
            "m1Distance: %d, m2Speed: %d, m2Distance: %d, WROTE: ",
            accel_quad_pulses_per_second_, m1_speed_quad_pulses_per_second_,
            m1_max_distance_quad_pulses_, m2_speed_quad_pulses_per_second_,
            m2_max_distance_quad_pulses_);
        roboclaw_.writeN2(true, 23, roboclaw_.portAddress_,
                          kMIXEDSPEEDACCELDIST,
                          SetDWORDval(accel_quad_pulses_per_second_),
                          SetDWORDval(m1_speed_quad_pulses_per_second_),
                          SetDWORDval(m1_max_distance_quad_pulses_),
                          SetDWORDval(m2_speed_quad_pulses_per_second_),
                          SetDWORDval(m2_max_distance_quad_pulses_),
                          1 /* Cancel any previous command. */
        );
      } catch (...) {
        RCUTILS_LOG_ERROR(
            "[RoboClaw::CmdDoBufferedM1M2DriveSpeedAccelDistance] Uncaught "
            "exception !!!");
      }
    }

    uint32_t accel_quad_pulses_per_second_;
    int32_t m1_speed_quad_pulses_per_second_;
    uint32_t m1_max_distance_quad_pulses_;
    int32_t m2_speed_quad_pulses_per_second_;
    uint32_t m2_max_distance_quad_pulses_;
  };

  friend class Cmd;  // Make Cmd a friend class of RoboClaw

 protected:
  DebugLog debug_log_;

  static const char *motorNames_[];
  static std::mutex
      buffered_command_mutex_;  // Global mutex for buffered commands
};
