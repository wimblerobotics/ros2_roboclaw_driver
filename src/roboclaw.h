#pragma once

#include <chrono>
#include <rclcpp/logger.hpp>
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

class RoboClaw {
 public:
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
    TRoboClawException(std::string ss) : s(ss) {}
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
           float m2MaxCurrent, std::string device_name, uint8_t device_port);

  ~RoboClaw();

  void doMixedSpeedDist(int32_t m1_quad_pulses_per_second,
                        int32_t m1_max_distance,
                        int32_t m2_quad_pulses_per_second,
                        int32_t m2_max_distance);

  void doMixedSpeedAccelDist(uint32_t accel_quad_pulses_per_second,
                             int32_t m1_quad_pulses_per_second,
                             uint32_t m1_max_distance,
                             int32_t m2_quad_pulses_per_second,
                             uint32_t m2_max_distance);

  EncodeResult getEncoderCommandResult(WHICH_ENC command);

  // Get RoboClaw error status bits.
  uint16_t getErrorStatus();

  // Get RoboClaw error status as a string.
  std::string getErrorString();

  float getLogicBatteryLevel();

  float getMainBatteryLevel();

  // Get the encoder value for motor 1.
  int32_t getM1Encoder();

  // Get the encoder value for motor 2.
  int32_t getM2Encoder();

  // Convenience structure to  hold a pair of current values.
  typedef struct {
    float m1Current;
    float m2Current;
  } TMotorCurrents;

  // Make sure you call getMotorCurrents before getMotorAlarms.
  int getMotorAlarms() { return motorAlarms_; }

  TMotorCurrents getMotorCurrents();

  TPIDQ getPIDQ(WHICH_MOTOR whichMotor);

  float getTemperature();

  // Get velocity (speed) of a motor.
  int32_t getVelocity(WHICH_VELOCITY whichVelocity);

  // Get RoboClaw software versions.
	std::string getVersion();
  
  // Stop motion.
  void stop();

  // Get singleton instance of class.
  static RoboClaw *singleton();

  static void readSensorGroup();
  
 private:
  EncodeResult cache_getEncoderCommandResult(WHICH_ENC command);

  // Get RoboClaw error status bits.
  uint16_t cache_getErrorStatus();

  // Get RoboClaw error status as a string.
  std::string cache_getErrorString();

  float cache_getLogicBatteryLevel();

  // Get the encoder value for motor 1.
  int32_t cache_getM1Encoder();

  int32_t cache_getM1Speed();

  // Get the encoder value for motor 2.
  int32_t cache_getM2Encoder();

  int32_t cache_getM2Speed();

  float cache_getMainBatteryLevel();

  TMotorCurrents cache_getMotorCurrents();

  TPIDQ cache_getPIDQ(WHICH_MOTOR whichMotor);

  float cache_getTemperature();

  // Get velocity (speed) of a motor.
  int32_t cache_getVelocity(WHICH_VELOCITY whichVelocity);

  static void sensorReadThread();

  typedef struct {
    uint16_t error_status;
    std::string error_string;
    float logic_battery_level;
    int32_t m1_encoder;
    EncodeResult m1_encoder_command_result;
    TPIDQ m1_pidq;
    int32_t m1_velocity;
    int32_t m2_encoder;
    EncodeResult m2_encoder_command_result;
    TPIDQ m2_pidq;
    int32_t m2_speed;
    float main_battery_level;
    int motor_alarms;
    TMotorCurrents motor_currents;
    float temperature;
    std::chrono::system_clock::time_point last_sensor_read_time_;
  } SensorValueGroup;

  static SensorValueGroup g_sensor_value_group_;

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

  unsigned long getUlongCommandResult(uint8_t command);

  uint32_t getULongCont(uint16_t &crc);

  unsigned short get2ByteCommandResult(uint8_t command);

  // Open the RoboClaw USB port.
  void openPort();

  // Read one byte from device with timeout.
  uint8_t readByteWithTimeout();

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
  void writeByte(uint8_t byte);

  // Write a stream of bytes to the device.
  void writeN(bool sendCRC, uint8_t cnt, ...);

  void SetEncoder(ROBOCLAW_COMMAND command, long value);

  static RoboClaw *g_singleton;
};
