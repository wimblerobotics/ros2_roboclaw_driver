#include "roboclaw.h"

#include <fcntl.h>
#include <math.h>
#include <poll.h>
#include <rcutils/logging_macros.h>
#include <stdio.h>
#include <sys/types.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include <boost/assign.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sstream>

#include "ros2_roboclaw_driver/srv/reset_encoders.h"

#if kDebugIO
const char *RoboClaw::motorNames_[] = {"M1", "M2", "NONE"};
const char *RoboClaw::commandNames_[] = {
    "Drive Forward M1",     // 0
    "Drive Backward M1",    //  1
    "Set Min MB",           // 2
    "Set Max MB",           // 3
    "Drive Forward M2",     // 4
    "Drive Backward M2",    // 5
    "Drive 7 Bit M1",       // 6
    "Drive 7 Bit M2",       // 7
    "Drive Forward Mixed",  // 8
    "Drive Backward Mixed", // 9
    "Turn Right Mixed",     // 10
    "Turn Left Mixed",      // 11
    "Drive FB Mixed",       // 12
    "Drive LR Mixed",       // 13
    "Set Serial Timeout",   // 14
    "Read Serial Timeout",  // 15
    "16",
    "17",
    "18",
    "19",
    "20",
    "Read Firmware Version", // 21
    "22",
    "23",
    "Read Main Battery Voltage",  // 24
    "Read Logic Battery Voltage", // 25
    "Set Min LB",                 // 26
    "Set Max LB",                 // 27
    "28",
    "29",
    "30",
    "31",
    "32",
    "33",
    "34",
    "35",
    "36",
    "37",
    "38",
    "39",
    "40",
    "41",
    "42",
    "43",
    "44",
    "45",
    "46",
    "47",
    "Read Motor PWMs",     // 48
    "Read Motor Currents", // 49
    "50",
    "51",
    "52",
    "53",
    "54",
    "55",
    "56",
    "Set Main Battery Volages",    // 57
    "Set Logic Battery Voltages",  // 58
    "Read Main Battery Voltages",  // 59
    "Read Logic Battery Voltages", // 60
    "61",
    "62",
    "63",
    "64",
    "65",
    "66",
    "67",
    "Set Default Duty Cycle Accel M1", // 68
    "Set Default Duty Cycle Accel M2", // 69
    "Set Default Speed M1",            // 70
    "Set Default Speed M2",            // 71
    "Read Default Speeds",             // 72
    "73",
    "Set S3, S4, S5 Modes",  // 74
    "Read S3, S4, S5 Modes", // 75
    "Set Deadband",          // 76
    "Read Deadband",         // 77
    "78",
    "79",
    "Restore Defaults",               // 80
    "Read Default Duty Cycle Accels", // 81
    "Read Temperature",               // 82
    "Read Temperature 2",             // 83
    "84",
    "85",
    "86",
    "87",
    "88",
    "89",
    "Read Status",               // 90
    "Read Encoder Modes",        // 91
    "Set M1 Encoder Mode",       // 92
    "Set M2 Encoder Mode",       // 93
    "Write Settings to EEPROM",  // 94
    "Read Settings from EEPROM", // 95
    "96",
    "97",
    "Set Standard Config",                 // 98
    "Read Standard Config",                // 99
    "Set CTRL Modes",                      // 100
    "Read CTRL Modes",                     // 101
    "Set CTRL1",                           // 102
    "Set CTRL2",                           // 103
    "Read CTRLs",                          // 104
    "Set Auto Home Duty/Speet/Timeout M1", // 105
    "Set Auto Home Duty/Speet/Timeout M2", // 106
    "Read Auto Home",                      // 107
    "108",
    "Set Speed Error Limits",  // 109
    "Read Speed Error Limits", // 110
    "111",
    "Set Speed Position Error Limits",  // 112
    "Read Speed Position Error Limits", // 113
    "114",
    "Set Battery Voltage Offsets",  // 115
    "Read Battery Voltage Offsets", // 116
    "Set Current Blanking %%s",     // 117
    "Read Current Blanking %%s",    // 118
    "119",
    "120",
    "121",
    "122",
    "123",
    "124",
    "125",
    "126",
    "127",
    "128",
    "129",
    "130",
    "131",
    "132",
    "Set M1 Max Current",  // 133
    "Set M2 Max Current",  // 134
    "Read M1 Max Current", // 135
    "Read M2 Max Current", // 136
    "137",
    "138",
    "139",
    "140",
    "141",
    "142",
    "143",
    "144",
    "145",
    "146",
    "147",
    "Set PWM Mode",  // 148
    "Read PWM Mode", // 149
    "150",
    "151",
    "152",
    "153",
    "154",
    "155",
    "156",
    "157",
    "158",
    "159",
    "160",
    "161",
    "162",
    "163",
    "164",
    "165",
    "166",
    "167",
    "168",
    "169",
    "170",
    "171",
    "172",
    "173",
    "174",
    "175",
    "176",
    "177",
    "178",
    "179",
    "180",
    "181",
    "182",
    "183",
    "184",
    "185",
    "186",
    "187",
    "188",
    "189",
    "190",
    "191",
    "192",
    "193",
    "194",
    "195",
    "196",
    "197",
    "198",
    "199",
    "200",
    "201",
    "202",
    "203",
    "204",
    "205",
    "206",
    "207",
    "208",
    "209",
    "210",
    "211",
    "212",
    "213",
    "214",
    "215",
    "216",
    "217",
    "218",
    "219",
    "220",
    "221",
    "222",
    "223",
    "224",
    "225",
    "226",
    "227",
    "228",
    "229",
    "230",
    "231",
    "232",
    "233",
    "234",
    "235",
    "236",
    "237",
    "238",
    "239",
    "240",
    "241",
    "242",
    "243",
    "244",
    "245",
    "246",
    "247",
    "248",
    "249",
    "250",
    "251",
    "Read User EEPROM Memory Location",  // 252
    "Write User EEPROM Memory Location", // 253
    "254",
    "255"};
#endif

RoboClaw::RoboClaw(const TPIDQ m1Pid, const TPIDQ m2Pid, float m1MaxCurrent,
                   float m2MaxCurrent, std::string device_name,
                   uint8_t device_port)
    : device_port_(device_port), maxCommandRetries_(3),
      maxM1Current_(m1MaxCurrent), maxM2Current_(m2MaxCurrent),
      device_name_(device_name), portAddress_(128) {
  openPort();
  RCUTILS_LOG_INFO("[RoboClaw::RoboClaw] RoboClaw software version: %s",
                   getVersion().c_str());
  setM1PID(m1Pid.p, m1Pid.i, m1Pid.d, m1Pid.qpps);
  setM2PID(m2Pid.p, m2Pid.i, m2Pid.d, m2Pid.qpps);
  ros2_roboclaw_driver::srv::ResetEncoders::Request resetRequest;
  resetRequest.left = 0;
  resetRequest.right = 0;
  ros2_roboclaw_driver::srv::ResetEncoders::Response response;
  resetEncoders(resetRequest, response);
  g_singleton = this;
  readSensorGroup();
}

RoboClaw::~RoboClaw() {}

void RoboClaw::doMixedSpeedDist(int32_t m1_quad_pulses_per_second,
                                int32_t m1_max_distance,
                                int32_t m2_quad_pulses_per_second,
                                int32_t m2_max_distance) {
  writeN(true, 19, portAddress_, kMIXEDSPEEDDIST,
         SetDWORDval(m1_quad_pulses_per_second), SetDWORDval(m1_max_distance),
         SetDWORDval(m2_quad_pulses_per_second), SetDWORDval(m2_max_distance),
         1 /* Cancel any previous command. */
  );
}

void RoboClaw::doMixedSpeedAccelDist(uint32_t accel_quad_pulses_per_second,
                                     int32_t m1_quad_pulses_per_second,
                                     uint32_t m1_max_distance,
                                     int32_t m2_quad_pulses_per_second,
                                     uint32_t m2_max_distance) {
  writeN(true, 23, portAddress_, kMIXEDSPEEDACCELDIST,
         SetDWORDval(accel_quad_pulses_per_second),
         SetDWORDval(m1_quad_pulses_per_second), SetDWORDval(m1_max_distance),
         SetDWORDval(m2_quad_pulses_per_second), SetDWORDval(m2_max_distance),
         1 /* Cancel any previous command. */
  );
}

RoboClaw::EncodeResult RoboClaw::getEncoderCommandResult(WHICH_ENC command) {
  if (command == kGETM1ENC) {
    return g_sensor_value_group_.m1_encoder_command_result;
  } else {
    return g_sensor_value_group_.m2_encoder_command_result;
  }
}

RoboClaw::EncodeResult
RoboClaw::cache_getEncoderCommandResult(WHICH_ENC command) {
  uint16_t crc = 0;
  updateCrc(crc, portAddress_);
  updateCrc(crc, command);

  writeN(false, 2, portAddress_, command);
  EncodeResult result = {0, 0};
  uint8_t datum = readByteWithTimeout();
  result.value |= datum << 24;
  updateCrc(crc, datum);

  datum = readByteWithTimeout();
  result.value |= datum << 16;
  updateCrc(crc, datum);

  datum = readByteWithTimeout();
  result.value |= datum << 8;
  updateCrc(crc, datum);

  datum = readByteWithTimeout();
  result.value |= datum;
  updateCrc(crc, datum);

  datum = readByteWithTimeout();
  result.status |= datum;
  updateCrc(crc, datum);

  uint16_t responseCrc = 0;
  datum = readByteWithTimeout();
  responseCrc = datum << 8;
  datum = readByteWithTimeout();
  responseCrc |= datum;
  if (responseCrc == crc) {
    return result;
  }

  RCUTILS_LOG_ERROR(
      "[RoboClaw::cache_getEncoderCommandResult] Expected CRC of: 0x%02X, but "
      "got: 0x%02X",
      int(crc), int(responseCrc));
  throw new TRoboClawException(
      "[RoboClaw::cache_getEncoderCommandResult] INVALID CRC");
}

uint16_t RoboClaw::getErrorStatus() {
  return g_sensor_value_group_.error_status;
}

uint16_t RoboClaw::cache_getErrorStatus() {
  for (int retry = 0; retry < maxCommandRetries_; retry++) {
    try {
      uint16_t crc = 0;
      updateCrc(crc, portAddress_);
      updateCrc(crc, kGETERROR);

      writeN(false, 2, portAddress_, kGETERROR);
      unsigned short result = (unsigned short)getULongCont(crc);
      uint16_t responseCrc = 0;
      uint16_t datum = readByteWithTimeout();
      responseCrc = datum << 8;
      datum = readByteWithTimeout();
      responseCrc |= datum;
      if (responseCrc == crc) {
        return result;
      } else {
        RCUTILS_LOG_ERROR(
            "[RoboClaw::cache_getErrorStatus] invalid CRC expected: 0x%02X, "
            "got: "
            "0x%02X",
            crc, responseCrc);
      }
    } catch (TRoboClawException *e) {
      RCUTILS_LOG_ERROR(
          "[RoboClaw::cache_getErrorStatus] Exception: %s, retry number: %d",
          e->what(), retry);
    } catch (...) {
      RCUTILS_LOG_ERROR(
          "[RoboClaw::cache_getErrorStatus] Uncaught exception !!!");
    }
  }

  RCUTILS_LOG_ERROR("[RoboClaw::cache_getErrorStatus] RETRY COUNT EXCEEDED");
  throw new TRoboClawException(
      "[RoboClaw::cache_getErrorStatus] RETRY COUNT EXCEEDED");
}

std::string RoboClaw::getErrorString() {
  return g_sensor_value_group_.error_string;
}

std::string RoboClaw::cache_getErrorString() {
  uint16_t errorStatus = getErrorStatus();
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

float RoboClaw::cache_getLogicBatteryLevel() {
  int retry;

  for (retry = 0; retry < maxCommandRetries_; retry++) {
    try {
      float result = ((float)get2ByteCommandResult(kGETLBATT)) / 10.0;
      return result;
    } catch (TRoboClawException *e) {
      RCUTILS_LOG_ERROR(
          "[RoboClaw::cache_getLogicBatteryLevel] Exception: %s, retry "
          "number: %d",
          e->what(), retry);
    } catch (...) {
      RCUTILS_LOG_ERROR(
          "[RoboClaw::cache_getLogicBatteryLevel] Uncaught exception !!!");
    }
  }

  RCUTILS_LOG_ERROR(
      "[RoboClaw::cache_getLogicBatteryLevel] RETRY COUNT EXCEEDED");
  throw new TRoboClawException(
      "[RoboClaw::cache_getLogicBatteryLevel] RETRY COUNT EXCEEDED");
}

int32_t RoboClaw::getM1Encoder() { return g_sensor_value_group_.m1_encoder; }

int32_t RoboClaw::cache_getM1Encoder() {
  int retry;

  for (retry = 0; retry < maxCommandRetries_; retry++) {
    try {
      EncodeResult result = getEncoderCommandResult(kGETM1ENC);
      return result.value;
    } catch (TRoboClawException *e) {
      RCUTILS_LOG_ERROR(
          "[RoboClaw::cache_getM1Encoder] Exception: %s, retry number: %d",
          e->what(), retry);
    } catch (...) {
      RCUTILS_LOG_ERROR(
          "[RoboClaw::cache_getM1Encoder] Uncaught exception !!!");
    }
  }

  RCUTILS_LOG_ERROR("[RoboClaw::cache_getM1Encoder] RETRY COUNT EXCEEDED");
  throw new TRoboClawException(
      "[RoboClaw::cache_getM1Encoder] RETRY COUNT EXCEEDED");
}

float RoboClaw::getMainBatteryLevel() {
  return g_sensor_value_group_.main_battery_level;
}

float RoboClaw::cache_getMainBatteryLevel() {
  int retry;

  for (retry = 0; retry < maxCommandRetries_; retry++) {
    try {
      float result = ((float)get2ByteCommandResult(kGETMBATT)) / 10.0;
      return result;
    } catch (TRoboClawException *e) {
      RCUTILS_LOG_ERROR_EXPRESSION(
          "[RoboClaw::cache_getMainBatteryLevel] Exception: %s, retry number: "
          "%d",
          e->what(), retry);
    } catch (...) {
      RCUTILS_LOG_ERROR(
          "[RoboClaw::cache_getMainBatteryLevel] Uncaught exception !!!");
    }
  }

  RCUTILS_LOG_ERROR(
      "[RoboClaw::cache_getMainBatteryLevel] RETRY COUNT EXCEEDED");
  throw new TRoboClawException(
      "[RoboClaw::cache_getMainBatteryLevel] RETRY COUNT EXCEEDED");
}

unsigned short RoboClaw::get2ByteCommandResult(uint8_t command) {
  uint16_t crc = 0;
  updateCrc(crc, portAddress_);
  updateCrc(crc, command);

  writeN(false, 2, portAddress_, command);
  unsigned short result = 0;
  uint8_t datum = readByteWithTimeout();
  result |= datum << 8;
  updateCrc(crc, datum);

  datum = readByteWithTimeout();
  result |= datum;
  updateCrc(crc, datum);

  uint16_t responseCrc = 0;
  datum = readByteWithTimeout();
  responseCrc = datum << 8;
  datum = readByteWithTimeout();
  responseCrc |= datum;
  if (responseCrc == crc) {
    return result;
  } else {
    RCUTILS_LOG_ERROR("[RoboClaw::get2ByteCommandResult] invalid CRC expected: "
                      "0x%02X, got: 0x%02X",
                      crc, responseCrc);
    throw new TRoboClawException(
        "[RoboClaw::get2ByteCommandResult] INVALID CRC");
    return 0;
  }
}

RoboClaw::TMotorCurrents RoboClaw::getMotorCurrents() {
  return g_sensor_value_group_.motor_currents;
}

RoboClaw::TMotorCurrents RoboClaw::cache_getMotorCurrents() {
  int retry;

  for (retry = 0; retry < maxCommandRetries_; retry++) {
    try {
      TMotorCurrents result;
      unsigned long currentPair = getUlongCommandResult(kGETCURRENTS);
      result.m1Current = ((int16_t)(currentPair >> 16)) * 0.010;
      result.m2Current = ((int16_t)(currentPair & 0xFFFF)) * 0.010;
      if (result.m1Current > maxM1Current_) {
        motorAlarms_ |= kM1_OVER_CURRENT_ALARM;
        RCUTILS_LOG_ERROR(
            "[RoboClaw::cache_getMotorCurrents] Motor 1 over current. Max "
            "allowed: %6.3f, found: %6.3f",
            maxM1Current_, result.m1Current);
        stop();
      } else {
        motorAlarms_ &= ~kM1_OVER_CURRENT_ALARM;
      }

      if (result.m2Current > maxM2Current_) {
        motorAlarms_ |= kM2_OVER_CURRENT_ALARM;
        RCUTILS_LOG_ERROR(
            "[RoboClaw::cache_getMotorCurrents] Motor 2 over current. Max "
            "allowed: %6.3f, found: %6.3f",
            maxM2Current_, result.m2Current);
        stop();
      } else {
        motorAlarms_ &= ~kM2_OVER_CURRENT_ALARM;
      }

      return result;
    } catch (TRoboClawException *e) {
      RCUTILS_LOG_ERROR(
          "[RoboClaw::cache_getMotorCurrents] Exception: %s, retry number: %d",
          e->what(), retry);
    } catch (...) {
      RCUTILS_LOG_ERROR(
          "[RoboClaw::cache_getMotorCurrents] Uncaught exception !!!");
    }
  }

  RCUTILS_LOG_ERROR("RoboClaw::cache_getMotorCurrents] RETRY COUNT EXCEEDED");
  throw new TRoboClawException(
      "[RoboClaw::cache_getMotorCurrents] RETRY COUNT EXCEEDED");
}

RoboClaw::TPIDQ RoboClaw::getPIDQ(WHICH_MOTOR whichMotor) {
  if (whichMotor == kGETM1PID) {
    return g_sensor_value_group_.m1_pidq;
  } else {
    return g_sensor_value_group_.m2_pidq;
  }
}

RoboClaw::TPIDQ RoboClaw::cache_getPIDQ(WHICH_MOTOR whichMotor) {
  int retry;

  for (retry = 0; retry < maxCommandRetries_; retry++) {
    TPIDQ result;
    try {
      uint16_t crc = 0;
      updateCrc(crc, portAddress_);
      updateCrc(crc, whichMotor);

      writeN(false, 2, portAddress_, whichMotor);
      result.p = (int32_t)getULongCont(crc);
      result.i = (int32_t)getULongCont(crc);
      result.d = (int32_t)getULongCont(crc);
      result.qpps = (int32_t)getULongCont(crc);

      uint16_t responseCrc = 0;
      uint16_t datum = readByteWithTimeout();
      responseCrc = datum << 8;
      datum = readByteWithTimeout();
      responseCrc |= datum;
      if (responseCrc == crc) {
        return result;
      } else {
        RCUTILS_LOG_ERROR(
            "[RoboClaw::cache_getPIDQ] invalid CRC expected: 0x%2X, got: "
            "0x%2X",
            crc, responseCrc);
      }
    } catch (TRoboClawException *e) {
      RCUTILS_LOG_ERROR(
          "[RoboClaw::cache_getPIDQ] Exception: %s, retry number: %d",
          e->what(), retry);
    } catch (...) {
      RCUTILS_LOG_ERROR("[RoboClaw::cache_getPIDQ] Uncaught exception !!!");
    }
  }

  RCUTILS_LOG_ERROR("[RoboClaw::cache_getPIDQ] RETRY COUNT EXCEEDED");
  throw new TRoboClawException(
      "[RoboClaw::cache_getPIDQ] RETRY COUNT EXCEEDED");
}

float RoboClaw::getTemperature() { return g_sensor_value_group_.temperature; }

float RoboClaw::cache_getTemperature() {
  int retry;

  for (retry = 0; retry < maxCommandRetries_; retry++) {
    try {
      uint16_t crc = 0;
      updateCrc(crc, portAddress_);
      updateCrc(crc, kGETTEMPERATURE);
      writeN(false, 2, portAddress_, kGETTEMPERATURE);
      uint16_t result = 0;
      uint8_t datum = readByteWithTimeout();
      updateCrc(crc, datum);
      result = datum << 8;
      datum = readByteWithTimeout();
      updateCrc(crc, datum);
      result |= datum;

      uint16_t responseCrc = 0;
      datum = readByteWithTimeout();
      responseCrc = datum << 8;
      datum = readByteWithTimeout();
      responseCrc |= datum;
      if (responseCrc == crc) {
        return result / 10.0;
      } else {
        RCUTILS_LOG_ERROR(
            "[RoboClaw::cache_getTemperature] invalid CRC expected: 0x%2X, "
            "got: 0x%2X",
            crc, responseCrc);
      }
    } catch (TRoboClawException *e) {
      RCUTILS_LOG_ERROR(
          "[RoboClaw::cache_getTemperature] Exception: %s, retry number: %d",
          e->what(), retry);
    } catch (...) {
      RCUTILS_LOG_ERROR(
          "[RoboClaw::cache_getTemperature] Uncaught exception !!!");
    }
  }

  RCUTILS_LOG_ERROR("[RoboClaw::cache_getTemperature] RETRY COUNT EXCEEDED");
  throw new TRoboClawException(
      "[RoboClaw::cache_getTemperature] RETRY COUNT EXCEEDED");
}

unsigned long RoboClaw::getUlongCommandResult(uint8_t command) {
  uint16_t crc = 0;
  updateCrc(crc, portAddress_);
  updateCrc(crc, command);

  writeN(false, 2, portAddress_, command);
  unsigned long result = 0;
  uint8_t datum = readByteWithTimeout();
  result |= datum << 24;
  updateCrc(crc, datum);
  datum = readByteWithTimeout();
  result |= datum << 16;
  updateCrc(crc, datum);
  datum = readByteWithTimeout();
  result |= datum << 8;
  updateCrc(crc, datum);
  datum = readByteWithTimeout();
  result |= datum;
  updateCrc(crc, datum);

  uint16_t responseCrc = 0;
  datum = readByteWithTimeout();
  responseCrc = datum << 8;
  datum = readByteWithTimeout();
  responseCrc |= datum;
  if (responseCrc == crc) {
    return result;
  }

  RCUTILS_LOG_ERROR(
      "[RoboClaw::getUlongCommandResult] Expected CRC of: 0x%02X, but "
      "got: 0x%02X",
      int(crc), int(responseCrc));
  throw new TRoboClawException("[RoboClaw::getUlongCommandResult] INVALID CRC");
  return 0;
}

uint32_t RoboClaw::getULongCont(uint16_t &crc) {
  uint32_t result = 0;
  uint8_t datum = readByteWithTimeout();
  result |= datum << 24;
  updateCrc(crc, datum);
  datum = readByteWithTimeout();
  result |= datum << 16;
  updateCrc(crc, datum);
  datum = readByteWithTimeout();
  result |= datum << 8;
  updateCrc(crc, datum);
  datum = readByteWithTimeout();
  result |= datum;
  updateCrc(crc, datum);
  return result;
}

int32_t RoboClaw::getVelocity(WHICH_VELOCITY whichVelocity) {
  if (whichVelocity == kGETM1SPEED) {
    return g_sensor_value_group_.m1_velocity;
  } else {
    return g_sensor_value_group_.m2_speed;
  }
}

int32_t RoboClaw::cache_getVelocity(WHICH_VELOCITY whichVelocity) {
  int retry;

  for (retry = 0; retry < maxCommandRetries_; retry++) {
    try {
      uint32_t result = getVelocityResult(whichVelocity);
      return result;
    } catch (TRoboClawException *e) {
      RCUTILS_LOG_ERROR(
          "[RoboClaw::cache_getVelocity] Exception: %s, retry number: %d",
          e->what(), retry);
    } catch (...) {
      RCUTILS_LOG_ERROR("[RoboClaw::cache_getVelocity] Uncaught exception !!!");
    }
  }

  RCUTILS_LOG_ERROR("RoboClaw::cache_getVelocity] RETRY COUNT EXCEEDED");
  throw new TRoboClawException(
      "[RoboClaw::cache_getVelocity] RETRY COUNT EXCEEDED");
}

int32_t RoboClaw::getVelocityResult(uint8_t command) {
  uint16_t crc = 0;
  updateCrc(crc, portAddress_);
  updateCrc(crc, command);

  writeN(false, 2, portAddress_, command);
  int32_t result = 0;
  uint8_t datum = readByteWithTimeout();
  result |= datum << 24;
  updateCrc(crc, datum);

  datum = readByteWithTimeout();
  result |= datum << 16;
  updateCrc(crc, datum);

  datum = readByteWithTimeout();
  result |= datum << 8;
  updateCrc(crc, datum);

  datum = readByteWithTimeout();
  result |= datum;
  updateCrc(crc, datum);

  uint8_t direction = readByteWithTimeout();
  updateCrc(crc, direction);
  if (direction != 0)
    result = -result;

  uint16_t responseCrc = 0;
  datum = readByteWithTimeout();
  responseCrc = datum << 8;
  datum = readByteWithTimeout();
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

int32_t RoboClaw::getM2Encoder() { return g_sensor_value_group_.m2_encoder; }

int32_t RoboClaw::cache_getM2Encoder() {
  int retry;

  for (retry = 0; retry < maxCommandRetries_; retry++) {
    try {
      EncodeResult result = getEncoderCommandResult(kGETM2ENC);
      return result.value;
    } catch (TRoboClawException *e) {
      RCUTILS_LOG_ERROR(
          "[RoboClaw::cache_getM2Encoder] Exception: %s, retry number: %d",
          e->what(), retry);
    } catch (...) {
      RCUTILS_LOG_ERROR(
          "[RoboClaw::cache_getM2Encoder] Uncaught exception !!!");
    }
  }

  RCUTILS_LOG_ERROR("[RoboClaw::cache_getM2Encoder] RETRY COUNT EXCEEDED");
  throw new TRoboClawException(
      "[RoboClaw::cache_getM2Encoder] RETRY COUNT EXCEEDED");
}

std::string RoboClaw::getVersion() {
  int retry;

  for (retry = 0; retry < maxCommandRetries_; retry++) {
    try {
      uint16_t crc = 0;
      updateCrc(crc, portAddress_);
      updateCrc(crc, kGETVERSION);
      writeN(false, 2, portAddress_, kGETVERSION);

      uint8_t i;
      uint8_t datum;
      std::stringstream version;

      for (i = 0; i < 48; i++) {
        datum = readByteWithTimeout();
        updateCrc(crc, datum);
        if (datum == 0) {
          uint16_t responseCrc = 0;
          datum = readByteWithTimeout();
          responseCrc = datum << 8;
          datum = readByteWithTimeout();
          responseCrc |= datum;
          if (responseCrc == crc) {
            return version.str();
          } else {
            RCUTILS_LOG_ERROR(
                "[RoboClaw::getVersion] invalid CRC expected: 0x%02X, "
                "got: 0x%02X",
                crc, responseCrc);
          }
        } else {
          version << (char)datum;
        }
      }

      RCUTILS_LOG_ERROR("[RoboClaw::getVersion] unexpected long string");
      throw new TRoboClawException(
          "[RoboClaw::getVersion] unexpected long string");
    } catch (TRoboClawException *e) {
      RCUTILS_LOG_ERROR(
          "[RoboClaw::getVersion] Exception: %s, retry number: %d", e->what(),
          retry);
    } catch (...) {
      RCUTILS_LOG_ERROR("[RoboClaw::getVersion] Uncaught exception !!!");
    }
  }

  RCUTILS_LOG_ERROR("[RoboClaw::getVersion] RETRY COUNT EXCEEDED");
  throw new TRoboClawException("[RoboClaw::getVersion] RETRY COUNT EXCEEDED");
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

uint8_t RoboClaw::readByteWithTimeout() {
  struct pollfd ufd[1];
  ufd[0].fd = device_port_;
  ufd[0].events = POLLIN;

  int retval = poll(ufd, 1, 11);
  if (retval < 0) {
    RCUTILS_LOG_ERROR("[RoboClaw::readByteWithTimeout] Poll failed (%d) %s",
                      errno, strerror(errno));
    throw new TRoboClawException("[RoboClaw::readByteWithTimeout] Read error");
  } else if (retval == 0) {
    std::stringstream ev;
    ev << "[RoboClaw::readByteWithTimeout] TIMEOUT revents: " << std::hex
       << ufd[0].revents;
    RCUTILS_LOG_ERROR(ev.str().c_str());
    throw new TRoboClawException("[RoboClaw::readByteWithTimeout] TIMEOUT");
  } else if (ufd[0].revents & POLLERR) {
    RCUTILS_LOG_ERROR("[RoboClaw::readByteWithTimeout] Error on socket");
    restartPort();
    throw new TRoboClawException(
        "[RoboClaw::readByteWithTimeout] Error on socket");
  } else if (ufd[0].revents & POLLIN) {
    unsigned char buffer[1];
    ssize_t bytesRead = ::read(device_port_, buffer, sizeof(buffer));
    if (bytesRead != 1) {
      RCUTILS_LOG_ERROR(
          "[RoboClaw::readByteWithTimeout] Failed to read 1 byte, read: "
          "%d",
          (int)bytesRead);
      throw TRoboClawException(
          "[RoboClaw::readByteWithTimeout] Failed to read 1 byte");
    }

    static const bool kDEBUG_READBYTE = true;
    if (kDEBUG_READBYTE) {
      if ((buffer[0] < 0x21) || (buffer[0] > 0x7F)) {
        RCUTILS_LOG_INFO("read ..> char: ?? 0x%02X <--", buffer[0]);
        // fprintf(stderr, "..> char: ?? 0x%02X <--", buffer[0]);
      } else {
        RCUTILS_LOG_INFO("read ..> char: %c  0x%02X <--", buffer[0], buffer[0]);
        // fprintf(stderr, "..> char: %c  0x%02X <--", buffer[0], buffer[0]);
      }
    }

    return buffer[0];
  } else {
    RCUTILS_LOG_ERROR("[RoboClaw::readByteWithTimeout] Unhandled case");
    throw new TRoboClawException(
        "[RoboClaw::readByteWithTimeout] Unhandled case");
  }

  return 0;
}

void RoboClaw::readSensorGroup() {
  if (singleton() != nullptr) {
    g_sensor_value_group_.error_status = singleton()->cache_getErrorStatus();
    g_sensor_value_group_.error_string = singleton()->cache_getErrorString();
    g_sensor_value_group_.logic_battery_level =
        singleton()->cache_getLogicBatteryLevel();
    g_sensor_value_group_.m1_encoder = singleton()->cache_getM1Encoder();
    g_sensor_value_group_.m1_encoder_command_result =
        singleton()->cache_getEncoderCommandResult(kGETM1ENC);
    g_sensor_value_group_.m1_pidq = singleton()->cache_getPIDQ(kGETM1PID);
    g_sensor_value_group_.m1_velocity =
        singleton()->cache_getVelocity(RoboClaw::kGETM1SPEED);
    g_sensor_value_group_.m2_encoder = singleton()->cache_getM2Encoder();
    g_sensor_value_group_.m2_encoder_command_result =
        singleton()->cache_getEncoderCommandResult(kGETM2ENC);
    g_sensor_value_group_.m1_pidq = singleton()->cache_getPIDQ(kGETM2PID);
    g_sensor_value_group_.m2_speed =
        singleton()->cache_getVelocity(RoboClaw::kGETM2SPEED);
    g_sensor_value_group_.main_battery_level =
        singleton()->cache_getMainBatteryLevel();

    // Call getMotorCurrents before getMotorAlarms;
    g_sensor_value_group_.motor_currents =
        singleton()->cache_getMotorCurrents();
    g_sensor_value_group_.motor_alarms = singleton()->getMotorAlarms();

    g_sensor_value_group_.temperature = singleton()->cache_getTemperature();
    g_sensor_value_group_.last_sensor_read_time_ =
        std::chrono::system_clock::now();
  }
}

bool RoboClaw::resetEncoders(
    ros2_roboclaw_driver::srv::ResetEncoders::Request &request,
    ros2_roboclaw_driver::srv::ResetEncoders::Response &response) {
  try {
    SetEncoder(kSETM1ENCODER, request.left);
    SetEncoder(kSETM2ENCODER, request.right);
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

void RoboClaw::SetEncoder(ROBOCLAW_COMMAND command, long value) {
  int retry;

  if ((command != kSETM1ENCODER) && (command != kSETM2ENCODER)) {
    RCUTILS_LOG_ERROR("[RoboClaw::SetEncoder] Invalid command value");
    throw new TRoboClawException(
        "[RoboClaw::SetEncoder] Invalid command value");
  }

  for (retry = 0; retry < maxCommandRetries_; retry++) {
    try {
      writeN(true, 6, portAddress_, command, SetDWORDval(value));
      return;
    } catch (TRoboClawException *e) {
      RCUTILS_LOG_ERROR(
          "[RoboClaw::SetEncoder] Exception: %s, retry number: %d", e->what(),
          retry);
    } catch (...) {
      RCUTILS_LOG_ERROR("[RoboClaw::SetEncoder] Uncaught exception !!!");
    }
  }

  RCUTILS_LOG_ERROR("[RoboClaw::SetEncoder] RETRY COUNT EXCEEDED");
  throw new TRoboClawException("[RoboClaw::SetEncoder] RETRY COUNT EXCEEDED");
}

void RoboClaw::setM1PID(float p, float i, float d, uint32_t qpps) {
  CmdSetPid command(*this, kM1, p, i, d, qpps);
  command.execute();

  // int retry;

  // for (retry = 0; retry < maxCommandRetries_; retry++) {
  //   try {
  //     uint32_t kp = int(p * 65536.0);
  //     uint32_t ki = int(i * 65536.0);
  //     uint32_t kd = int(d * 65536.0);
  //     writeN(true, 18, portAddress_, kSETM1PID, SetDWORDval(kd),
  //            SetDWORDval(kp), SetDWORDval(ki), SetDWORDval(qpps));
  //     return;
  //   } catch (TRoboClawException *e) {
  //     RCUTILS_LOG_ERROR("[RoboClaw::setM1PID] Exception: %s, retry number:
  //     %d",
  //                       e->what(), retry);
  //   } catch (...) {
  //     RCUTILS_LOG_ERROR("[RoboClaw::setM1PID] Uncaught exception !!!");
  //   }
  // }

  // RCUTILS_LOG_ERROR("[RoboClaw::setM1PID] RETRY COUNT EXCEEDED");
  // throw new TRoboClawException("[RoboClaw::setM1PID] RETRY COUNT EXCEEDED");
}

void RoboClaw::setM2PID(float p, float i, float d, uint32_t qpps) {
  int retry;

  for (retry = 0; retry < maxCommandRetries_; retry++) {
    try {
      uint32_t kp = int(p * 65536.0);
      uint32_t ki = int(i * 65536.0);
      uint32_t kd = int(d * 65536.0);
      writeN(true, 18, portAddress_, kSETM2PID, SetDWORDval(kd),
             SetDWORDval(kp), SetDWORDval(ki), SetDWORDval(qpps));
      return;
    } catch (TRoboClawException *e) {
      RCUTILS_LOG_ERROR("[RoboClaw::setM2PID] Exception: %s, retry number: %d",
                        e->what(), retry);
    } catch (...) {
      RCUTILS_LOG_ERROR("[RoboClaw::setM2PID] Uncaught exception !!!");
    }
  }

  RCUTILS_LOG_ERROR("[RoboClaw::setM2PID] RETRY COUNT EXCEEDED");
  throw new TRoboClawException("[RoboClaw::setM2PID] RETRY COUNT EXCEEDED");
}

void RoboClaw::stop() {
  int retry;

  for (retry = 0; retry < maxCommandRetries_; retry++) {
    try {
      writeN(true, 6, portAddress_, kMIXEDDUTY, 0, 0, 0, 0);
      RCUTILS_LOG_INFO("[RoboClaw::stop] Stop requested"); // #####
      return;
    } catch (TRoboClawException *e) {
      RCUTILS_LOG_ERROR("[RoboClaw::stop] Exception: %s, retry number: %d",
                        e->what(), retry);
      restartPort();
    } catch (...) {
      RCUTILS_LOG_ERROR("[RoboClaw::stop] Uncaught exception !!!");
    }
  }

  RCUTILS_LOG_ERROR("[RoboClaw::stop] RETRY COUNT EXCEEDED");
  throw new TRoboClawException("[RoboClaw::stop] RETRY COUNT EXCEEDED");
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

void RoboClaw::writeByte(uint8_t byte) {
  ssize_t result;
  do {
    result = ::write(device_port_, &byte, 1);
    RCUTILS_LOG_INFO("--> wrote: 0x%02X, result: %ld", byte, result); // ####
  } while (result == -1 && errno == EAGAIN);

  if (result != 1) {
    RCUTILS_LOG_ERROR(
        "[RoboClaw::writeByte] Unable to write one byte, result: %d, "
        "errno: %d)",
        (int)result, errno);
    restartPort();
    throw new TRoboClawException(
        "[RoboClaw::writeByte] Unable to write one byte");
  }
}

void RoboClaw::writeByte2(uint8_t byte) {
  ssize_t result;
  do {
    result = ::write(device_port_, &byte, 1);
// RCUTILS_LOG_INFO("--> wrote: 0x%02X, result: %ld", byte, result);  // ####
#if kDebugIO
    if (result == 1) {
      appendToLog("%02X ", byte);
    } else {
      appendToLog(">%02X< ", byte);
    }
#endif

  } while (result == -1 && errno == EAGAIN);

  if (result != 1) {
    RCUTILS_LOG_ERROR(
        "[RoboClaw::writeByte] Unable to write one byte, result: %d, "
        "errno: %d)",
        (int)result, errno);
    restartPort();
    throw new TRoboClawException(
        "[RoboClaw::writeByte] Unable to write one byte");
  }
}

void RoboClaw::writeN(bool sendCRC, uint8_t cnt, ...) {
  uint16_t crc = 0;
  va_list marker;
  va_start(marker, cnt);
  char msg[128];

  // int origFlags = fcntl(device_port_, F_GETFL, 0);
  // fcntl(device_port_, F_SETFL, origFlags & ~O_NONBLOCK);

  for (uint8_t i = 0; i < cnt; i++) {
    uint8_t byte = va_arg(marker, int);
    if (i <= 1) {
      snprintf(msg, sizeof(msg),
               "RoboClaw::writeN] sendCRC: %d, cnt: %d, byte[%d]: %d", sendCRC,
               cnt, i, byte);
      RCUTILS_LOG_INFO("%s", msg);
      if (i == 1) {
        snprintf(msg, sizeof(msg), "[RoboClaw::writeN] command: %s",
                 commandNames_[byte]);
        RCUTILS_LOG_INFO("%s", msg);
      }
    }

    updateCrc(crc, byte);
    writeByte(byte);
  }

  va_end(marker);

  if (sendCRC) {
    writeByte(crc >> 8);
    writeByte(crc);

    uint8_t response = readByteWithTimeout();
    if (response != 0xFF) {
      snprintf(msg, sizeof(msg),
               "[RoboClaw::writeN] Invalid ACK response, expected 0xFF but got "
               "0x%02X",
               response);
      RCUTILS_LOG_ERROR("%s", msg);
      throw new TRoboClawException(msg);
    }
  }

  // fcntl(device_port_, F_SETFL, origFlags);
}

void RoboClaw::writeN2(bool sendCRC, uint8_t cnt, ...) {
  uint16_t crc = 0;
  va_list marker;
  va_start(marker, cnt);
  char msg[128];

  // int origFlags = fcntl(device_port_, F_GETFL, 0);
  // fcntl(device_port_, F_SETFL, origFlags & ~O_NONBLOCK);

  for (uint8_t i = 0; i < cnt; i++) {
    uint8_t byte = va_arg(marker, int);
    if (i <= 1) {
      snprintf(msg, sizeof(msg),
               "RoboClaw::writeN2] sendCRC: %d, cnt: %d, byte[%d]: %d", sendCRC,
               cnt, i, byte);
      RCUTILS_LOG_INFO("%s", msg);
      if (i == 1) {
        snprintf(msg, sizeof(msg), "[RoboClaw::writeN2] command: %s",
                 commandNames_[byte]);
        RCUTILS_LOG_INFO("%s", msg);
      }
    }

    updateCrc(crc, byte);
    writeByte2(byte);
  }

  va_end(marker);

  if (sendCRC) {
    writeByte2(crc >> 8);
    writeByte2(crc);

    uint8_t response = readByteWithTimeout();
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

RoboClaw::SensorValueGroup RoboClaw::g_sensor_value_group_;
RoboClaw *RoboClaw::g_singleton = nullptr;