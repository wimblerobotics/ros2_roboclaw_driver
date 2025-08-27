// MIT License
// Author: Michael Wimble <mike@wimblerobotics.com>
#include "roboclaw_driver/hardware_interface.hpp"

#include <cstring>
namespace roboclaw_driver {

// Command codes (from legacy enum; verify against current datasheet)
static constexpr uint8_t CMD_RESET_ENCODERS = 0x14;          // RESETENC (20)
static constexpr uint8_t CMD_GET_VERSION = 0x15;             // GETVERSION (21)
static constexpr uint8_t CMD_GET_MBATT = 0x18;               // GETMBATT (24)
static constexpr uint8_t CMD_GET_LBATT = 0x19;               // GETLBATT (25)
static constexpr uint8_t CMD_SET_M1_PIDQ = 0x1C;             // SETM1PID (28)
static constexpr uint8_t CMD_SET_M2_PIDQ = 0x1D;             // SETM2PID (29)
static constexpr uint8_t CMD_GET_M1_ISPEED = 0x1E;           // GETM1ISPEED (30)
static constexpr uint8_t CMD_GET_M2_ISPEED = 0x1F;           // GETM2ISPEED (31)
static constexpr uint8_t CMD_MIXED_SPEED = 0x25;             // kMIXEDSPEED (37)
static constexpr uint8_t CMD_MIXED_SPEED_ACCEL_DIST = 0x2E;  // MIXEDSPEEDACCELDIST (46)
static constexpr uint8_t CMD_GET_CURRENTS = 0x31;            // GETCURRENTS (49)
static constexpr uint8_t CMD_GET_TEMPERATURE = 0x52;         // GETTEMPERATURE (82)
static constexpr uint8_t CMD_GET_ERROR = 0x5A;               // GETERROR (90)
static constexpr uint8_t CMD_READ_ENCODER_M1 =
    0x16;  // GETM1ENC (22) NOTE legacy header used SETM1ENCODER at 22; adjust if needed
static constexpr uint8_t CMD_READ_ENCODER_M2 = 0x17;  // GETM2ENC (23)
// NOTE: Some code points above differ from earlier placeholder mapping; confirm with datasheet.

bool HardwareInterface::initialize(std::string& err) {
  auto resp = proto_.transact(CMD_GET_ERROR, nullptr, 0, 4, 3, 0.05, err);
  if (!resp) return false;
  return true;
}

bool HardwareInterface::setPID(int motor_index, const MotorPIDConfig& cfg, std::string& err) {
  uint8_t cmd = (motor_index == 1) ? CMD_SET_M1_PIDQ : CMD_SET_M2_PIDQ;
  // Datasheet: P,I,D,QPPS 32-bit each (P/I/D scaled by 65536). Big-endian.
  uint8_t payload[16];
  auto pack32 = [](uint8_t* p, int32_t v) {
    p[0] = (v >> 24) & 0xFF;
    p[1] = (v >> 16) & 0xFF;
    p[2] = (v >> 8) & 0xFF;
    p[3] = v & 0xFF;
  };
  pack32(&payload[0], static_cast<int32_t>(cfg.p * 65536.0));
  pack32(&payload[4], static_cast<int32_t>(cfg.i * 65536.0));
  pack32(&payload[8], static_cast<int32_t>(cfg.d * 65536.0));
  pack32(&payload[12], static_cast<int32_t>(cfg.qpps));
  return proto_.writePacket(cmd, payload, sizeof(payload), err);
}

bool HardwareInterface::driveMixedAccelDist(int32_t accel_qpps, int32_t m1_qpps, uint32_t m1_dist,
                                            int32_t m2_qpps, uint32_t m2_dist, std::string& err) {
  // Format assumption (verify): accel, m1_speed, m1_dist, m2_speed, m2_dist (all 32-bit)
  uint8_t payload[20];
  auto pack32 = [](uint8_t* p, int32_t v) {
    p[0] = (v >> 24) & 0xFF;
    p[1] = (v >> 16) & 0xFF;
    p[2] = (v >> 8) & 0xFF;
    p[3] = v & 0xFF;
  };
  auto packU32 = pack32;
  pack32(&payload[0], accel_qpps);
  pack32(&payload[4], m1_qpps);
  packU32(&payload[8], static_cast<int32_t>(m1_dist));
  pack32(&payload[12], m2_qpps);
  packU32(&payload[16], static_cast<int32_t>(m2_dist));
  return proto_.writePacket(CMD_MIXED_SPEED_ACCEL_DIST, payload, sizeof(payload), err);
}

bool HardwareInterface::driveSpeeds(int32_t m1_qpps, int32_t m2_qpps, std::string& err) {
  uint8_t payload[8];
  payload[0] = (m1_qpps >> 24) & 0xFF;
  payload[1] = (m1_qpps >> 16) & 0xFF;
  payload[2] = (m1_qpps >> 8) & 0xFF;
  payload[3] = m1_qpps & 0xFF;
  payload[4] = (m2_qpps >> 24) & 0xFF;
  payload[5] = (m2_qpps >> 16) & 0xFF;
  payload[6] = (m2_qpps >> 8) & 0xFF;
  payload[7] = m2_qpps & 0xFF;
  if (!proto_.writePacket(CMD_MIXED_SPEED, payload, sizeof(payload), err)) return false;
  setLastCommandSpeeds(m1_qpps, m2_qpps);
  return true;
}

bool HardwareInterface::stop(std::string& err) {
  return driveSpeeds(0, 0, err);
}

bool HardwareInterface::resetEncoders(std::string& err) {
  if (!proto_.writePacket(CMD_RESET_ENCODERS, nullptr, 0, err)) return false;
  std::lock_guard<std::mutex> lk(mtx_);
  cache_.m1_enc.value = 0;
  cache_.m2_enc.value = 0;
  cache_.m1_enc.speed_qpps = 0;
  cache_.m2_enc.speed_qpps = 0;
  return true;
}

std::optional<Snapshot> HardwareInterface::readSnapshot(std::string& err) {
  Snapshot snap;
  if (!readEncoders(snap, err)) return std::nullopt;
  if (!readCurrents(snap, err)) return std::nullopt;
  if (!readVoltages(snap, err)) return std::nullopt;
  if (!readTemps(snap, err)) return std::nullopt;
  if (!readStatusBits(snap, err)) return std::nullopt;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    cache_ = snap;
  }
  return snap;
}

bool HardwareInterface::readEncoders(Snapshot& snap, std::string& err) {
  auto r1 = proto_.transact(CMD_READ_ENCODER_M1, nullptr, 0, 5, 3, 0.05, err);
  if (!r1) return false;
  snap.m1_enc.value = (int64_t(r1->data[0]) << 24) | (int64_t(r1->data[1]) << 16) |
                      (int64_t(r1->data[2]) << 8) | r1->data[3];
  snap.m1_enc.status = r1->data[4];
  auto r2 = proto_.transact(CMD_READ_ENCODER_M2, nullptr, 0, 5, 3, 0.05, err);
  if (!r2) return false;
  snap.m2_enc.value = (int64_t(r2->data[0]) << 24) | (int64_t(r2->data[1]) << 16) |
                      (int64_t(r2->data[2]) << 8) | r2->data[3];
  snap.m2_enc.status = r2->data[4];
  // Speeds (instantaneous) if available
  auto s1 = proto_.transact(CMD_GET_M1_ISPEED, nullptr, 0, 5, 3, 0.05, err);
  if (s1) {
    int32_t raw = (int32_t(s1->data[0]) << 24) | (int32_t(s1->data[1]) << 16) |
                  (int32_t(s1->data[2]) << 8) | int32_t(s1->data[3]);
    snap.m1_enc.speed_qpps = raw;
  }
  auto s2 = proto_.transact(CMD_GET_M2_ISPEED, nullptr, 0, 5, 3, 0.05, err);
  if (s2) {
    int32_t raw = (int32_t(s2->data[0]) << 24) | (int32_t(s2->data[1]) << 16) |
                  (int32_t(s2->data[2]) << 8) | int32_t(s2->data[3]);
    snap.m2_enc.speed_qpps = raw;
  }
  return true;
}

bool HardwareInterface::readCurrents(Snapshot& snap, std::string& err) {
  auto rc = proto_.transact(CMD_GET_CURRENTS, nullptr, 0, 4, 3, 0.05, err);
  if (!rc) return false;
  uint16_t m1 = (uint16_t(rc->data[0]) << 8) | rc->data[1];
  uint16_t m2 = (uint16_t(rc->data[2]) << 8) | rc->data[3];
  snap.currents.m1_inst = m1 / 100.0;  // 0.01A units
  snap.currents.m2_inst = m2 / 100.0;
  return true;
}

bool HardwareInterface::readVoltages(Snapshot& snap, std::string& err) {
  auto vm = proto_.transact(CMD_GET_MBATT, nullptr, 0, 2, 3, 0.05, err);
  if (!vm) return false;
  uint16_t mv = (uint16_t(vm->data[0]) << 8) | vm->data[1];
  snap.volts.main = mv / 10.0;  // 0.1V units
  auto vl = proto_.transact(CMD_GET_LBATT, nullptr, 0, 2, 3, 0.05, err);
  if (vl) {
    uint16_t lv = (uint16_t(vl->data[0]) << 8) | vl->data[1];
    snap.volts.logic = lv / 10.0;
  }
  return true;
}

bool HardwareInterface::readTemps(Snapshot& snap, std::string& err) {
  auto t1 = proto_.transact(CMD_GET_TEMPERATURE, nullptr, 0, 2, 3, 0.05, err);
  if (t1) {
    uint16_t raw = (uint16_t(t1->data[0]) << 8) | t1->data[1];
    snap.temps.t1 = raw / 10.0;  // 0.1C
  }
  // Some boards have only one temp sensor; t2 left zero if not available
  return true;
}

bool HardwareInterface::readStatusBits(Snapshot& snap, std::string& err) {
  auto rs = proto_.transact(CMD_GET_ERROR, nullptr, 0, 4, 3, 0.05, err);
  if (!rs) return false;
  snap.status_bits = (uint32_t(rs->data[0]) << 24) | (uint32_t(rs->data[1]) << 16) |
                     (uint32_t(rs->data[2]) << 8) | uint32_t(rs->data[3]);
  return true;
}

}  // namespace roboclaw_driver
