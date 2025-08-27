// MIT License
// Author: Michael Wimble <mike@wimblerobotics.com>
#include "roboclaw_driver/status_decoder.hpp"

#include <sstream>
namespace roboclaw_driver {
StatusDecoder::StatusDecoder() {
  bit_table_ = {
      {0x00000001, "ERROR_ESTOP", false},
      {0x00000002, "ERROR_TEMP", false},
      {0x00000004, "ERROR_TEMP2", false},
      {0x00000010, "ERROR_LBATHIGH", false},
      {0x00000020, "ERROR_LBATLOW", false},
      {0x00000040, "ERROR_FAULTM1", false},
      {0x00000080, "ERROR_FAULTM2", false},
      {0x00000100, "ERROR_SPEED1", false},
      {0x00000200, "ERROR_SPEED2", false},
      {0x00000400, "ERROR_POS1", false},
      {0x00000800, "ERROR_POS2", false},
      {0x00001000, "ERROR_CURRENTM1", false},
      {0x00002000, "ERROR_CURRENTM2", false},
      {0x00010000, "WARN_OVERCURRENTM1", true},
      {0x00020000, "WARN_OVERCURRENTM2", true},
      {0x00040000, "WARN_MBATHIGH", true},
      {0x00080000, "WARN_MBATLOW", true},
      {0x00100000, "WARN_TEMP", true},
      {0x00200000, "WARN_TEMP2", true},
      {0x00400000, "WARN_S4", true},
      {0x00800000, "WARN_S5", true},
      {0x10000000, "WARN_CAN", true},
      {0x20000000, "WARN_BOOT", true},
      {0x40000000, "WARN_OVERREGENM1", true},
      {0x80000000, "WARN_OVERREGENM2", true},
  };
}
std::string StatusDecoder::toJson(uint32_t bits) const {
  std::ostringstream oss;
  oss << '{';
  bool first = true;
  for (auto& b : bit_table_) {
    if (bits & b.mask) {
      if (!first)
        oss << ',';
      else
        first = false;
      oss << '"' << b.name << '"' << ':' << '1';
    }
  }
  // Add unknown bits if any
  uint32_t known = 0;
  for (auto& b : bit_table_) known |= b.mask;
  uint32_t unknown = bits & ~known;
  if (unknown) {
    if (!first)
      oss << ',';
    else
      first = false;
    oss << "\"UNKNOWN_0x" << std::hex << unknown << "\":1";
  }
  oss << '}';
  return oss.str();
}
std::vector<std::string> StatusDecoder::names(uint32_t bits) const {
  std::vector<std::string> out;
  out.reserve(8);
  for (auto& b : bit_table_)
    if (bits & b.mask) out.emplace_back(b.name);
  uint32_t known = 0;
  for (auto& b : bit_table_) known |= b.mask;
  uint32_t unknown = bits & ~known;
  if (unknown) {
    std::ostringstream oss;
    oss << "UNKNOWN_0x" << std::hex << unknown;
    out.push_back(oss.str());
  }
  return out;
}
}  // namespace roboclaw_driver
