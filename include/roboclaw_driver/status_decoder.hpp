// MIT License
// Author: Michael Wimble <mike@wimblerobotics.com>
#pragma once
#include <cstdint>
#include <map>
#include <string>
#include <vector>
namespace roboclaw_driver {
class StatusDecoder {
 public:
  struct BitInfo {
    uint32_t mask;
    const char* name;
    bool warning;
  };
  StatusDecoder();
  std::string toJson(uint32_t bits) const;              // JSON of true bits
  std::vector<std::string> names(uint32_t bits) const;  // list of set bit names
 private:
  std::vector<BitInfo> bit_table_;
};
}  // namespace roboclaw_driver
