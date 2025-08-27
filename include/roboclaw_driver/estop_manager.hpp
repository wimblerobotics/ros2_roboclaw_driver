// MIT License
// Author: Michael Wimble <mike@wimblerobotics.com>
#pragma once
#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <vector>
namespace roboclaw_driver {
class EStopManager {
 public:
  enum class SafetyState : uint8_t { OK = 0, DECEL = 1, ESTOP = 2 };
  struct SourceInfo {
    std::string reason;
  };  // future: timestamp
  bool set(const std::string& source, const std::string& reason);  // returns true if new
  bool clear(const std::string& source);                           // true if existed
  void clearAll();
  bool hasAny() const {
    return !sources_.empty();
  }
  std::vector<std::string> activeSources() const;
  std::vector<std::string> reasons() const;
  SafetyState state() const {
    return hasAny() ? SafetyState::ESTOP : SafetyState::OK;
  }

 private:
  std::map<std::string, SourceInfo> sources_;
};
}  // namespace roboclaw_driver
