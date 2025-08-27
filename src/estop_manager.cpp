// MIT License
// Author: Michael Wimble <mike@wimblerobotics.com>
#include "roboclaw_driver/estop_manager.hpp"
namespace roboclaw_driver {
bool EStopManager::set(const std::string& source, const std::string& reason) {
  auto it = sources_.find(source);
  if (it == sources_.end()) {
    sources_[source] = {reason};
    return true;
  } else {
    // update reason
    it->second.reason = reason;
    return false;
  }
}
bool EStopManager::clear(const std::string& source) {
  return sources_.erase(source) > 0;
}
void EStopManager::clearAll() {
  sources_.clear();
}
std::vector<std::string> EStopManager::activeSources() const {
  std::vector<std::string> v;
  v.reserve(sources_.size());
  for (auto& p : sources_) v.push_back(p.first);
  return v;
}
std::vector<std::string> EStopManager::reasons() const {
  std::vector<std::string> v;
  v.reserve(sources_.size());
  for (auto& p : sources_) v.push_back(p.second.reason);
  return v;
}
}  // namespace roboclaw_driver
