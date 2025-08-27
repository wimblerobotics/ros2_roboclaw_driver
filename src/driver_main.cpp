// MIT License
// Author: Michael Wimble <mike@wimblerobotics.com>
#include <rclcpp/rclcpp.hpp>

#include "roboclaw_driver/driver_node.hpp"
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<roboclaw_driver::DriverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
