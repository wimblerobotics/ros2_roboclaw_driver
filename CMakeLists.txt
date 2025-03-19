cmake_minimum_required(VERSION 3.5)
project(ros2_roboclaw_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Enforce C++17 or higher
set(CMAKE_CXX_STANDARD 17) # or 14 or 17, etc.
set(CMAKE_CXX_STANDARD_REQUIRED ON) # Ensure the standard is enforced

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-g)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  ${geometry_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  /usr/include # Ensure standard system headers are included
)

set(msg_files
  "msg/RoboClawStatus.msg"
)

set(srv_files
  "srv/ResetEncoders.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(
  ros2_roboclaw_driver_node
  src/motor_driver_node.cpp
  src/motor_driver.cpp
  src/roboclaw.cpp
)

ament_target_dependencies(
  ros2_roboclaw_driver_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
)

target_link_libraries(ros2_roboclaw_driver_node "${cpp_typesupport_target}")

# Install config files.
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  ros2_roboclaw_driver_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
