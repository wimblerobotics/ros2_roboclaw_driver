# ros2_roboclaw_driver
This is a ROS2 (Jazzy) driver for BasicMicro RoboClaw motor controllers using a direct, minimal interface. The code now talks straight to the RoboClaw over a POSIX serial device (USB CDC ACM or UART) without intermediate abstraction layers.

## Prerequisites
- ROS2 Jazzy
- git

## Build
```
mkdir -p ~/ros2_roboclaw_driver/src
cd ~/ros2_roboclaw_driver/src
git clone https://github.com/wimblerobotics/ros2_roboclaw_driver.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/local_setup.bash
```

Debug symbols are always enabled (-g -ggdb3) for easier troubleshooting.

## Topics
- /cmd_vel (subscription) geometry_msgs/Twist
- /roboclaw/status (publisher) ros2_roboclaw_driver/RoboClawStatus
- /odom (optional) nav_msgs/Odometry
- /joint_states (optional) sensor_msgs/JointState
- /roboclaw/estop (publish / subscribe)

## Services
- /roboclaw/reset_encoders
- /roboclaw/clear_all_estops
- /roboclaw/clear_estop_source

## Status Message (excerpt)
Refer to msg/RoboClawStatus.msg for full field list. Includes encoder values, currents, voltages, temperatures, error bits.

## Configuration
See config/motor_driver.yaml for parameters: device_name, baud_rate (ignored for USB CDC), accel, wheel geometry, pulses per meter / revolution, publish_* toggles, safety timeouts.

## Direct Device Layer
The previous transport / protocol / hardware_interface abstractions were removed. All low level interaction is in:
- include/roboclaw_driver/roboclaw_device.hpp
- src/roboclaw_device.cpp

Implemented commands (address+command framing with optional CRC on RX / TX per RoboClaw manual):
- 20 Reset Encoders
- 21 Read Firmware Version (variable length + CRC)
- 22/23 Read M1/M2 Encoder (value + status + CRC)
- 24/25 Read Main/Logic Battery Voltage
- 28/29 Set M1/M2 PID (with P,I,D,QPPS payload)
- 30/31 Read M1/M2 Speed (value + direction byte + CRC)
- 37 Mixed Speed (drive two speeds)
- 46 Mixed Speed Accel Distance (TODO: add if needed)
- 49 Read Currents
- 82 Read Temperature
- 90 Read Error Status (32-bit + CRC)

## Safety
Driver applies command timeout; if no /cmd_vel within configured window motors are commanded to zero. Additional higher level safety logic hooks remain (EStop manager & safety supervisor) but can be extended.

## Code Organization
- config/ : YAML parameter file(s)
- include/roboclaw_driver/ : headers (device + utility components)
- msg/, srv/ : custom interfaces
- src/ : node, device, support modules

Removed files: protocol.*, transport.*, hardware_interface.* (legacy abstraction) and motor_driver.* (superseded). Stubs remain only if external packages still include old headers; they will be deleted in a future cleanup.

## UART vs USB
Specify device_name (e.g. /dev/ttyACM0 for USB, /dev/ttyAMA0 for Pi UART). For UART ensure baud_rate matches RoboClaw configuration. For USB the baud_rate parameter is ignored.

## Encoder Reset
Encoders are reset on explicit /roboclaw/reset_encoders call. (Previously auto-reset on startup—now explicit.)

## Firmware Version
Queried during device construction (future improvement: log or publish).

## Future TODO
- Implement buffered accel-distance motion command (46) in new device layer
- Extend safety supervisor integration directly on raw snapshot
- Publish firmware version field in status
- Optional CRC / I/O error counters

## Removed / Deprecated Parameters

The following parameters have been removed as of the rearchitect branch cleanup:

- `stall_speed_ratio`
- `stall_min_command`
- `stall_timeout`
- `estop_auto_clear`

Buffered distance‑limited drive commands provide adequate protection against run-on motion during `cmd_vel` gaps, so stall detection was eliminated to reduce false estops and simplify the safety model. Any references in older configs should be deleted; they are silently ignored if present.

## License
MIT
