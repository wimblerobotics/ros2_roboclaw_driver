# Test Utilities for ros2_roboclaw_driver

## Passive Golden Data Capture (Option A)

Purpose: Collect synchronized RoboClawStatus + Odometry samples WITHOUT opening the serial port a second time. Ensures single-owner access (the main roboclaw_driver node owns the device). Output is a CSV suitable for golden vector and unit tests.

### Build
```
colcon build --packages-select ros2_roboclaw_driver
source install/setup.bash
```

### Run Driver
Launch (example):
```
ros2 run ros2_roboclaw_driver roboclaw_driver --ros-args \
  -p port:=/dev/roboclaw -p baud_rate:=230400 -p device_address:=128 \
  -p publish_odom:=true -p publish_joint_states:=true -p publish_tf:=false
```
Confirm /roboclaw/status and /odom are publishing.

### Run Passive Capture
```
ros2 run ros2_roboclaw_driver passive_capture --ros-args -p sample_limit:=200 -p output_path:=golden_200
```
This creates golden_200/status_samples.csv with columns:
```
t,odom_x,odom_y,odom_yaw,m1_cmd,m1_meas,m1_enc,m2_cmd,m2_meas,m2_enc,main_v,logic_v,temp1,temp2,error_bits,crc_errors,io_errors,retries
```

## Square Path Functional Test
Demonstrates closed-loop usage of cmd_vel to drive a 0.5m square, relying solely on odometry feedback to trigger stops and 90° turns.

### Run
With the driver already running (as above):
```
ros2 run ros2_roboclaw_driver square_runner --ros-args \
  -p side_length:=0.5 -p linear_speed:=0.15 -p angular_speed:=0.6 \
  -p stop_margin:=0.01 -p angle_margin:=0.02
```
Node logic:
1. Move forward until traveled distance >= side_length - stop_margin.
2. Stop, then rotate left until yaw change >= 90° - angle_margin.
3. Repeat 4 edges then shutdown.

### Suggested Improvements
- Add dynamic speed smoothing (ramp down near target distance / angle).
- Validate encoder delta vs odom delta per edge; write per-edge metrics to CSV.
- Assert commanded speed vs measured (within tolerance) and track max error.
- Promote into arostest/launch for automated regression with a simulated transport.

## Next Steps
1. Integrate command stats (crc / io / retry) into RoboClawStatus if not already.
2. Provide a mock transport and replay of golden status_samples.csv for deterministic unit tests.
3. Extend square_runner to log per-edge diagnostics for automated pass/fail criteria.

## Notes
- Only one process should own the serial device. Do NOT run capture_harness concurrently with passive_capture.
- For motion accuracy tuning, ensure pulses_per_meter and wheel_separation parameters are correctly set.
