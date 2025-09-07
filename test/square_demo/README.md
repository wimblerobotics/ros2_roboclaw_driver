# Square Test Program

This test program drives the robot in a square pattern using odometry feedback to verify the RoboClaw driver functionality.

## Overview

The `square_runner_test` program:
- Drives the robot in a square pattern of configurable size
- Uses `/odom` topic for position and orientation feedback
- Publishes to `/cmd_vel` topic to control robot movement
- Implements a state machine to manage forward movement and turning
- Provides real-time feedback on test progress

## Features

- **Configurable square size**: Default 0.5m, can be specified as command line argument
- **Odometry-based navigation**: Uses encoder feedback for precise movement
- **Safety tolerances**: 5cm position tolerance, ~5° angular tolerance
- **State machine control**: Clean transitions between moving and turning
- **Real-time logging**: Progress updates during test execution

## Prerequisites

1. **RoboClaw driver running**:
   ```bash
   ros2 launch ros2_roboclaw_driver ros2_roboclaw_driver.launch.py
   ```

2. **Robot on flat surface**: Ensure adequate space for the square pattern

3. **Proper calibration**: Verify wheel parameters are correctly configured

## Usage

### Using Direct Executable

#### Basic Usage (0.5m square)
```bash
# Run with default 0.5m square
ros2 run ros2_roboclaw_driver square_runner_test
```

#### Custom Square Size
```bash
# Run with 1.0m square
ros2 run ros2_roboclaw_driver square_runner_test 1.0

# Run with 0.25m square
ros2 run ros2_roboclaw_driver square_runner_test 0.25
```

### Using Launch File (Recommended)

#### Basic Usage (0.5m square)
```bash
# Run with default 0.5m square
ros2 launch ros2_roboclaw_driver square_test.launch.py
```

#### Custom Square Size
```bash
# Run with 1.0m square
ros2 launch ros2_roboclaw_driver square_test.launch.py square_size:=1.0

# Run with 0.25m square
ros2 launch ros2_roboclaw_driver square_test.launch.py square_size:=0.25
```

### Complete Test Procedure

1. **Start the RoboClaw driver**:
   ```bash
   cd /path/to/your/workspace
   source install/setup.bash
   ros2 launch ros2_roboclaw_driver ros2_roboclaw_driver.launch.py
   ```

2. **In a new terminal, run the test**:
   ```bash
   cd /path/to/your/workspace
   source install/setup.bash
   ros2 run ros2_roboclaw_driver square_runner_test [square_size]
   ```

3. **Monitor test progress**:
   - Watch console output for state transitions
   - Observe robot movement
   - Test completes automatically after 4 sides

4. **Stop if needed**:
   - Press `Ctrl+C` to stop the test at any time
   - Robot will stop immediately

## Expected Behavior

1. **Initialization**: Program waits for initial odometry data
2. **Side 1**: Robot moves forward for specified distance
3. **Turn 1**: Robot turns left 90°
4. **Sides 2-4**: Repeat forward movement and left turns
5. **Completion**: Robot stops and reports successful completion

## Test Parameters

| Parameter | Default Value | Description |
|-----------|---------------|-------------|
| Square size | 0.5 m | Side length of the square |
| Linear speed | 0.2 m/s | Forward movement speed |
| Angular speed | 0.5 rad/s | Turning speed (~28.6°/s) |
| Position tolerance | 0.05 m | Distance accuracy (5 cm) |
| Angular tolerance | 0.087 rad | Turning accuracy (~5°) |
| Control frequency | 20 Hz | Control loop update rate |

## Troubleshooting

### Robot doesn't move
- Check that RoboClaw driver is running
- Verify `/cmd_vel` topic is being published: `ros2 topic echo /cmd_vel`
- Ensure robot has sufficient battery power

### Poor square accuracy
- Check wheel parameter calibration in `motor_driver.yaml`
- Verify odometry is publishing: `ros2 topic echo /odom`
- Ensure robot is on a flat, non-slip surface

### Test gets stuck
- Check for odometry data: `ros2 topic hz /odom`
- Verify no obstacles are blocking robot movement
- Stop test with `Ctrl+C` and restart if needed

### Error messages
- **"Waiting for initial odometry"**: Normal, wait for driver to start publishing
- **"Error parsing square size"**: Check command line argument is a valid number
- **"Square size must be positive"**: Use a positive value for square size

## Integration Testing

This test verifies:
- ✅ **Motor control**: Forward movement with proper speed control
- ✅ **Encoder feedback**: Accurate distance measurement via odometry
- ✅ **Turning control**: Precise 90° rotations
- ✅ **State management**: Clean transitions between movement phases
- ✅ **Topic communication**: Proper `/cmd_vel` publishing and `/odom` subscription

## Build Information

The test is automatically built with the main package. If you need to rebuild:

```bash
cd /path/to/your/workspace
colcon build --packages-select ros2_roboclaw_driver
```

The executable will be installed to `install/ros2_roboclaw_driver/lib/ros2_roboclaw_driver/square_runner_test`.
