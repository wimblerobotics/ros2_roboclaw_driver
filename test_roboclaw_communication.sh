#!/bin/bash

echo "=== RoboClaw Communication Diagnostics ==="
echo "Testing different baud rates and addresses..."
echo

# Common baud rates for RoboClaw
BAUD_RATES=(9600 19200 38400 57600 115200)
# Common addresses (128 is default, but let's test a few)
ADDRESSES=(128 129 130 131)

for baud in "${BAUD_RATES[@]}"; do
    echo "Testing baud rate: $baud"
    for addr in "${ADDRESSES[@]}"; do
        echo "  Testing address: $addr"
        
        # Create temporary config
        cat > /tmp/test_config.yaml << EOF
roboclaw_driver:
  ros__parameters:
    device_name: "/dev/roboclaw"
    baud_rate: $baud
    device_port: $addr
    accel_quad_pulses_per_second: 1000.0
    max_linear_velocity: 0.3
    max_angular_velocity: 0.07
    max_seconds_uncommanded_travel: 0.25
    wheel_radius: 0.10169
    wheel_separation: 0.345
    quad_pulses_per_meter: 1566
    quad_pulses_per_revolution: 1566.0
    publish_odom: true
    publish_joint_states: true
    publish_tf: true
    safety_enabled: false
    overcurrent_limit_m1: 5.0
    overcurrent_limit_m2: 5.0
    overcurrent_detect_time: 0.1
    overcurrent_clear_time: 1.0
    overcurrent_hysteresis: 0.5
    temp1_limit: 70.0
    temp2_limit: 70.0
    temp_clear_delta: 5.0
    runaway_speed_factor: 2.0
    runaway_detect_time: 0.5
    stall_speed_ratio: 0.1
    stall_min_command: 0.1
    stall_timeout: 1.0
    estop_auto_clear: false
EOF
        
        # Test this configuration with a 3-second timeout
        timeout 3s ros2 run ros2_roboclaw_driver roboclaw_driver --ros-args --params-file /tmp/test_config.yaml --log-level ERROR 2>&1 | grep -E "(Hardware init|Failed to open|crc mismatch)" &
        
        # Wait for the test to complete
        wait
        
        # Check if it succeeded (no error output means success)
        if [ $? -eq 0 ]; then
            echo "    ✓ SUCCESS: Baud $baud, Address $addr"
            echo "    Working configuration found!"
            echo "    Update your config file with: baud_rate: $baud, device_port: $addr"
            exit 0
        else
            echo "    ✗ Failed"
        fi
    done
    echo
done

echo "No working configuration found. Possible issues:"
echo "1. RoboClaw not connected properly"
echo "2. RoboClaw in different communication mode"
echo "3. Hardware issues"
echo "4. RoboClaw firmware needs configuration"
