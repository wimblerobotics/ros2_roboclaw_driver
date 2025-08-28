#!/bin/bash

echo "=== RoboClaw Device Debugging Script ==="
echo

echo "1. Checking if /dev/roboclaw exists:"
if [ -e "/dev/roboclaw" ]; then
    echo "✓ /dev/roboclaw exists"
    ls -la /dev/roboclaw
    echo
    echo "2. Checking if it's a character device:"
    if [ -c "/dev/roboclaw" ]; then
        echo "✓ /dev/roboclaw is a character device"
    else
        echo "✗ /dev/roboclaw is NOT a character device"
        file /dev/roboclaw
    fi
    echo
    echo "3. Checking permissions:"
    if [ -r "/dev/roboclaw" ] && [ -w "/dev/roboclaw" ]; then
        echo "✓ /dev/roboclaw has read/write permissions"
    else
        echo "✗ /dev/roboclaw missing read/write permissions"
        echo "Current user: $(whoami)"
        echo "Groups: $(groups)"
    fi
else
    echo "✗ /dev/roboclaw does not exist"
    echo
    echo "Checking for similar devices:"
    ls -la /dev/tty* | grep -E "(USB|ACM)" || echo "No USB/ACM devices found"
    echo
    echo "Checking for RoboClaw-like devices:"
    lsusb | grep -i "roboclaw\|basicmicro\|03eb:2404" || echo "No RoboClaw USB devices found"
fi

echo
echo "4. Available serial devices:"
ls -la /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "No ttyUSB* or ttyACM* devices found"

echo
echo "5. USB device tree:"
lsusb

echo
echo "6. dmesg for recent USB/serial activity:"
dmesg | tail -20 | grep -i "usb\|tty\|serial" || echo "No recent USB/serial activity"
