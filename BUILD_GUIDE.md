# RoboClaw Driver Build Commands

## Recommended Commands (with symlink-install and debug symbols)

### Build for debugging (VS Code)
```bash
colcon build --packages-select ros2_roboclaw_driver --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

### Build for release (but still with debug symbols)
```bash
colcon build --packages-select ros2_roboclaw_driver --symlink-install
```

### Clean build
```bash
rm -rf build/ros2_roboclaw_driver install/ros2_roboclaw_driver
```

## Benefits of --symlink-install

- **Faster iteration**: Changes to launch files, config files, and other resources are immediately available without rebuilding
- **No copying**: Creates symbolic links instead of copying files, saving disk space
- **Real-time updates**: Modify YAML configs, launch files, etc. and they take effect immediately

## Debug Symbol Configuration

The CMakeLists.txt now:
- Always generates debug symbols (`-g -ggdb3`)
- Preserves debug symbols even in release builds
- Disables optimizations in debug builds (`-O0`) for accurate debugging
- Generates compile_commands.json for better IntelliSense

## VS Code Integration

All build tasks now use `--symlink-install` by default:
- `build_roboclaw_driver`: Debug build with symlink-install
- `build_roboclaw_driver_release`: Release build with symlink-install
- Both preserve debug symbols for optimal debugging experience
