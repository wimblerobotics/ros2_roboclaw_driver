import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments for debugging
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable GDB debugging'
    )
    
    valgrind_arg = DeclareLaunchArgument(
        'valgrind',
        default_value='false',
        description='Enable Valgrind memory debugging'
    )

    # Get the config file
    package_name = 'ros2_roboclaw_driver'
    config_file_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'roboclaw_driver.yaml'
    )

    # Load the configuration parameters
    with open(config_file_path, 'r') as file:
        config_params = yaml.safe_load(file)['roboclaw_driver']['ros__parameters']

    # Add debug logging parameters
    config_params.update({
        'log_level': 'DEBUG',
        'use_sim_time': False,
    })

    ld = LaunchDescription()
    ld.add_action(debug_arg)
    ld.add_action(valgrind_arg)

    # Create the node with conditional debugging
    roboclaw_driver_node = Node(
        package='ros2_roboclaw_driver',
        executable='roboclaw_driver',
        name='roboclaw_driver',
        parameters=[config_params],
        output='screen',
        emulate_tty=True,
        respawn=False,  # Don't respawn when debugging
        # Enable verbose logging
        arguments=['--ros-args', '--log-level', 'DEBUG'],
        # GDB debugging (uncomment next line and comment the one after to enable)
        # prefix=['xterm -e gdb -ex "set confirm off" -ex "run" -ex "bt" --args'],
        prefix=[],
        # Valgrind debugging (uncomment to enable)
        # prefix=['valgrind', '--tool=memcheck', '--leak-check=full', '--show-leak-kinds=all', '--track-origins=yes'],
    )

    ld.add_action(roboclaw_driver_node)
    return ld
