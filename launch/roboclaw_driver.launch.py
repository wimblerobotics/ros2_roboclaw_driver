import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Get the new config file for the roboclaw_driver architecture
    package_name = 'ros2_roboclaw_driver'
    config_file_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'roboclaw_driver.yaml'
    )

    # Load the configuration parameters
    with open(config_file_path, 'r') as file:
        config_params = yaml.safe_load(file)['roboclaw_driver']['ros__parameters']

    ld = LaunchDescription()

    # Launch the new roboclaw_driver node
    roboclaw_driver_node = Node(
        package='ros2_roboclaw_driver',
        executable='roboclaw_driver',
        name='roboclaw_driver',
        parameters=[config_params],
        output='screen',
        emulate_tty=True,
        respawn=True,
        # Uncomment the following line for GDB debugging:
        # prefix=['xterm -e gdb -ex run --args'],
        # Uncomment the following line for Valgrind debugging:
        # prefix=['valgrind --tool=memcheck --leak-check=full --show-leak-kinds=all'],
    )

    ld.add_action(roboclaw_driver_node)
    return ld
