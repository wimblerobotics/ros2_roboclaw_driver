import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions
from launch_ros.actions import Node


def generate_launch_description():
    # The following 'my_package_name' should be changed to the
    # package name in your robot workspace that will contain
    # your own yaml configuration file.
    #
    # Also the 'motor_driver.yaml' should be changed to the yaml
    # configuration file name in your robot package. The example
    # launch file here assumes that your configuration file will
    # be under the 'config' directory within the my_package_name
    # folder.
    #
    # The name 'ros2_roboclaw_driver'
    # here will point to this workspace and its example configuration
    # file which is unlikely to work out-of-the-box for your hardware
    # so it is expected that your will also create a yaml file to
    # match your hardware. See the README file for more information.

    my_package_name = 'ros2_roboclaw_driver'
    configFilePath = os.path.join(
        get_package_share_directory(my_package_name),
        'config',
        'motor_driver.yaml'
    )

    # Extract the relevant configuration parameters from the yaml file.
    # Doing it this way allows you, for example, to include the configuration
    # parameters in a larger yaml file that also provides parameters for
    # other packages. See the example yaml file provided and the README
    # file for more information.

    with open(configFilePath, 'r') as file:
        configParams = yaml.safe_load(
            file)['motor_driver_node']['ros__parameters']

    ld = LaunchDescription()

    # The 'emulate_tty' here helps colorize the log output to the console
    motor_driver_node = Node(
        emulate_tty=True,
        executable='roboclaw_driver',
        package='ros2_roboclaw_driver',
        parameters=[configParams],
        # prefix=['xterm -e gdb -ex run --args'],
        respawn=True,
        output='screen')
    ld.add_action(motor_driver_node)
    return ld
