# SPDX-License-Identifier: Apache-2.0
# Copyright 2025 Wimblerobotics
# https://github.com/wimblerobotics/ros2_roboclaw_driver

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    square_size_arg = DeclareLaunchArgument(
        'square_size',
        default_value='0.5',
        description='Side length of the square in meters'
    )

    square_runner_node = Node(
        package='ros2_roboclaw_driver',
        executable='square_runner_test',
        name='square_runner_test',
        arguments=[LaunchConfiguration('square_size')],
        # remappings=[
        #         ('/odom', '/sigyn/wheel_odom'),
        #     ],
        output='screen'
    )

    return LaunchDescription([
        square_size_arg,
        LogInfo(msg=['Starting square test with size: ', LaunchConfiguration('square_size'), ' meters']),
        square_runner_node
    ])
