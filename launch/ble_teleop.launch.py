#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ble_teleop',
            executable='ble_cmd_vel_node',
            name='ble_cmd_vel_publisher',
            output='screen',
            parameters=[
                {'linear_speed': 0.2},  # m/s
                {'angular_speed': 0.3}  # rad/s
            ]
        )
    ])
