#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='qr_code_scanner',
            executable='qr_detector',
            name='qr_detector_node',
            output='screen',
            parameters=[
                {'use_sim_time': True}
            ]
        ),
        Node(
            package='qr_code_scanner',
            executable='image_recorder',
            name='image_recorder_node',
            output='screen',
            parameters=[
                {'use_sim_time': True}
            ]
        ),
    ])
