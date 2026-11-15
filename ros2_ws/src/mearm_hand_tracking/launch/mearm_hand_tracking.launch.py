#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('mearm_hand_tracking')
    default_params = os.path.join(pkg_share, 'config', 'mearm_hand_tracking_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_index',
            default_value='5',
            description='V4L2 index of the USB camera to open (index 0 is the '
                        'Surface IR/Windows-Hello sensor on this machine; the '
                        'real RGB "Integrated Camera" is index 4)'),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='YAML file with hand-tracking / HSV parameters'),

        Node(
            package='mearm_hand_tracking',
            executable='mearm_hand_tracking_node',
            name='mearm_hand_tracking_node',
            parameters=[
                LaunchConfiguration('params_file'),
                {'camera_index': LaunchConfiguration('camera_index')},
            ],
            output='screen'),
    ])
