#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('mearm_visual_servoing')
    default_params = os.path.join(
        pkg_share, 'config', 'mearm_visual_servoing_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='YAML file with visual servoing parameters'),

        Node(
            package='mearm_visual_servoing',
            executable='mearm_visual_servoing_node',
            name='mearm_visual_servoing_node',
            parameters=[
                LaunchConfiguration('params_file'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            output='screen'),
    ])
