#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('solar_hunter_cpp')
    default_params = os.path.join(pkg_share, 'config', 'solar_hunter_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='YAML file with solar-hunter parameters'),
        DeclareLaunchArgument(
            'target',
            default_value='meteors',
            description='Named aim direction: locusts, meteors, space_debris, '
                        'moon_south_darkside, solar_sail'),
        DeclareLaunchArgument(
            'camera_index',
            default_value='4',
            description='V4L2 index of the camera for the focus-centering loop '
                        '(set to -1 to run aim-only, no camera)'),
        DeclareLaunchArgument(
            'pan_joint_index',
            default_value='0',
            description='Index of the pan joint in /joint_states position[]'),
        DeclareLaunchArgument(
            'tilt_joint_index',
            default_value='1',
            description='Index of the tilt joint in /joint_states position[]'),
        DeclareLaunchArgument(
            'show_debug_windows',
            default_value='true',
            description='Show OpenCV debug windows'),
        DeclareLaunchArgument(
            'auto_target',
            default_value='true',
            description='Stage 1: retarget automatically from the object-detection '
                        'classifier (/object_detection/detections) as it classifies '
                        'space_debris, meteors, locusts, solar_sail, or '
                        'moon_south_darkside'),
        DeclareLaunchArgument(
            'target_is_manual',
            default_value='false',
            description='Set true to lock onto the --target/target param and ignore '
                        'the object-detection classifier entirely'),

        Node(
            package='solar_hunter_cpp',
            executable='solar_hunter_cpp_node',
            name='solar_hunter',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'target': LaunchConfiguration('target'),
                    'camera_index': LaunchConfiguration('camera_index'),
                    'pan_joint_index': LaunchConfiguration('pan_joint_index'),
                    'tilt_joint_index': LaunchConfiguration('tilt_joint_index'),
                    'show_debug_windows': LaunchConfiguration('show_debug_windows'),
                    'auto_target': LaunchConfiguration('auto_target'),
                    'target_is_manual': LaunchConfiguration('target_is_manual'),
                },
            ],
            output='screen'),
    ])
