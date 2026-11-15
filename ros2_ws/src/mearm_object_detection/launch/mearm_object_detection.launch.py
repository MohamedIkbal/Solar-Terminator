#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('mearm_object_detection')
    default_params = os.path.join(
        pkg_share, 'config', 'mearm_object_detection_params.yaml')
    default_model = os.path.join(
        pkg_share, 'models', 'space_object_classifier.onnx')
    default_labels = os.path.join(pkg_share, 'models', 'labels.txt')

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_index',
            default_value='4',
            description='V4L2 index of the USB camera to open. Confirmed via '
                        '`v4l2-ctl --device=/dev/videoN --all`: video4 is the '
                        'real YUYV capture node on the "Integrated Camera"; '
                        'video5 is a metadata-only node and cannot be opened '
                        'for capture. Re-check if the device re-enumerates.'),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='YAML file with detector parameters'),
        DeclareLaunchArgument(
            'model_path',
            default_value=default_model,
            description='Path to the ONNX space-object classifier'),
        DeclareLaunchArgument(
            'labels_path',
            default_value=default_labels,
            description='Path to the newline-separated class labels file'),
        DeclareLaunchArgument(
            'unknown_image_dir',
            default_value=os.path.expanduser(
                '~/ros2_ws/src/mearm_object_detection/training_images/unknown'),
            description='Directory where low-confidence "unknown" crops are saved '
                        '(empty string disables saving)'),

        Node(
            package='mearm_object_detection',
            executable='mearm_object_detection_node',
            name='mearm_object_detection_node',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'camera_index': LaunchConfiguration('camera_index'),
                    'model_path': LaunchConfiguration('model_path'),
                    'labels_path': LaunchConfiguration('labels_path'),
                    'unknown_image_dir': LaunchConfiguration('unknown_image_dir'),
                },
            ],
            output='screen'),
    ])
