#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0',
                              description='Serial port of the STM32'),
        DeclareLaunchArgument('baud', default_value='115200',
                              description='Serial baud rate'),
        DeclareLaunchArgument('enable_scurve', default_value='true',
                              description='Enable S-curve smoothing'),
        DeclareLaunchArgument('scurve_duration_s', default_value='0.3',
                              description='S-curve move duration (s)'),
        DeclareLaunchArgument('scurve_update_hz', default_value='50.0',
                              description='S-curve update rate (Hz)'),
        DeclareLaunchArgument('max_write_hz', default_value='10.0',
                              description='Max serial writes per joint per second'),
        DeclareLaunchArgument('min_change_rad', default_value='0.02',
                              description='Min rad change before re-sending'),
        Node(
            package='pca9685_multiples',
            executable='servo_serial_bridge',
            name='servo_serial_bridge',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baud': LaunchConfiguration('baud'),
                'enable_scurve': LaunchConfiguration('enable_scurve'),
                'scurve_duration_s': LaunchConfiguration('scurve_duration_s'),
                'scurve_update_hz': LaunchConfiguration('scurve_update_hz'),
                'max_write_hz': LaunchConfiguration('max_write_hz'),
                'min_change_rad': LaunchConfiguration('min_change_rad'),
            }],
            output='screen',
        ),
    ])