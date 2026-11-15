#!/usr/bin/env python3
"""display.launch.py for mearm_description.

Delegates to mearm_bringup/bringup.launch.py so the field command works:
  ros2 launch mearm_description display.launch.py \
      use_mock_hardware:=false serial_port:=/dev/ttyACM0 start_rviz:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("use_mock_hardware", default_value="true"),
        DeclareLaunchArgument("start_rviz", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0"),
    ]

    bringup_launch = PathJoinSubstitution(
        [FindPackageShare("mearm_bringup"), "launch", "bringup.launch.py"]
    )

    include_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch),
        launch_arguments={
            "use_mock_hardware": LaunchConfiguration("use_mock_hardware"),
            "start_rviz": LaunchConfiguration("start_rviz"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "serial_port": LaunchConfiguration("serial_port"),
        }.items(),
    )

    return LaunchDescription(declared_arguments + [include_bringup])
