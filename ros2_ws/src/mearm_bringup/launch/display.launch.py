#!/usr/bin/env python3
"""
display.launch.py
=================
Stage 1 — RViz-only visualization (NO ros2_control, NO hardware required).

Launches:
  1. robot_state_publisher  — publishes /tf from URDF
  2. joint_state_publisher_gui — GUI sliders for the 3 driven joints
                                 joint4_wrist_pitch is computed automatically
                                 from the <mimic> constraint in the URDF
  3. rviz2 — visualizer

Usage (inside Docker container):
  ros2 launch mearm_bringup display.launch.py

Note: joint4_wrist_pitch is a passive mimic joint. The GUI will show a slider
for it but moving it is meaningless — robot_state_publisher will override its
value based on joint3's position via the URDF <mimic> tag.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Declare launch arguments ──────────────────────────────────────────────
    declared_arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution(
                [FindPackageShare("mearm_bringup"), "rviz", "display.rviz"]
            ),
            description="Full path to the RViz config file",
        ),
    ]

    # ── URDF via xacro ────────────────────────────────────────────────────────
    urdf_file = PathJoinSubstitution(
        [FindPackageShare("mearm_description"), "urdf", "wrapper.urdf.xacro"]
    )

    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                urdf_file,
                " use_mock_hardware:=true",
            ]
        ),
        value_type=str,
    )

    # ── Nodes ─────────────────────────────────────────────────────────────────

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    # GUI sliders — zero initial position so the arm starts upright
    # joint4_wrist_pitch slider will appear but its value is overridden
    # by the <mimic> constraint propagation in robot_state_publisher.
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            # Initial positions: all zeros so arm starts in a known pose
            {"zeros.joint_0": 0.0},
            {"zeros.joint_1": 0.0},
            {"zeros.joint_2": 0.0},
            {"zeros.joint_3": 0.0},
            # mimic joints will be taken care of automatically
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    return LaunchDescription(
        declared_arguments
        + [
            robot_state_publisher_node,
            joint_state_publisher_gui_node,
            rviz_node,
        ]
    )
