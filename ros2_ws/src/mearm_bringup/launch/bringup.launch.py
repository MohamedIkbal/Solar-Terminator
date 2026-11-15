#!/usr/bin/env python3
"""
bringup.launch.py
=================
Full ros2_control bringup for the 4-DOF MeArm acrylic robotic arm.

This launch file starts the complete control stack:
  1. robot_state_publisher     — Publishes the URDF-derived TF tree.
  2. ros2_control_node         — The controller_manager with controllers.yaml.
  3. joint_state_broadcaster   — Publishes /joint_states from hardware state
                                 interfaces (spawned with 3 s delay for CM
                                 readiness).
  4. arm_controller            — JointTrajectoryController that accepts goals
                                 on /arm_controller/joint_trajectory (spawned
                                 2 s after broadcaster confirms active).
  5. servo_serial_bridge       — Direct ROS 2 node (mearm_bringup package)
                                 that writes "J<n> <rad>\n" lines straight to
                                 the STM32 over USB-serial (ONLY when
                                 use_mock_hardware:=false). No micro-ROS agent
                                 or XRCE transport involved — the STM32
                                 firmware is a plain HAL UART command parser
                                 and the whole ROS 2 graph lives on the host.
  6. rviz2                     — 3D visualisation (optional).

Usage — mock hardware (default, no STM32 needed):
  ros2 launch mearm_bringup bringup.launch.py

Usage — real hardware (STM32 flashed with the direct-serial firmware and
plugged in over USB, typically /dev/ttyACM0):
  ros2 launch mearm_bringup bringup.launch.py use_mock_hardware:=false

Usage — headless (no RViz):
  ros2 launch mearm_bringup bringup.launch.py start_rviz:=false
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # ── Package share directories (resolved at launch time) ───────────────────
    bringup_share = get_package_share_directory("mearm_bringup")
    description_share = get_package_share_directory("mearm_description")

    # ── Resolved file paths ──────────────────────────────────────────────────
    # controllers.yaml — ALL controller_manager params including controller
    # types AND arm_controller runtime config (joints, interfaces, constraints).
    controllers_yaml = os.path.join(bringup_share, "config", "controllers.yaml")

    # wrapper.urdf.xacro — top-level URDF including robot.urdf + ros2_control.
    urdf_file = os.path.join(description_share, "urdf", "wrapper.urdf.xacro")

    # RViz saved configuration for arm visualisation.
    rviz_config = os.path.join(bringup_share, "rviz", "display.rviz")

    # ── Launch arguments ──────────────────────────────────────────────────────
    # These can be overridden on the command line, e.g.:
    #   ros2 launch mearm_bringup bringup.launch.py use_mock_hardware:=false
    declared_arguments = [
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="true",
            description="Use mock hardware (true) or real STM32 (false)",
        ),
        DeclareLaunchArgument(
            "start_rviz",
            default_value="true",
            description="Launch RViz2 alongside the control stack",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock (for Gazebo integration)",
        ),
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyACM0",
            description="Serial device for the direct STM32 bridge (real hardware only)",
        ),
        DeclareLaunchArgument(
            "baud_rate",
            default_value="115200",
            description="Baud rate for the direct STM32 serial bridge",
        ),
    ]

    # ── Robot description (xacro → URDF string) ──────────────────────────────
    # Process the xacro file at launch time, passing the use_mock_hardware arg
    # so the correct hardware plugin is selected in the URDF.
    robot_description_content = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                urdf_file,
                " use_mock_hardware:=",
                LaunchConfiguration("use_mock_hardware"),
            ]
        ),
        value_type=str,
    )
    robot_description = {"robot_description": robot_description_content}

    # ── Nodes ─────────────────────────────────────────────────────────────────

    # robot_state_publisher: reads the URDF, listens for /joint_states, and
    # broadcasts the full TF tree for RViz and MoveIt.
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    # ros2_control_node (controller_manager): the core of the control stack.
    # Loads the hardware interface from the URDF and spawns controllers.
    #
    # Parameters loaded from:
    #   1. robot_description — the processed URDF string
    #   2. controllers.yaml  — controller types + arm_controller runtime params
    #
    # The CM forwards params from its namespace to child controllers:
    #   controller_manager.ros__parameters.arm_controller.joints
    #   → becomes /arm_controller's "joints" parameter
    #
    # DO NOT set name="controller_manager" here!  The executable already names
    # itself "controller_manager" internally.  Adding name= causes a global
    # -r __node:=controller_manager remapping that leaks into ALL sub-nodes
    # (including controllers), breaking their parameter namespace resolution.
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            robot_description,
            controllers_yaml,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    # ── Controller Spawners ───────────────────────────────────────────────────
    # The spawner executable tells the controller_manager to load, configure,
    # and activate a named controller.

    # joint_state_broadcaster: reads hardware state interfaces and publishes
    # /joint_states.  Must be active before arm_controller.
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    # arm_controller (JointTrajectoryController): accepts trajectory goals and
    # interpolates smooth position commands to the hardware.
    #
    # --param-file is REQUIRED on ros2_control v2.53+ (Humble).  The spawner
    # searches for a top-level "arm_controller:" key in this YAML file, then
    # calls set_controller_parameters() to set "params_file" on the CM node.
    # The CM reads that file path and loads the params into the controller
    # node before calling configure().
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager", "/controller_manager",
            "--param-file", controllers_yaml,
        ],
        output="screen",
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller","--controller-manager","/controller_manager","--param-file", controllers_yaml],
        output="screen",
    )

    # servo_serial_bridge: direct ROS 2 <-> STM32 link, real hardware only.
    # Subscribes /arm_joint_commands (the same topic mearm_hardware writes)
    # and forwards each value as a plain "J<n> <rad>\n" line over USB-serial.
    # No micro-ROS agent, no XRCE transport — just a serial port.
    servo_serial_bridge_node = Node(
        package="mearm_bringup",
        executable="servo_serial_bridge",
        name="servo_serial_bridge",
        output="screen",
        parameters=[
            {"port": LaunchConfiguration("serial_port")},
            {"baud": LaunchConfiguration("baud_rate")},
            {"max_write_hz": 10.0},
            {"min_change_rad": 0.02},
        ],
        condition=UnlessCondition(LaunchConfiguration("use_mock_hardware")),
    )

    # ── Sequencing ────────────────────────────────────────────────────────────
    # 1. Wait 3 s for the controller_manager to fully initialise and load the
    #    hardware interface before spawning any controllers.
    # 2. After joint_state_broadcaster exits (successfully configured), wait
    #    2 s then spawn arm_controller.
    # This ordering ensures state interfaces are available before the JTC
    # tries to claim them.

    delayed_broadcaster = TimerAction(
        period=3.0,
        actions=[joint_state_broadcaster_spawner],
    )

    delayed_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                TimerAction(
                    period=2.0,
                    actions=[arm_controller_spawner],
                )
            ],
        )
    )

    delayed_gripper_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[
                TimerAction(
                    period=1.0,
                    actions=[gripper_controller_spawner],
                )
            ],
        )
    )
    # ── RViz2 (optional) ─────────────────────────────────────────────────────
    # 3D visualisation with a saved display configuration that shows the arm
    # model, TF tree, and joint state topics.
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("start_rviz")),
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    # ── Assemble the launch description ───────────────────────────────────────
    return LaunchDescription(
        declared_arguments
        + [
            robot_state_publisher_node,
            controller_manager_node,
            delayed_broadcaster,
            delayed_arm_controller,
            delayed_gripper_controller,
            servo_serial_bridge_node,
            rviz_node,
        ]
    )
