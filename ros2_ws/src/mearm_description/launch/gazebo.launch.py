#!/usr/bin/env python3
"""Launch the MeArm in Gazebo (Harmonic/gz sim) with ros2_control position control."""
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    pkg_share = get_package_share_directory('mearm_description')
    xacro_path = os.path.join(pkg_share, 'urdf', 'mearm.urdf.xacro')
    world_path = os.path.join(pkg_share, 'worlds', 'mearm_world.sdf')

    # Spawning the robot from the /robot_description topic converts the URDF to
    # SDF internally, and that conversion rewrites package:// mesh URIs into
    # model://<package_name>/... URIs. Gazebo resolves model:// by searching
    # GZ_SIM_RESOURCE_PATH for a directory named after the package, so point it
    # at the installed share directory's parent (share/mearm_description lives
    # under share/, matching model://mearm_description/...).
    #
    # The MeArm's four-bar linkage relies on URDF <mimic> joints being enforced
    # as a real physics constraint. Per Gazebo's own release notes, mimic
    # constraints are only implemented by the Bullet-Featherstone physics
    # engine plugin (neither DART nor ODE support them), so the world file's
    # Physics system plugin explicitly loads gz-physics-bullet-featherstone-plugin.
    # That plugin ships in a separate vendor package install prefix that
    # gz-sim's default plugin search does not cover, so we point
    # GZ_SIM_PHYSICS_ENGINE_PATH at the vendor plugin directory explicitly
    # (all gz-physics-9 engine plugins, including bullet-featherstone, live
    # in the same directory).
    #
    # NOTE: these are passed via `additional_env` directly on the ExecuteProcess
    # below (NOT as separate AppendEnvironmentVariable launch actions) because
    # the launch-context environment propagation to ExecuteProcess was found to
    # silently drop GZ_SIM_PHYSICS_ENGINE_PATH here, leaving Gazebo on its
    # default physics engine with the mimic constraints unenforced.
    ros_prefix = os.environ.get('ROS_DISTRO')
    ros_prefix = f'/opt/ros/{ros_prefix}' if ros_prefix else '/opt/ros/lyrical'
    gz_resource_path = os.path.dirname(pkg_share)
    gz_physics_engine_path = os.path.join(
        ros_prefix, 'opt', 'gz_physics_vendor', 'lib', 'gz-physics-9', 'engine-plugins'
    )
    existing_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    merged_resource_path = (
        f'{gz_resource_path}{os.pathsep}{existing_resource_path}'
        if existing_resource_path else gz_resource_path
    )

    # Process xacro -> URDF XML string
    robot_description = xacro.process_file(xacro_path).toxml()

    # 1) Start Gazebo (gz sim) with our world
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],
        output='screen',
        additional_env={
            'GZ_SIM_RESOURCE_PATH': merged_resource_path,
            'GZ_SIM_PHYSICS_ENGINE_PATH': gz_physics_engine_path,
        },
    )

    # 2) robot_state_publisher publishes /robot_description and TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    # 3) Spawn the robot into Gazebo once it has started
    spawn_robot = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=['-topic', '/robot_description', '-name', 'mearm', '-z', '0.0'],
                output='screen',
            )
        ],
    )

    # 4) Bridge /clock from Gazebo to ROS so use_sim_time works
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # 5) Spawn controllers once the controller_manager (hosted by gz_ros2_control) is up.
    # The Bullet-Featherstone physics plugin (required for the mimic-joint linkage)
    # can take longer than a fixed few seconds to load on this machine, so instead of
    # racing a hardcoded TimerAction clock (which produced "Aborting, no controller is
    # switched! (::STRICT switch)" when the controller_manager wasn't ready yet), each
    # spawner is given a generous --controller-manager-timeout to actually WAIT for it,
    # and later spawners are chained to start only after the previous one exits
    # successfully (configured + activated), not on a guessed delay.
    controllers_yaml = os.path.join(pkg_share, 'config', 'mearm_controllers.yaml')

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager-timeout', '60',
        ],
        output='screen',
    )
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller', '--param-file', controllers_yaml,
            '--controller-manager-timeout', '60',
        ],
        output='screen',
    )
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'gripper_controller', '--param-file', controllers_yaml,
            '--controller-manager-timeout', '60',
        ],
        output='screen',
    )

    # Spawn joint_state_broadcaster shortly after the robot exists; it will itself
    # wait (up to 60s) for the controller_manager rather than failing immediately.
    spawn_joint_state_broadcaster = TimerAction(
        period=6.0,
        actions=[joint_state_broadcaster_spawner],
    )

    delayed_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )
    delayed_gripper_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        clock_bridge,
        spawn_robot,
        spawn_joint_state_broadcaster,
        delayed_arm_controller,
        delayed_gripper_controller,
    ])

