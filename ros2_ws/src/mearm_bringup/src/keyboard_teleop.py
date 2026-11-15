#!/usr/bin/env python3
"""
keyboard_teleop.py
==================
Keyboard teleoperation node for the 4-DOF MeArm acrylic robotic arm.

Publishes JointTrajectory messages to /arm_controller/joint_trajectory,
which is consumed by the joint_trajectory_controller managed by ros2_control.

Key Bindings:
  A / D    →  joint_0 (base yaw)      +/- (rotate left/right)
  W / S    →  joint_1 (shoulder)       +/- (forward/back)
  Q / E    →  joint_2 (elbow)          +/- (up/down)
  Z / X    →  joint_3 (gripper)        +/- (open/close)
  Space    →  Move arm to HOME position (all joints = 0)
  P        →  Print current joint positions
  Ctrl+C   →  Quit

Notes:
  - All 4 joints are independently controlled.
  - Each keypress increments the target by STEP_RAD (default 0.05 rad ≈ 2.9°).
  - The current target is clamped to the URDF joint limits before publishing.
  - Trajectory execution time is EXEC_TIME seconds (smooth motion).
  - Requires arm_controller to be active (run bringup.launch.py first).

Usage:
  ros2 run mearm_bringup keyboard_teleop
  OR
  ros2 launch mearm_bringup bringup.launch.py    # in terminal 1
  ros2 run mearm_bringup keyboard_teleop          # in terminal 2
"""

import sys
import tty
import termios
import threading
import math

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


# ── Constants ──────────────────────────────────────────────────────────────

# Step size per keypress in radians (~2.9° per press).
# Increase for faster movement, decrease for finer control.
STEP_RAD = 0.05

# Duration (seconds) for each trajectory segment.
# The controller interpolates smoothly over this period.
EXEC_TIME = 0.4

# Topic published by the joint_trajectory_controller.
# Must match the controller name in controllers.yaml.
CONTROLLER_TOPIC = "/arm_controller/joint_trajectory"
GRIPPER_TOPIC = "/gripper_controller/joint_trajectory"

# Joint names — must EXACTLY match the URDF ros2_control block
# (wrapper.urdf.xacro) and controllers.yaml.
JOINT_NAMES = [
    "joint_0",   # base yaw (turret)
    "joint_1",   # shoulder (right servo)
    "joint_2",   # elbow (left servo)
    "joint_3",   # gripper servo
]

# Joint limits (radians) — must match the URDF <limit> tags in robot.urdf.
# If these don't match the URDF, the controller may reject trajectories
# that are within keyboard range but outside the URDF-declared limits.
JOINT_LIMITS = {
    "joint_0": (-1.2, 1.2),   # ±45° base rotation
    "joint_1": (-1.5707, 1.0),      # shoulder: -90° to ~57°
    "joint_2": (0.0, 1.5708),       # elbow: 0° to 90°
    "joint_3": (0.0, 1.5708),       # gripper: 0° to 90° (matches URDF)
}

# Key → (joint_index, direction_sign)
# Lower-case letters so we can handle both upper and lower.
KEY_MAP = {
    'a': (0, +1),   # joint_0: base yaw left
    'd': (0, -1),   # joint_0: base yaw right
    'w': (1, +1),   # joint_1: shoulder up/forward
    's': (1, -1),   # joint_1: shoulder down/back
    'q': (2, +1),   # joint_2: elbow up
    'e': (2, -1),   # joint_2: elbow down
    'z': (3, +1),   # joint_3: gripper open
    'x': (3, -1),   # joint_3: gripper close
}


# ── Non-blocking key reader ─────────────────────────────────────────────────

def get_key(settings):
    """Read one character from stdin without blocking on Enter.

    Temporarily sets the terminal to raw mode for single-character reads,
    then restores the original settings.

    Args:
        settings: Original terminal attributes (from termios.tcgetattr).

    Returns:
        A single character string.
    """
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


# ── ROS 2 Node ─────────────────────────────────────────────────────────────

class KeyboardTeleop(Node):
    """ROS 2 node that reads keyboard input and publishes JointTrajectory
    messages for teleoperation of the 4-DOF arm.

    Each keypress adjusts the corresponding joint target by STEP_RAD and
    publishes a new trajectory point to the arm_controller.
    """

    def __init__(self):
        super().__init__("arm_keyboard_teleop")

        # Publisher for JointTrajectory messages to the arm_controller
        self.pub = self.create_publisher(JointTrajectory, CONTROLLER_TOPIC, 10)
        self.gripper_pub = self.create_publisher(JointTrajectory, GRIPPER_TOPIC, 10)

        # Current target joint positions — start at home (all zeros)
        self.positions = [0.0, 0.0, 0.0, 0.0]  # [joint_0, joint_1, joint_2, joint_3]

        # Print the control banner
        self.get_logger().info(
            "\n"
            "─────────────────────────────────────\n"
            "  4-DOF Arm Keyboard Teleoperation\n"
            "─────────────────────────────────────\n"
            "  A / D  →  Base yaw (joint_0) +/-\n"
            "  W / S  →  Shoulder (joint_1)  +/-\n"
            "  Q / E  →  Elbow   (joint_2)  +/-\n"
            "  Z / X  →  Gripper (joint_3)  +/-\n"
            "  Space  →  HOME (all zeros)\n"
            "  P      →  Print positions\n"
            "  Ctrl+C →  Quit\n"
            "─────────────────────────────────────\n"
            f"  Publishing to: {CONTROLLER_TOPIC}\n"
            f"  Step: {math.degrees(STEP_RAD):.1f}° per key\n"
            "─────────────────────────────────────\n"
        )

        self.print_positions()

    def clamp(self, joint_name: str, value: float) -> float:
        """Clamp a joint position to its URDF-defined limits.

        Args:
            joint_name: Name of the joint (must exist in JOINT_LIMITS).
            value:      Desired position in radians.

        Returns:
            The clamped position value.
        """
        lo, hi = JOINT_LIMITS[joint_name]
        clamped = max(lo, min(hi, value))
        if clamped != value:
            self.get_logger().warn(
                f"{joint_name}: clamped {value:.3f} → {clamped:.3f} rad"
            )
        return clamped

    def print_positions(self):
        """Log the current target position of every joint."""
        lines = ["Current targets (rad):"]
        for i, name in enumerate(JOINT_NAMES):
            deg = math.degrees(self.positions[i])
            lines.append(f"  {name:28s}: {self.positions[i]:+.3f} ({deg:+.1f}°)")
        self.get_logger().info("\n".join(lines))

    def publish(self):
        """Build and publish a JointTrajectory message with the current
        target positions.

        The trajectory contains a single waypoint at time_from_start =
        EXEC_TIME, and the controller interpolates from the current state
        to that waypoint.
        """
        # Arm trajectory (joints 0-2)
        arm_msg = JointTrajectory()
        arm_msg.joint_names = JOINT_NAMES[:3]
        arm_pt = JointTrajectoryPoint()
        arm_pt.positions = list(self.positions[:3])
        arm_pt.velocities = [0.0] * 3
        arm_pt.accelerations = [0.0] * 3
        arm_pt.time_from_start = Duration(sec=sec_int, nanosec=nsec_int)
        arm_msg.points = [arm_pt]
        self.pub.publish(arm_msg)

        # Gripper trajectory (joint 3)
        grip_msg = JointTrajectory()
        grip_msg.joint_names = [JOINT_NAMES[3]]
        grip_pt = JointTrajectoryPoint()
        grip_pt.positions = [self.positions[3]]
        grip_pt.velocities = [0.0]
        grip_pt.accelerations = [0.0]
        grip_pt.time_from_start = Duration(sec=sec_int, nanosec=nsec_int)
        grip_msg.points = [grip_pt]
        self.gripper_pub.publish(grip_msg)

    def handle_key(self, key: str) -> bool:
        """Process a single keypress.

        Args:
            key: The character read from stdin.

        Returns:
            True to keep running, False to exit the teleop loop.
        """
        lower = key.lower()

        if key == '\x03':  # Ctrl+C
            return False

        elif lower in KEY_MAP:
            joint_idx, direction = KEY_MAP[lower]
            joint_name = JOINT_NAMES[joint_idx]
            new_pos = self.positions[joint_idx] + direction * STEP_RAD
            self.positions[joint_idx] = self.clamp(joint_name, new_pos)
            self.get_logger().info(
                f"{joint_name}: {self.positions[joint_idx]:+.3f} rad "
                f"({math.degrees(self.positions[joint_idx]):+.1f}°)"
            )
            self.publish()

        elif key == ' ':
            self.positions = [0.0, 0.0, 0.0, 0.0]
            self.get_logger().info("→ HOME position")
            self.publish()

        elif lower == 'p':
            self.print_positions()

        return True


def main(args=None):
    """Entry point for the keyboard teleop node.

    Sets the terminal to raw mode for immediate key reads, spins the
    ROS 2 node in a background thread, and runs the key-handling loop
    in the foreground.
    """
    rclpy.init(args=args)
    node = KeyboardTeleop()

    # Save terminal settings so we can restore them on exit
    settings = termios.tcgetattr(sys.stdin)

    # Spin ROS 2 callbacks (timers, subscriptions) in a background thread
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    try:
        running = True
        while running and rclpy.ok():
            key = get_key(settings)
            running = node.handle_key(key)
    except Exception as e:
        node.get_logger().error(f"Key read error: {e}")
    finally:
        # Restore terminal to cooked mode before exiting
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.get_logger().info("Keyboard teleop stopped.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
