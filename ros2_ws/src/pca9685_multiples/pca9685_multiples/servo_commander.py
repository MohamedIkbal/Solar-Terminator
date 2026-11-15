#!/usr/bin/env python3
"""Servo commander: publish /arm_joint_commands (Float64MultiArray, 4 rad) to
drive the PCA9685 servos on the STM32 (pca9685-multiple firmware).

No micro-ROS, no XRCE agent: the `servo_serial_bridge` node subscribes to
/arm_joint_commands and writes "J<n> <rad>\n" lines straight to the STM32's
serial port. The firmware is command-only and does not transmit state back,
so there is no joint-state echo to listen for.

Usage:
    ros2 run pca9685_multiple servo_commander --ros-args -p joint_values:="[0.0,0.0,0.0,0.0]"
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class ServoCommander(Node):
    def __init__(self):
        super().__init__("servo_commander")

        self.declare_parameter("joint_values", [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("publish_rate", 10.0)

        self.joint_values = list(self.get_parameter("joint_values").value)
        if len(self.joint_values) != 4:
            self.get_logger().warn("joint_values must have 4 entries; padding/truncating")
            while len(self.joint_values) < 4:
                self.joint_values.append(0.0)
            self.joint_values = self.joint_values[:4]

        self.pub = self.create_publisher(Float64MultiArray, "/arm_joint_commands", 10)

        period = 1.0 / max(0.1, float(self.get_parameter("publish_rate").value))
        self.timer = self.create_timer(period, self.publish_cmd)
        self.get_logger().info(f"Publishing /arm_joint_commands = {self.joint_values}")

    def publish_cmd(self):
        msg = Float64MultiArray()
        msg.data = [float(v) for v in self.joint_values]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ServoCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
