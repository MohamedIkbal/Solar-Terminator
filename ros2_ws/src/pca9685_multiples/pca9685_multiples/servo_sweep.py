#!/usr/bin/env python3
"""Sweep all 4 servos through a sine wave to verify the direct serial link
to the STM32 (via the servo_serial_bridge node) — no micro-ROS involved."""
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class ServoSweep(Node):
    def __init__(self):
        super().__init__("servo_sweep")
        self.declare_parameter("amplitude_rad", 0.8)
        self.declare_parameter("frequency_hz", 0.2)
        self.declare_parameter("publish_rate", 20.0)

        self.amp = float(self.get_parameter("amplitude_rad").value)
        self.freq = float(self.get_parameter("frequency_hz").value)
        self.rate = float(self.get_parameter("publish_rate").value)

        self.pub = self.create_publisher(Float64MultiArray, "/arm_joint_commands", 10)

        self.t0 = self.get_clock().now()
        period = 1.0 / max(1.0, self.rate)
        self.timer = self.create_timer(period, self.tick)
        self.get_logger().info("Sweeping servos via direct serial bridge (/arm_joint_commands)")

    def tick(self):
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        val = self.amp * math.sin(2 * math.pi * self.freq * t)
        msg = Float64MultiArray()
        msg.data = [val, val, val, val]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ServoSweep()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
