#!/usr/bin/env python3
"""Center all servos (0 rad -> 90 deg)."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class ServoCenter(Node):
    def __init__(self):
        super().__init__("servo_center")
        self.pub = self.create_publisher(Float64MultiArray, "/arm_joint_commands", 10)
        self.timer = self.create_timer(0.1, self.tick)
        self.count = 0

    def tick(self):
        msg = Float64MultiArray()
        msg.data = [0.0, 0.0, 0.0, 0.0]
        self.pub.publish(msg)
        self.count += 1
        if self.count >= 5:
            self.get_logger().info("Centered all servos (0 rad).")
            raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = ServoCenter()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
