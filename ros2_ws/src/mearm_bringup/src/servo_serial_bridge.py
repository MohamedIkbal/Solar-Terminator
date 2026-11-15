#!/usr/bin/env python3
import time
import threading

import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class ServoSerialBridge(Node):
    def __init__(self):
        super().__init__("servo_serial_bridge")
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("max_write_hz", 10.0)
        self.declare_parameter("min_change_rad", 0.02)

        port = self.get_parameter("port").value
        baud = int(self.get_parameter("baud").value)
        self.max_write_hz = float(self.get_parameter("max_write_hz").value)
        self.min_change_rad = float(self.get_parameter("min_change_rad").value)
        self.min_interval = (1.0 / self.max_write_hz) if self.max_write_hz > 0 else 0.0

        self.ser = serial.Serial(port, baud, timeout=0)
        time.sleep(2)
        self.ser.reset_input_buffer()

        self.last_cmd = [0.0, 0.0, 0.0, 0.0]
        self.last_write_time = [0.0, 0.0, 0.0, 0.0]
        self.lock = threading.Lock()

        self.sub = self.create_subscription(
            Float64MultiArray, "/arm_joint_commands", self.cb, 10)
        self.get_logger().info(
            f"Serial bridge open on {port} @ {baud} "
            f"(max {self.max_write_hz} Hz/joint, min_change {self.min_change_rad} rad)")

    def cb(self, msg):
        now = time.monotonic()
        with self.lock:
            for i, v in enumerate(list(msg.data)[:4]):
                v = float(v)
                if (now - self.last_write_time[i] >= self.min_interval or
                        abs(v - self.last_cmd[i]) > self.min_change_rad):
                    try:
                        self.ser.write(f"J{i} {v:.3f}\n".encode())
                        self.ser.flush()
                    except serial.SerialException as e:
                        self.get_logger().error(f"Serial write failed: {e}")
                        return
                    self.last_cmd[i] = v
                    self.last_write_time[i] = now


def main(args=None):
    rclpy.init(args=args)
    node = ServoSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.ser.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
