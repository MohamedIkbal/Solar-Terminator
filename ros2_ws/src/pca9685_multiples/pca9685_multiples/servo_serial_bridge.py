#!/usr/bin/env python3
import time
import threading

import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


def _smootherstep(t: float) -> float:
    """Cubic S-curve: 6t^5 − 15t^4 + 10t^3. Zero velocity at t=0 and t=1."""
    if t <= 0.0:
        return 0.0
    if t >= 1.0:
        return 1.0
    t2 = t * t
    t3 = t2 * t
    t4 = t3 * t
    t5 = t4 * t
    return 6.0 * t5 - 15.0 * t4 + 10.0 * t3
class ServoSerialBridge(Node):
    def __init__(self):
        super().__init__("servo_serial_bridge")
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud", 115200)
        # Max writes per joint per second — prevents flooding the STM32 and
        # keeps real servos from jerking when commands arrive faster than they
        # can physically move.
        self.declare_parameter("max_write_hz", 10.0)
        # Only re-send a joint command if it changed by more than this (rad).
        self.declare_parameter("min_change_rad", 0.02)
        # ── S-curve smoothing ─────────────────────────────────────────────
        self.declare_parameter("enable_scurve", False)
        self.declare_parameter("scurve_duration_s", 0.3)
        self.declare_parameter("scurve_update_hz", 50.0)
        # ──────────────────────────────────────────────────────────────────

        port = self.get_parameter("port").value
        baud = int(self.get_parameter("baud").value)
        self.max_write_hz = float(self.get_parameter("max_write_hz").value)
        self.min_change_rad = float(self.get_parameter("min_change_rad").value)
        self.min_interval = (1.0 / self.max_write_hz) if self.max_write_hz > 0 else 0.0

        # S-curve parameters
        self.enable_scurve = bool(self.get_parameter("enable_scurve").value)
        self.scurve_duration_s = float(self.get_parameter("scurve_duration_s").value)
        self.scurve_update_hz = float(self.get_parameter("scurve_update_hz").value)
        self.scurve_dt = 1.0 / self.scurve_update_hz if self.scurve_update_hz > 0 else 0.02

        # Non-blocking serial so a slow STM32 can't stall the ROS callback thread.
        self.ser = serial.Serial(port, baud, timeout=0)
        time.sleep(2)
        self.ser.reset_input_buffer()

        self.last_cmd = [0.0, 0.0, 0.0, 0.0]
        self.last_write_time = [0.0, 0.0, 0.0, 0.0]
        self.lock = threading.Lock()

        # Per-joint S-curve state
        self.scurve_pos = [0.0, 0.0, 0.0, 0.0]
        self.scurve_target = [0.0, 0.0, 0.0, 0.0]
        self.scurve_start = [0.0, 0.0, 0.0, 0.0]
        self.scurve_t = [1.0, 1.0, 1.0, 1.0]
        self.scurve_active = [False, False, False, False]

        self.sub = self.create_subscription(
            Float64MultiArray, "/arm_joint_commands", self.cb, 10)

        # Timer that advances the S-curve and writes to serial at a fixed rate.
        if self.enable_scurve:
            self.scurve_timer = self.create_timer(
                self.scurve_dt, self.scurve_tick)
        self.get_logger().info(
            f"Serial bridge open on {port} @ {baud} "
            f"(max {self.max_write_hz} Hz/joint, min_change {self.min_change_rad} rad, "
            f"scurve={'on' if self.enable_scurve else 'off'})")

    def cb(self, msg):
        now = time.monotonic()
        with self.lock:
            for i, v in enumerate(list(msg.data)[:4]):
                v = float(v)
                if self.enable_scurve:
                    # Set a new S-curve target; the timer interpolates and writes.
                    if abs(v - self.scurve_target[i]) > self.min_change_rad:
                        self.scurve_start[i] = self.scurve_pos[i]
                        self.scurve_target[i] = v
                        self.scurve_t[i] = 0.0
                        self.scurve_active[i] = True
                else:
                    # Original behaviour: write immediately (rate-limited).
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

    def scurve_tick(self):
        """Advance each active S-curve by one tick and write to serial."""
        now = time.monotonic()
        with self.lock:
            for i in range(4):
                if not self.scurve_active[i]:
                    continue

                self.scurve_t[i] += self.scurve_dt / self.scurve_duration_s
                if self.scurve_t[i] >= 1.0:
                    self.scurve_t[i] = 1.0
                    self.scurve_active[i] = False
                    self.scurve_pos[i] = self.scurve_target[i]
                else:
                    s = _smootherstep(self.scurve_t[i])
                    self.scurve_pos[i] = (
                        self.scurve_start[i]
                        + (self.scurve_target[i] - self.scurve_start[i]) * s
                    )

                if abs(self.scurve_pos[i] - self.last_cmd[i]) > self.min_change_rad:
                    try:
                        self.ser.write(f"J{i} {self.scurve_pos[i]:.3f}\n".encode())
                        self.ser.flush()
                    except serial.SerialException as e:
                        self.get_logger().error(f"Serial write failed: {e}")
                        self.scurve_active[i] = False
                        return
                    self.last_cmd[i] = self.scurve_pos[i]
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
