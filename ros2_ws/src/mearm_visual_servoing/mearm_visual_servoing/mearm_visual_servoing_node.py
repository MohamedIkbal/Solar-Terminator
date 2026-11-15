#!/usr/bin/env python3
"""Image-based visual servoing for the MeArm.

Subscribes to the vision_msgs/Detection2D stream published by
mearm_object_detection (/object_detection/detections) and converts the
tracked object's pixel position/size into joint targets, which are then
streamed out on the same interfaces the rest of the stack already expects:

  * /arm_joint_commands        (std_msgs/Float64MultiArray, 4 values)
                                 consumed by pca9685_multiple's
                                 servo_serial_bridge to drive the real servos.
  * /arm_controller/joint_trajectory   (trajectory_msgs/JointTrajectory)
  * /gripper_controller/joint_trajectory (trajectory_msgs/JointTrajectory)
                                 consumed by the ros2_control
                                 JointTrajectoryControllers so the simulated/
                                 RViz arm follows along.

Keeping this control loop in its own node (rather than inline inside the
detection node) means the detection side only has to report what it saw,
and the servoing behaviour (gains, deadband, smoothing, home pose, timeout)
can be tuned or swapped without touching the vision/classifier code.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from vision_msgs.msg import Detection2D

UNKNOWN_LABEL = "unknown"


def map_range(value, in_min, in_max, out_min, out_max):
    if in_max == in_min:
        t = 0.0
    else:
        t = (value - in_min) / (in_max - in_min)
    t = max(0.0, min(1.0, t))
    return out_min + t * (out_max - out_min)


class MeArmVisualServoingNode(Node):
    def __init__(self):
        super().__init__("mearm_visual_servoing_node")

        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)
        self.declare_parameter("min_object_area", 400.0)
        self.declare_parameter("max_object_area", 150000.0)
        self.declare_parameter("min_confidence", 0.5)
        self.declare_parameter("smoothing_alpha", 0.25)
        self.declare_parameter("deadband_px", 4.0)
        self.declare_parameter("object_lost_timeout_ms", 800)
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("max_joint_speed_rad_s", 1.0)
        self.declare_parameter("joint0_min", -1.2)
        self.declare_parameter("joint0_max", 1.2)
        self.declare_parameter("joint1_min", -1.57)
        self.declare_parameter("joint1_max", 1.0)
        self.declare_parameter("joint2_min", 0.0)
        self.declare_parameter("joint2_max", 1.57)
        self.declare_parameter("joint3_min", 0.0)
        self.declare_parameter("joint3_max", 0.42)
        self.declare_parameter("joint3_tracking", 0.0)

        self.image_width = self.get_parameter("image_width").value
        self.image_height = self.get_parameter("image_height").value
        self.min_object_area = self.get_parameter("min_object_area").value
        self.max_object_area = self.get_parameter("max_object_area").value
        self.min_confidence = self.get_parameter("min_confidence").value
        self.smoothing_alpha = self.get_parameter("smoothing_alpha").value
        self.deadband_px = self.get_parameter("deadband_px").value
        self.object_lost_timeout_ms = self.get_parameter("object_lost_timeout_ms").value
        control_rate_hz = self.get_parameter("control_rate_hz").value
        self.max_joint_speed_rad_s = self.get_parameter("max_joint_speed_rad_s").value
        self.control_rate_hz = control_rate_hz

        self.j0_min = self.get_parameter("joint0_min").value
        self.j0_max = self.get_parameter("joint0_max").value
        self.j1_min = self.get_parameter("joint1_min").value
        self.j1_max = self.get_parameter("joint1_max").value
        self.j2_min = self.get_parameter("joint2_min").value
        self.j2_max = self.get_parameter("joint2_max").value
        self.j3_min = self.get_parameter("joint3_min").value
        self.j3_max = self.get_parameter("joint3_max").value
        self.j3_tracking = self.get_parameter("joint3_tracking").value

        self.home_j0 = 0.0
        self.home_j1 = (self.j1_min + self.j1_max) / 2.0
        self.home_j2 = (self.j2_min + self.j2_max) / 2.0
        self.home_j3 = (self.j3_min + self.j3_max) / 2.0

        self.smoothed = [self.home_j0, self.home_j1, self.home_j2, self.home_j3]
        self.last_target_px = None  # (nx, ny, area) of the last accepted detection
        self.target_class = ""  # "" = servo toward any known class
        self.last_seen = self.get_clock().now()
        self.have_seen_once = False

        self.cmd_pub = self.create_publisher(Float64MultiArray, "/arm_joint_commands", 10)
        self.arm_traj_pub = self.create_publisher(
            JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self.gripper_traj_pub = self.create_publisher(
            JointTrajectory, "/gripper_controller/joint_trajectory", 10)

        self.create_subscription(
            Detection2D, "/object_detection/detections", self._detection_cb, 10)
        self.create_subscription(
            String, "/object_detection/target_class", self._target_class_cb, 10)

        period = 1.0 / control_rate_hz
        self.create_timer(period, self._control_tick)

        self.get_logger().info(
            "mearm_visual_servoing_node started: image %dx%d, control %.1f Hz, "
            "lost timeout %d ms" % (
                self.image_width, self.image_height, control_rate_hz,
                self.object_lost_timeout_ms))

    def _target_class_cb(self, msg: String):
        self.target_class = msg.data
        self.get_logger().info(f"Visual servoing target class set to: '{self.target_class}'")

    def _detection_cb(self, msg: Detection2D):
        label = msg.id
        score = msg.results[0].hypothesis.score if msg.results else 0.0

        if label == UNKNOWN_LABEL or label == "none" or not label:
            return
        if score < self.min_confidence:
            return
        if self.target_class and label != self.target_class:
            return

        nx = msg.bbox.center.position.x / float(self.image_width)
        ny = msg.bbox.center.position.y / float(self.image_height)
        area = msg.bbox.size_x * msg.bbox.size_y

        # Deadband: ignore sub-pixel jitter so the arm doesn't hunt in place
        # on a perfectly still target.
        if self.last_target_px is not None:
            dx_px = abs(nx * self.image_width - self.last_target_px[0] * self.image_width)
            dy_px = abs(ny * self.image_height - self.last_target_px[1] * self.image_height)
            if dx_px < self.deadband_px and dy_px < self.deadband_px:
                self.last_seen = self.get_clock().now()
                self.have_seen_once = True
                return

        self.last_target_px = (nx, ny, area)
        self.last_seen = self.get_clock().now()
        self.have_seen_once = True

    def _control_tick(self):
        elapsed_ms = (self.get_clock().now() - self.last_seen).nanoseconds / 1e6

        if self.have_seen_once and elapsed_ms <= self.object_lost_timeout_ms \
                and self.last_target_px is not None:
            nx, ny, area = self.last_target_px
            target_j0 = map_range(nx, 0.0, 1.0, self.j0_min, self.j0_max)
            # Image y grows downward; joint1 "up" is the max of its range.
            target_j1 = map_range(ny, 0.0, 1.0, self.j1_max, self.j1_min)
            target_j2 = map_range(
                area, self.min_object_area, self.max_object_area, self.j2_min, self.j2_max)
            target_j3 = self.j3_tracking
        else:
            target_j0, target_j1, target_j2, target_j3 = (
                self.home_j0, self.home_j1, self.home_j2, self.home_j3)

        targets = [target_j0, target_j1, target_j2, target_j3]
        # Cap how far a joint may move in a single tick regardless of how
        # large the raw EMA step would be. Without this, a big setpoint jump
        # (object re-acquired after being lost, or snapping between a
        # tracking target and the home pose) produces a fast initial swing
        # even with a slow smoothing_alpha, since the EMA's first step is
        # proportional to the whole error. This keeps commanded joint speed
        # bounded to a value real MeArm servos can actually track smoothly.
        max_step = self.max_joint_speed_rad_s / self.control_rate_hz
        for i, t in enumerate(targets):
            step = self.smoothing_alpha * (t - self.smoothed[i])
            step = max(-max_step, min(max_step, step))
            self.smoothed[i] += step

        self._publish()

    def _publish(self):
        cmd_msg = Float64MultiArray()
        cmd_msg.data = list(self.smoothed)
        self.cmd_pub.publish(cmd_msg)

        now = self.get_clock().now().to_msg()
        time_from_start = rclpy.duration.Duration(seconds=0.2).to_msg()

        arm_traj = JointTrajectory()
        arm_traj.header.stamp = now
        arm_traj.joint_names = ["joint_0", "joint_1", "joint_2"]
        arm_point = JointTrajectoryPoint()
        arm_point.positions = self.smoothed[:3]
        arm_point.time_from_start = time_from_start
        arm_traj.points.append(arm_point)
        self.arm_traj_pub.publish(arm_traj)

        gripper_traj = JointTrajectory()
        gripper_traj.header.stamp = now
        gripper_traj.joint_names = ["joint_3"]
        gripper_point = JointTrajectoryPoint()
        gripper_point.positions = [self.smoothed[3]]
        gripper_point.time_from_start = time_from_start
        gripper_traj.points.append(gripper_point)
        self.gripper_traj_pub.publish(gripper_traj)


def main(args=None):
    rclpy.init(args=args)
    node = MeArmVisualServoingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
