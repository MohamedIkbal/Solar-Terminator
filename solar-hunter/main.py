#!/usr/bin/env python3
"""
main.py
Solar-hunter pipeline: object detection -> blob (LED/sunlight) detector ->
reflector aiming -> (optional) MeArm servo command publish.

Three stages, in order:
  1. Object detection (mearm_object_detection) classifies what's in frame
     into one of: space_debris, meteor(s), locust(s), solar_sail,
     moon_south_darkside, and publishes it on
     /object_detection/detections (vision_msgs/Detection2D). This script
     picks that up to choose WHERE to aim (azimuth/elevation).
  2. detect_light_blob() finds the white LED / yellow sunlight in the same
     camera feed -- this is WHAT we're actually centering the reflector on.
  3. reflector_angle computes the pan/tilt servo angles that point the
     MeArm's reflector (mounted on the gripper) so it reflects that light
     toward the target chosen in stage 1.

Usage:
  python3 main.py --target meteors                 # manual override, one-shot
                                                     # aim, no camera
  python3 main.py --target meteors --camera 0       # manual override + live
                                                     # focus-centering loop
  python3 main.py --camera 5                        # no --target: wait for
                                                     # the mearm_object_detection
                                                     # classifier to publish a
                                                     # class label, then keep
                                                     # auto-retargeting as the
                                                     # classification changes
  python3 main.py --list-targets

If ROS 2 (rclpy) is available, aim angles are published as radians on
/arm_joint_commands (std_msgs/Float64MultiArray) by patching the pan/
tilt entries of the arm's existing joint vector (so other joints are
left untouched). Without rclpy installed this still runs standalone and
just prints the computed angles -- nothing here requires ROS 2.
"""

import argparse
import sys
import time

import numpy as np

import targets
import reflector_angle
from detector import detect_light_blob, draw_debug_overlay

try:
    import cv2
except ImportError:
    cv2 = None

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float64MultiArray
    from sensor_msgs.msg import JointState
    from vision_msgs.msg import Detection2D
    HAVE_ROS2 = True
except ImportError:
    HAVE_ROS2 = False

# Index of the pan and tilt joints inside the MeArm's joint vector.
# Adjust these to match the real arm's joint order (check `ros2 topic
# echo /joint_states` -> name[] array index).
PAN_JOINT_INDEX = 0
TILT_JOINT_INDEX = 1
DEFAULT_NUM_JOINTS = 4

# Minimum classifier confidence (stage 1) before we trust its class label
# enough to retarget the reflector.
DETECTION_CONFIDENCE_THRESHOLD = 0.5


def deg2rad(d):
    return float(np.radians(d))


class JointBridge:
    """Thin optional ROS 2 wrapper: tracks last known joint vector from
    /joint_states, the latest object-detection class label from
    /object_detection/detections, and republishes /arm_joint_commands with
    pan/tilt overwritten."""

    def __init__(self):
        self.last_positions = [0.0] * DEFAULT_NUM_JOINTS
        self.detected_target_name = None
        if not HAVE_ROS2:
            self.node = None
            return
        rclpy.init(args=None)
        self.node = Node("solar_hunter")
        self.pub = self.node.create_publisher(Float64MultiArray, "/arm_joint_commands", 10)
        self.node.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)
        self.node.create_subscription(
            Detection2D, "/object_detection/detections", self._on_detection, 10)

    def _on_joint_states(self, msg):
        if msg.position:
            self.last_positions = list(msg.position)

    def _on_detection(self, msg):
        """Stage 1: pick up the object-detection classifier's class label
        (space_debris / meteor / locust / solar_sail / moon_south_darkside)
        and resolve it to a known aim target."""
        label = msg.id
        score = msg.results[0].hypothesis.score if msg.results else 0.0

        if not label or label.lower() in ("unknown", "none"):
            return
        if score < DETECTION_CONFIDENCE_THRESHOLD:
            return

        try:
            t = targets.get_target(label)
        except KeyError:
            return  # classifier emitted a label we don't have a target for

        if t.name != self.detected_target_name:
            print(f"[object_detection] classified target: '{t.name}' (score={score:.2f})")
        self.detected_target_name = t.name

    def get_detected_target_name(self):
        if HAVE_ROS2 and self.node is not None:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return self.detected_target_name

    def publish_pan_tilt(self, pan_deg: float, tilt_deg: float):
        if not HAVE_ROS2 or self.node is None:
            print(f"[dry-run, no ROS2] would publish pan={pan_deg:.2f}deg "
                  f"tilt={tilt_deg:.2f}deg on /arm_joint_commands")
            return
        rclpy.spin_once(self.node, timeout_sec=0.1)
        positions = list(self.last_positions)
        needed = max(PAN_JOINT_INDEX, TILT_JOINT_INDEX) + 1
        if len(positions) < needed:
            positions += [0.0] * (needed - len(positions))
        positions[PAN_JOINT_INDEX] = deg2rad(pan_deg)
        positions[TILT_JOINT_INDEX] = deg2rad(tilt_deg)

        msg = Float64MultiArray()
        msg.data = positions
        self.pub.publish(msg)
        print(f"published /arm_joint_commands: {['%.3f' % p for p in positions]}")

    def shutdown(self):
        if HAVE_ROS2 and self.node is not None:
            self.node.destroy_node()
            rclpy.shutdown()


def aim_at_target(target_name: str, bridge: JointBridge):
    t = targets.get_target(target_name)
    aim = reflector_angle.mirror_axis_to_servo_angles(t.direction_vector())
    print(f"Target '{t.name}' (az={t.azimuth_deg:.1f} el={t.elevation_deg:.1f}) "
          f"-> pan={aim.pan_deg:.2f} deg, tilt={aim.tilt_deg:.2f} deg"
          + (" [CLIPPED to servo range]" if aim.clipped else ""))
    bridge.publish_pan_tilt(aim.pan_deg, aim.tilt_deg)
    return aim


def wait_for_detected_target(bridge: JointBridge, timeout_s: float = 30.0):
    """Stage 1 gate: block until the object-detection classifier reports a
    recognized target label, or give up after timeout_s seconds. Returns
    the target name, or None."""
    if not HAVE_ROS2:
        print("ROS2 (rclpy) is not available; cannot wait for object "
              "detection without --target.")
        return None
    print("Waiting for object detection to classify a target "
          "(space_debris / meteor / locust / solar_sail / moon_south_darkside)...")
    start = time.time()
    while time.time() - start < timeout_s:
        name = bridge.get_detected_target_name()
        if name is not None:
            return name
        time.sleep(0.05)
    return None


def run_camera_centering_loop(camera_index: int, bridge: JointBridge, base_aim,
                               auto_target: bool = False):
    """Stages 2+3, live: keep the detected white-LED/yellow-sun blob
    centered on the reflector's focal point by nudging pan/tilt around the
    base aim angle computed for the chosen target. When auto_target is
    True, also keep polling the object-detection classifier (stage 1) and
    re-aim at a new base direction whenever the classified target changes."""
    if cv2 is None:
        print("opencv-python is not installed; cannot run the camera loop.")
        return
    cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"Could not open camera index {camera_index}.")
        return

    last_detected_name = bridge.get_detected_target_name() if auto_target else None

    print(f"Focus distance assumed: {reflector_angle.FOCUS_DISTANCE_M} m. "
          "Press 'q' to quit.")
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break

            if auto_target:
                detected_name = bridge.get_detected_target_name()
                if detected_name is not None and detected_name != last_detected_name:
                    print(f"[object_detection] target changed to '{detected_name}', "
                          "retargeting reflector...")
                    base_aim = aim_at_target(detected_name, bridge)
                    last_detected_name = detected_name

            blob = detect_light_blob(frame)
            pan_corr, tilt_corr = reflector_angle.focus_alignment_correction(blob)

            pan_deg = base_aim.pan_deg + pan_corr
            tilt_deg = base_aim.tilt_deg + tilt_corr
            bridge.publish_pan_tilt(pan_deg, tilt_deg)

            debug = draw_debug_overlay(frame, blob)
            cv2.putText(
                debug,
                f"pan={pan_deg:.1f} tilt={tilt_deg:.1f}",
                (10, debug.shape[0] - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1, cv2.LINE_AA,
            )
            cv2.imshow("solar-hunter", debug)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
            time.sleep(0.02)
    finally:
        cap.release()
        if cv2 is not None:
            cv2.destroyAllWindows()


def run_detection_only_loop(bridge: JointBridge, base_aim):
    """Camera-less auto-retarget loop: keep polling the object-detection
    classifier (stage 1) and re-aim at a new base direction whenever the
    classified target changes. LED/light centering (stages 2+3) is
    skipped here since no --camera was given."""
    print("Auto-retargeting from object-detection classifications; "
          "LED/light centering is skipped since no --camera was given. "
          "Press Ctrl+C to quit.")
    last_detected_name = bridge.get_detected_target_name()
    try:
        while True:
            detected_name = bridge.get_detected_target_name()
            if detected_name is not None and detected_name != last_detected_name:
                print(f"[object_detection] target changed to '{detected_name}', "
                      "retargeting reflector...")
                base_aim = aim_at_target(detected_name, bridge)
                last_detected_name = detected_name
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass


def main():
    parser = argparse.ArgumentParser(description="solar-hunter reflector aiming pipeline")
    parser.add_argument(
        "--target", type=str, default=None,
        help=f"manual override; one of: {', '.join(targets.list_targets())}. "
             "If omitted, waits for the object-detection classifier "
             "(stage 1) to pick a target instead.")
    parser.add_argument("--camera", type=int, default=None,
                         help="camera index to run the live focus-centering loop "
                              "(on this machine index 0 is the Surface front camera, "
                              "which stays dark under Linux regardless of lighting - "
                              "use 5, the Integrated Camera, instead)")
    parser.add_argument("--list-targets", action="store_true")
    args = parser.parse_args()

    if args.list_targets:
        for name in targets.list_targets():
            t = targets.get_target(name)
            print(f"{name:22s} az={t.azimuth_deg:6.1f} el={t.elevation_deg:6.1f}  {t.note}")
        return 0

    bridge = JointBridge()
    try:
        if args.target:
            resolved_target_name = args.target
        else:
            resolved_target_name = wait_for_detected_target(bridge)
            if resolved_target_name is None:
                print("No target was manually specified (--target) and no object "
                      "was classified by the object-detection pipeline.")
                return 1

        aim = aim_at_target(resolved_target_name, bridge)
        if args.camera is not None:
            run_camera_centering_loop(
                args.camera, bridge, aim, auto_target=not bool(args.target))
        elif not args.target:
            run_detection_only_loop(bridge, aim)
    finally:
        bridge.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
