#!/usr/bin/env bash
# Terminal 3 — Object Detection
#
# Runs BOTH pieces of stage 1+2 together:
#   1. mearm_object_detection_node (ONNX classifier) owns the camera and
#      classifies what's in frame into: space_debris, meteor, locust,
#      solar_sail, moon_south_darkside. It publishes the label on
#      /object_detection/detections and shows its own debug window
#      (classification overlay) since show_debug_windows:true in its
#      params yaml.
#   2. main.py (no --target, no --camera) subscribes to that topic and
#      keeps re-aiming the reflector (publishing /arm_joint_commands)
#      every time the classified target changes.
#
# main.py is NOT given --camera here: the camera is a single physical
# device (V4L2 does not allow two processes to open it at once on this
# hardware), and the classifier above already owns it. The LED/light
# centering (stage 2/3 fine correction) that --camera would add is
# therefore skipped in this mode; coarse aiming still tracks whatever
# the classifier detects.
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd /home/ikbal/ros2_ws
source install/setup.bash

CAMERA_INDEX="${CAMERA_INDEX:-4}"
echo "[03_object_detection] starting classifier on camera_index=$CAMERA_INDEX (/dev/video$CAMERA_INDEX)"

ros2 launch mearm_object_detection mearm_object_detection.launch.py \
  camera_index:="$CAMERA_INDEX" &
CLASSIFIER_PID=$!
trap 'kill "$CLASSIFIER_PID" 2>/dev/null' EXIT

sleep 3
echo "[03_object_detection] starting reflector auto-retarget (waiting for a classified target)..."
exec python3 "$HERE/main.py"
