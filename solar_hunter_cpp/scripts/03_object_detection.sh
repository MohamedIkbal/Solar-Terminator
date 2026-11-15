#!/usr/bin/env bash
# Terminal 3 — Object Detection (C++ solar_hunter_cpp port)
#
# Runs BOTH pieces of stage 1+2 together, same split as the Python
# pipeline's 03_object_detection.sh:
#   1. mearm_object_detection_node (ONNX classifier) owns the camera and
#      classifies what's in frame into: space_debris, meteor, locust,
#      solar_sail, moon_south_darkside. It publishes the label on
#      /object_detection/detections and shows its own debug window.
#   2. solar_hunter_cpp_node subscribes to that topic and keeps
#      re-aiming the reflector (publishing /arm_joint_commands) every
#      time the classified target changes.
#
# solar_hunter_cpp_node is started with camera_index=-1 (aim-only) here:
# the camera is a single physical device (V4L2 does not allow two
# processes to open it at once on this hardware), and the classifier
# above already owns it. The LED/light centering (stage 2/3 fine
# correction) that a live camera would add is therefore skipped in this
# mode; coarse aiming still tracks whatever the classifier detects via
# auto_target + /object_detection/detections.
set -e
cd /home/ikbal/ros2_ws
source install/setup.bash
source /home/ikbal/solar_hunter_cpp/install/setup.bash

# NOTE: camera_index default is 4 on this machine (/dev/video4 = Integrated Camera).
#       index 5 exists as a device node but OpenCV cannot open it for capture.
#       Override with: CAMERA_INDEX=<n> ./scripts/03_object_detection.sh
CAMERA_INDEX="${CAMERA_INDEX:-4}"
TARGET="${TARGET:-moon_south_darkside}"

echo "[03_object_detection] starting classifier on camera_index=$CAMERA_INDEX (/dev/video$CAMERA_INDEX)"
ros2 launch mearm_object_detection mearm_object_detection.launch.py \
  camera_index:="$CAMERA_INDEX" &
CLASSIFIER_PID=$!
trap 'kill "$CLASSIFIER_PID" 2>/dev/null' EXIT

sleep 3
echo "[03_object_detection] starting solar_hunter_cpp reflector auto-retarget (target=$TARGET, aim-only, camera owned by classifier)..."
exec ros2 run solar_hunter_cpp solar_hunter_cpp_node \
  --ros-args \
  -p camera_index:=-1 \
  -p target:="$TARGET" \
  -p publish_rate_hz:=10.0 \
  -p show_debug_windows:=false \
  -p auto_target:=true \
  -p target_is_manual:=false
