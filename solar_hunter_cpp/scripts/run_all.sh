#!/usr/bin/env bash
# run_all.sh — launches the full mEArm + solar_hunter_cpp stack in 4 xterm
# windows (one per original terminal) and reports what it started.
#
# Usage:
#   ./scripts/run_all.sh                       # default ports/camera
#   SERIAL_PORT=/dev/ttyACM1 CAMERA_INDEX=0 ./scripts/run_all.sh
#
# Each window keeps running after this script returns; close a window
# (or Ctrl+C inside it) to stop that stage. Run ./scripts/stop_all.sh
# to kill everything at once.

set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export SERIAL_PORT="${SERIAL_PORT:-/dev/ttyACM0}"
export CAMERA_INDEX="${CAMERA_INDEX:-4}"
export TARGET="${TARGET:-moon_south_darkside}"

echo "Starting serial bridge (port=$SERIAL_PORT)..."
xterm -T "1-serial-bridge" -hold -e "$HERE/01_serial_bridge.sh" &
sleep 3

echo "Starting robot description + RViz (port=$SERIAL_PORT)..."
xterm -T "2-description-rviz" -hold -e "$HERE/02_description_rviz.sh" &
sleep 5

echo "Starting object detection (camera=$CAMERA_INDEX, target=$TARGET)..."
xterm -T "3-object-detection" -hold -e "$HERE/03_object_detection.sh" &
sleep 3

echo "Starting C++ visual servoing..."
xterm -T "4-visual-servoing" -hold -e "$HERE/04_visual_servoing.sh" &

echo "All 4 stages launched in separate xterm windows."
echo "Check with: ros2 node list"
