#!/usr/bin/env bash
# Terminal 2 — Robot Description + RViz
set -e
cd /home/ikbal/ros2_ws
source install/setup.bash
source /home/ikbal/solar_hunter_cpp/install/setup.bash
exec ros2 launch mearm_description display.launch.py \
  use_mock_hardware:=false \
  serial_port:="${SERIAL_PORT:-/dev/ttyACM0}" \
  start_rviz:=true
