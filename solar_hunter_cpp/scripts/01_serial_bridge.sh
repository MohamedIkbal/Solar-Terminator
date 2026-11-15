#!/usr/bin/env bash
# Terminal 1 — Serial Bridge to STM32 (with S-curve safety net)
set -e
cd /home/ikbal/ros2_ws
source install/setup.bash
source /home/ikbal/solar_hunter_cpp/install/setup.bash
exec ros2 run pca9685_multiples servo_serial_bridge --ros-args \
  -p port:="${SERIAL_PORT:-/dev/ttyACM0}" \
  -p enable_scurve:=true \
  -p scurve_duration_s:=0.3 \
  -p scurve_update_hz:=50.0
