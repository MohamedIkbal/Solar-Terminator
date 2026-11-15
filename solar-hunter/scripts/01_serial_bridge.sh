#!/usr/bin/env bash
# Terminal 1 — Serial Bridge to STM32 (with S-curve safety net)
#
# IMPORTANT: terminal 2 (02_description_rviz.sh -> display.launch.py
# use_mock_hardware:=false) already starts its OWN servo_serial_bridge
# (from mearm_bringup, wired into the ros2_control stack) on the same
# serial port. Running BOTH bridges at once means two processes write to
# /dev/ttyACM0 simultaneously, which garbles the commands the STM32
# receives and is why the arm stops responding. Guard against that here.
set -e
cd /home/ikbal/ros2_ws
source install/setup.bash

PORT="${SERIAL_PORT:-/dev/ttyACM0}"
if command -v fuser >/dev/null 2>&1 && fuser "$PORT" >/dev/null 2>&1; then
  echo "[01_serial_bridge] $PORT is already held by another process:"
  fuser -v "$PORT" 2>&1 || true
  echo "[01_serial_bridge] Refusing to start a second bridge on the same port."
  echo "[01_serial_bridge] If you are running terminal 2 with use_mock_hardware:=false,"
  echo "[01_serial_bridge] that launch already owns the serial bridge -- skip this terminal."
  exit 1
fi

exec ros2 run pca9685_multiples servo_serial_bridge --ros-args \
  -p port:="$PORT" \
  -p enable_scurve:=true \
  -p scurve_duration_s:=0.3 \
  -p scurve_update_hz:=50.0
