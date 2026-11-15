#!/usr/bin/env bash
# Terminal 4 — C++ Visual Servoing (S-curve enabled)
set -e
cd /home/ikbal/ros2_ws
source install/setup.bash
exec ros2 run mearm_visual_servoing_cpp mearm_visual_servoing_cpp_node \
  --ros-args \
  -p max_joint_speed_rad_s:=0.3 \
  -p smoothing_alpha:=0.1 \
  -p use_scurve_smoothing:=true \
  -p scurve_duration_s:=0.4 \
  -p scurve_min_delta_rad:=0.01
