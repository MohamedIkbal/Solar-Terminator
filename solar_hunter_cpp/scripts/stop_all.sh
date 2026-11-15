#!/usr/bin/env bash
# stop_all.sh — stops all 4 mEArm stages started by run_all.sh
pkill -f servo_serial_bridge 2>/dev/null
pkill -f "mearm_description display.launch.py" 2>/dev/null
pkill -f solar_hunter_cpp_node 2>/dev/null
pkill -f mearm_visual_servoing_cpp_node 2>/dev/null
echo "Sent stop signal to all 4 stages."
