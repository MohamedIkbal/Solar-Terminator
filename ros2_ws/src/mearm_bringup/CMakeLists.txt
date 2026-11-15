# CMakeLists.txt — mearm_bringup
# ===================================
# Build configuration for the bringup package.
#
# This is a "leaf" package: it contains no compiled code, only launch files,
# YAML configs, RViz presets, and a Python teleop script.  Everything is
# installed directly into the package share directory.
cmake_minimum_required(VERSION 3.8)
project(mearm_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# ── Install launch files ────────────────────────────────────────────────────
# bringup.launch.py (and any future launch files) are installed into
# share/mearm_bringup/launch/ so ros2 launch can find them.
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# ── Install controller YAML configs ─────────────────────────────────────────
# controllers.yaml is the single source of truth for all controller_manager
# parameters (controller types, joints, interfaces, constraints).
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# ── Install RViz presets ────────────────────────────────────────────────────
# display.rviz contains saved panel layouts, topic subscriptions, and
# visualisation settings for the arm.
install(
  DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

# ── Install Python scripts ──────────────────────────────────────────────────
# keyboard_teleop.py is installed as an executable named 'keyboard_teleop'
# so it can be run with: ros2 run mearm_bringup keyboard_teleop
install(
  PROGRAMS src/keyboard_teleop.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME keyboard_teleop
)

# servo_serial_bridge.py is the direct ROS 2 <-> STM32 serial link (real
# hardware only). It used to live in a separate pca9685_multiple package;
# that package no longer exists, so it is installed from here instead and
# run with: ros2 run mearm_bringup servo_serial_bridge
install(
  PROGRAMS src/servo_serial_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME servo_serial_bridge
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
