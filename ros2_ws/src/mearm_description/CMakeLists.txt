cmake_minimum_required(VERSION 3.8)
project(mearm_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(gz_ros2_control REQUIRED)
find_package(controller_manager REQUIRED)
find_package(joint_trajectory_controller REQUIRED)
find_package(joint_state_broadcaster REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# C++ launchers that replace launch/gazebo.launch.py and launch/display.launch.py
# (the find_package() calls above are used by the URDF/xacro and the launch
# files these executables replace; these two executables are the only C++
# code in this otherwise description-only package)
add_executable(mearm_gazebo_launch src/mearm_gazebo_launch.cpp)
target_compile_features(mearm_gazebo_launch PUBLIC c_std_99 cxx_std_17)
target_link_libraries(mearm_gazebo_launch ament_index_cpp::ament_index_cpp)

add_executable(mearm_display_launch src/mearm_display_launch.cpp)
target_compile_features(mearm_display_launch PUBLIC c_std_99 cxx_std_17)
target_link_libraries(mearm_display_launch ament_index_cpp::ament_index_cpp)

install(TARGETS mearm_gazebo_launch mearm_display_launch
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/)

# Install urdf files
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}/)

# Install worlds files
install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}/)

# Install meshes files
install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}/)

# Install rviz files
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
