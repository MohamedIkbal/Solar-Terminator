cmake_minimum_required(VERSION 3.8)
project(mearm_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(
  mearm_hardware
  SHARED
  src/arm_system_hardware.cpp
)

target_include_directories(
  mearm_hardware
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(
  mearm_hardware
  PUBLIC
  rclcpp::rclcpp
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
)

target_compile_definitions(mearm_hardware PRIVATE "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  hardware_interface mearm_hardware.xml
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS mearm_hardware
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  FILES mearm_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(mearm_hardware)
ament_export_targets(export_${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
