cmake_minimum_required(VERSION 3.8)
project(solar_hunter_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_executable(${PROJECT_NAME}_node src/solar_hunter_node.cpp src/targets.cpp src/reflector_angle.cpp src/detector.cpp)
target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME}_node PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}_node
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${vision_msgs_TARGETS}
  ${OpenCV_LIBS})

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
