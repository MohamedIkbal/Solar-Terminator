cmake_minimum_required(VERSION 3.8)
project(mearm_object_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Add executable
add_executable(${PROJECT_NAME}_node src/mearm_object_detection_node.cpp)
target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME}_node PUBLIC c_std_99 cxx_std_17)

# NOTE: this ROS install exports modern CMake imported targets and no longer
# provides the classic ament_target_dependencies() macro, so we link
# directly against the namespaced imported targets instead.
target_include_directories(${PROJECT_NAME}_node PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  /usr/include/onnxruntime)
target_link_libraries(
  ${PROJECT_NAME}_node
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${trajectory_msgs_TARGETS}
  ${vision_msgs_TARGETS}
  ${OpenCV_LIBS}
  ${std_msgs_TARGETS}
  onnxruntime
)

# C++ launcher that replaces launch/mearm_object_detection.launch.py
add_executable(${PROJECT_NAME}_launch src/mearm_object_detection_launch.cpp)
target_compile_features(${PROJECT_NAME}_launch PUBLIC c_std_99 cxx_std_17)
target_link_libraries(${PROJECT_NAME}_launch ament_index_cpp::ament_index_cpp)

# Install executable
install(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}_launch
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/)

# Install model files (classifier weights, labels.txt)
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}/)

# Install Python script 'generate_classifier_model' as a ROS2 executable
install(PROGRAMS scripts/generate_classifier_model.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME generate_classifier_model)

# Install Python script 'train_classifier' as a ROS2 executable
install(PROGRAMS scripts/train_classifier.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME train_classifier)

# Install Python script 'verify_trained_model' as a ROS2 executable
install(PROGRAMS scripts/verify_trained_model.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME verify_trained_model)

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
