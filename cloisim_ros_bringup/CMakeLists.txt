###############################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(cloisim_ros_bringup)

include("../cloisim_ros_base/cmake/cloisim_ros_package.cmake")
cloisim_ros_package()

set(STANDALONE_EXEC_NAME "bringup")

################################################################################
# Find colcon packages and libraries for colcon and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cloisim_ros_bringup_param REQUIRED)
find_package(cloisim_ros_websocket_service REQUIRED)
find_package(cloisim_ros_camera REQUIRED)
find_package(cloisim_ros_micom REQUIRED)
find_package(cloisim_ros_lidar REQUIRED)
find_package(cloisim_ros_camera REQUIRED)
find_package(cloisim_ros_multicamera REQUIRED)
find_package(cloisim_ros_realsense REQUIRED)
find_package(cloisim_ros_gps REQUIRED)
find_package(cloisim_ros_imu REQUIRED)
find_package(cloisim_ros_sonar REQUIRED)
find_package(cloisim_ros_elevator_system REQUIRED)
find_package(cloisim_ros_ground_truth REQUIRED)
find_package(cloisim_ros_world REQUIRED)
find_package(cloisim_ros_actor REQUIRED)
find_package(cloisim_ros_joint_control REQUIRED)


set(dependencies
  rclcpp
  cloisim_ros_bringup_param
  cloisim_ros_camera
  cloisim_ros_micom
  cloisim_ros_lidar
  cloisim_ros_camera
  cloisim_ros_multicamera
  cloisim_ros_realsense
  cloisim_ros_gps
  cloisim_ros_imu
  cloisim_ros_sonar
  cloisim_ros_elevator_system
  cloisim_ros_ground_truth
  cloisim_ros_world
  cloisim_ros_actor
  cloisim_ros_joint_control
)

################################################################################
# Build
################################################################################
add_executable(
  ${STANDALONE_EXEC_NAME}
  src/main.cpp
  src/bringup.cpp
)

ament_target_dependencies(
  ${STANDALONE_EXEC_NAME}
  ${dependencies}
)

################################################################################
# Install
################################################################################
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS ${STANDALONE_EXEC_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()