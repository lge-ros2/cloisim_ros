###############################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(cloisim_ros_bringup)

include("../cloisim_ros_base/cmake/cloisim_ros_package.cmake")
cloisim_ros_package()

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
find_package(cloisim_ros_depthcamera REQUIRED)
find_package(cloisim_ros_multicamera REQUIRED)
find_package(cloisim_ros_realsense REQUIRED)
find_package(cloisim_ros_gps REQUIRED)
find_package(cloisim_ros_elevatorsystem REQUIRED)
find_package(cloisim_ros_groundtruth REQUIRED)
find_package(cloisim_ros_world REQUIRED)
find_package(cloisim_ros_actor REQUIRED)


set(dependencies
  rclcpp
  cloisim_ros_bringup_param
  cloisim_ros_camera
  cloisim_ros_micom
  cloisim_ros_lidar
  cloisim_ros_camera
  cloisim_ros_depthcamera
  cloisim_ros_multicamera
  cloisim_ros_realsense
  cloisim_ros_gps
  cloisim_ros_elevatorsystem
  cloisim_ros_groundtruth
  cloisim_ros_world
  cloisim_ros_actor
)

################################################################################
# Build
################################################################################
add_executable(
  ${PROJECT_NAME}
  src/main.cpp
)

ament_target_dependencies(
  ${PROJECT_NAME}
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
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()