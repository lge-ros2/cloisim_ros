###############################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(cloisim_ros_base)

include("cmake/cloisim_ros_package.cmake")
cloisim_ros_package()

################################################################################
# Find colcon packages and libraries for colcon and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cloisim_ros_bridge_zmq REQUIRED)
find_package(cloisim_ros_protobuf_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# if(BUILD_TESTING)
  find_package(ament_cmake_cppcheck REQUIRED)
  ament_cppcheck()
# endif()

set(dependencies
  rclcpp
  tf2_ros
  cloisim_ros_bridge_zmq
  cloisim_ros_protobuf_msgs
  geometry_msgs
)

################################################################################
# Build
################################################################################
add_library(
  ${PROJECT_NAME} SHARED
  src/base.cpp
)

target_include_directories(
  ${PROJECT_NAME} PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  ${PROJECT_NAME}
  ${dependencies}
)

################################################################################
# Install
################################################################################
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  INCLUDES DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

ament_package()