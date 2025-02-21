###############################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(cloisim_ros_micom)

include("../cloisim_ros_base/cmake/cloisim_ros_package.cmake")
cloisim_ros_package()

################################################################################
# Find colcon packages and libraries for colcon and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cloisim_ros_msgs REQUIRED)
find_package(cloisim_ros_base REQUIRED)
find_package(cloisim_ros_bringup_param REQUIRED)

set(dependencies
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  std_srvs
  cloisim_ros_msgs
  cloisim_ros_base
  cloisim_ros_bringup_param
)

################################################################################
# Build
################################################################################
add_library(
  ${PROJECT_NAME}_core SHARED
  src/micom.cpp
)

target_include_directories(
  ${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  ${PROJECT_NAME}_core
  ${dependencies}
)

add_executable(
  ${STANDALONE_EXEC_NAME}
  src/main.cpp
)

target_link_libraries(
  ${STANDALONE_EXEC_NAME} PRIVATE
  ${PROJECT_NAME}_core
)

################################################################################
# Install
################################################################################
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}_core
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  INCLUDES DESTINATION include
)

install(
  TARGETS ${STANDALONE_EXEC_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

ament_package()