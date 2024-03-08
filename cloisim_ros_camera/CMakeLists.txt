###############################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(cloisim_ros_camera)

include("../cloisim_ros_base/cmake/cloisim_ros_package.cmake")
cloisim_ros_package()

set(STANDALONE_CAMERA "camera")
set(STANDALONE_DEPTHCAMERA "depth_camera")
set(STANDALONE_SEGMENTATIONCAMERA "segmentation_camera")

################################################################################
# Find colcon packages and libraries for colcon and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(image_transport REQUIRED)
find_package(cloisim_ros_base REQUIRED)
find_package(cloisim_ros_bringup_param REQUIRED)
find_package(vision_msgs REQUIRED)

set(dependencies
  sensor_msgs
  camera_info_manager
  image_transport
  cloisim_ros_base
  cloisim_ros_bringup_param
  vision_msgs
)

################################################################################
# Build
################################################################################
add_library(${PROJECT_NAME}_core SHARED
  src/camera_base.cpp
  src/camera.cpp
  src/depth_camera.cpp
  src/segmentation_camera.cpp
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
  ${STANDALONE_CAMERA}
  src/main_cam.cpp
)

target_link_libraries(
  ${STANDALONE_CAMERA} PRIVATE
  ${PROJECT_NAME}_core
)

add_executable(
  ${STANDALONE_DEPTHCAMERA}
  src/main_depthcam.cpp
)

target_link_libraries(
  ${STANDALONE_DEPTHCAMERA} PRIVATE
  ${PROJECT_NAME}_core
)

add_executable(
  ${STANDALONE_SEGMENTATIONCAMERA}
  src/main_segcam.cpp
)

target_link_libraries(
  ${STANDALONE_SEGMENTATIONCAMERA} PRIVATE
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
  TARGETS ${STANDALONE_CAMERA}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${STANDALONE_DEPTHCAMERA}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${STANDALONE_SEGMENTATIONCAMERA}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

ament_package()