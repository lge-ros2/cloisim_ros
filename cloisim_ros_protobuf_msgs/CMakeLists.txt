###############################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(cloisim_ros_protobuf_msgs)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

################################################################################
# Find colcon packages and libraries for colcon and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(Protobuf REQUIRED)

set(PROTOBUF_MSGS_DIRECTORY "msgs")

# file(GLOB PROTOBUF_DEFINITION_FILES "${PROTOBUF_INPUT_DIRECTORY}/*.proto")
list(APPEND PROTOBUF_DEFINITION_FILES "any;param;param_v;color")
list(APPEND PROTOBUF_DEFINITION_FILES "header;time;vector2d;vector3d;quaternion;pose;twist")
list(APPEND PROTOBUF_DEFINITION_FILES "imu;image;images_stamped;image_stamped;camerasensor;distortion")
list(APPEND PROTOBUF_DEFINITION_FILES "laserscan;laserscan_stamped")
list(APPEND PROTOBUF_DEFINITION_FILES "micom;battery;pointcloud;gps")

set(PROTOBUF_IMPORT_DIRS)
set(Protobuf_IMPORT_DIRS ${PROJECT_SOURCE_DIR}/${PROTOBUF_MSGS_DIRECTORY})

set(protobuf_msgs)
foreach(file ${PROTOBUF_DEFINITION_FILES})
  get_filename_component(PROTOBUF_FILE ${file} NAME_WLE)
  message(STATUS "Generate ${PROTOBUF_FILE}.proto")
  set(protobuf_msgs ${protobuf_msgs} ${PROJECT_SOURCE_DIR}/${PROTOBUF_MSGS_DIRECTORY}/${PROTOBUF_FILE}.proto)
endforeach()

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${protobuf_msgs})
# message(${protobuf_msgs})
# message("\n${PROTO_SRCS}\n ${PROTO_HDRS}\n")

################################################################################
# Build
################################################################################
add_library(
  ${PROJECT_NAME} SHARED
  ${PROTO_SRCS}
)

target_include_directories(
  ${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROTOBUF_MESSAGES_NAME}>
  $<INSTALL_INTERFACE:include/${PROTOBUF_MESSAGES_NAME}>
)

target_link_libraries(
  ${PROJECT_NAME} PUBLIC
  ${PROTOBUF_LIBRARIES}
)

################################################################################
# Install
################################################################################
install(
  FILES ${PROTO_HDRS}
  DESTINATION include/cloisim_msgs
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()