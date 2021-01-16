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

set(PROTOBUF_MESSAGES_NAME "cloisim_msgs")
set(PROTOBUF_INPUT_DIRECTORY "msgs")
set(PROTOBUF_OUTPUT_INCLUDE_DIRECTORY "include/${PROTOBUF_MESSAGES_NAME}")
set(PROTOBUF_OUTPUT_SOURCE_DIRECTORY "src/${PROTOBUF_MESSAGES_NAME}")

file(MAKE_DIRECTORY "${PROTOBUF_OUTPUT_INCLUDE_DIRECTORY}")
file(MAKE_DIRECTORY "${PROTOBUF_OUTPUT_SOURCE_DIRECTORY}")

# file(GLOB PROTOBUF_DEFINITION_FILES "${PROTOBUF_INPUT_DIRECTORY}/*.proto")
list(APPEND PROTOBUF_DEFINITION_FILES "any;param;param_v;color")
list(APPEND PROTOBUF_DEFINITION_FILES "time;vector2d;vector3d;quaternion;pose")
list(APPEND PROTOBUF_DEFINITION_FILES "imu;image;images_stamped;image_stamped;camerasensor;distortion")
list(APPEND PROTOBUF_DEFINITION_FILES "laserscan;laserscan_stamped")
list(APPEND PROTOBUF_DEFINITION_FILES "micom;battery;pointcloud;gps")

foreach(file ${PROTOBUF_DEFINITION_FILES})
  get_filename_component(PROTOBUF_FILE ${file} NAME_WLE)
  set(PROTOBUF_COMMAND "protoc --proto_path=${PROTOBUF_INPUT_DIRECTORY} --cpp_out=${PROTOBUF_OUTPUT_INCLUDE_DIRECTORY} ${PROTOBUF_FILE}.proto")
  execute_process(COMMAND bash -c ${PROTOBUF_COMMAND} WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} RESULTS_VARIABLE proto_result)
  # message("Generate ${PROTOBUF_FILE}.proto -> result: ${proto_result}")
  file(RENAME ${PROTOBUF_OUTPUT_INCLUDE_DIRECTORY}/${PROTOBUF_FILE}.pb.cc ${PROTOBUF_OUTPUT_SOURCE_DIRECTORY}/${PROTOBUF_FILE}.pb.cc)
endforeach()

################################################################################
# Build
################################################################################
file(GLOB CLOISIM_MSGS_SRCS
  ./src/${PROTOBUF_MESSAGES_NAME}/*.pb.cc)

add_library(
  ${PROJECT_NAME} SHARED
  ${CLOISIM_MSGS_SRCS}
)

target_include_directories(
  ${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROTOBUF_MESSAGES_NAME}>
  $<INSTALL_INTERFACE:include/${PROTOBUF_MESSAGES_NAME}>
)

################################################################################
# Install
################################################################################
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

ament_package()