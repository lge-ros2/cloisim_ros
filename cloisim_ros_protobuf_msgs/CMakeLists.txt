###############################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(cloisim_ros_protobuf_msgs)

find_package(ament_cmake REQUIRED)

#execute_process(COMMAND ${CMAKE_SOURCE_DIR}/gen_proto_code.sh ${CMAKE_SOURCE_DIR})


set(PROTOBUF_INPUT_DIRECTORY "msgs")
set(PROTOBUF_OUTPUT_INCLUDE_DIRECTORY "include/cloisim_msgs")
set(PROTOBUF_OUTPUT_SOURCE_DIRECTORY "src/cloisim_msgs")

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
  message("Generate ${PROTOBUF_FILE}.proto")
  set(PROTOBUF_COMMAND "protoc --proto_path=${PROTOBUF_INPUT_DIRECTORY} --cpp_out=${PROTOBUF_OUTPUT_INCLUDE_DIRECTORY} ${PROTOBUF_FILE}.proto")
  execute_process(COMMAND bash -c ${PROTOBUF_COMMAND} WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} RESULTS_VARIABLE proto_result)
  message("\tresult: ${proto_result}")

  file(RENAME ${PROTOBUF_OUTPUT_INCLUDE_DIRECTORY}/${PROTOBUF_FILE}.pb.cc ${PROTOBUF_OUTPUT_SOURCE_DIRECTORY}/${PROTOBUF_FILE}.pb.cc)
endforeach()

################################################################################
# Build
################################################################################
FILE(GLOB CLOISIM_MSGS_SRCS
  ./src/cloisim_msgs/*.pb.cc)

add_library(${PROJECT_NAME} SHARED
  ${CLOISIM_MSGS_SRCS}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/cloisim_msgs>
  $<INSTALL_INTERFACE:include/cloisim_msgs>
)

################################################################################
# Install
################################################################################
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  INCLUDES DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()