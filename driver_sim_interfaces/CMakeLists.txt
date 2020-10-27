###############################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(driver_sim_interfaces)

find_package(ament_cmake REQUIRED)

execute_process(COMMAND ${CMAKE_SOURCE_DIR}/gen_proto_code.sh ${CMAKE_SOURCE_DIR})

################################################################################
# Build
################################################################################
include_directories(include/protobuf)

FILE(GLOB PROTOBUF_MSG_SRCS
  ./src/protobuf/*.pb.cc)

add_library(${PROJECT_NAME} SHARED
  ${PROTOBUF_MSG_SRCS})

################################################################################
# Install
################################################################################
install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_interfaces(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_libraries(${PROJECT_NAME} ${dependencies})
ament_export_include_directories(include)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  INCLUDES DESTINATION include
)

ament_package()