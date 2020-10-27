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
FILE(GLOB PROTOBUF_MSG_SRCS
  ./src/protobuf/*.pb.cc)

add_library(${PROJECT_NAME} SHARED
  ${PROTOBUF_MSG_SRCS}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/protobuf>
  $<INSTALL_INTERFACE:include/protobuf>
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