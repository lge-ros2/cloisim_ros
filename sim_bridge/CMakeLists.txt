###############################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(sim_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fstack-protector -O2)
endif()

find_package(ament_cmake REQUIRED)

################################################################################
# Build
################################################################################
include_directories(include)

set(dependencies
  zmq
  protobuf)

add_library(${PROJECT_NAME} SHARED
  src/sim_bridge.cpp)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${dependencies})

################################################################################
# Install
################################################################################
ament_export_interfaces(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
# ament_export_dependencies(${dependencies})
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} ${dependencies})

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  # RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()