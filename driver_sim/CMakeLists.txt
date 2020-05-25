###############################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(driver_sim)

execute_process(COMMAND ${CMAKE_SOURCE_DIR}/gen_proto_code.sh ${CMAKE_SOURCE_DIR}/include)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fstack-protector -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sim_bridge REQUIRED)

################################################################################
# Build
################################################################################
include_directories(include)

FILE(GLOB PROTOBUF_MSG_SRCS
  ./include/protobuf/*.pb.cc)

add_library(${PROJECT_NAME} SHARED
  src/driver_sim.cpp
  ${PROTOBUF_MSG_SRCS})

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${dependencies})

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
  tf2_ros
  sim_bridge)

################################################################################
# Install
################################################################################
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib/
  LIBRARY DESTINATION lib/
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} ${dependencies})

ament_package()