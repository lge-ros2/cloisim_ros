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
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sim_bridge REQUIRED)

file(GLOB protobuf_msg_srcs
  ./include/protobuf/*.pb.cc
)

set(dependencies
  rclcpp
  tf2_geometry_msgs
  tf2_ros
  sim_bridge
)

################################################################################
# Build
################################################################################
add_library(${PROJECT_NAME} SHARED
  src/driver_sim.cpp
  ${protobuf_msg_srcs}
)

target_include_directories(${PROJECT_NAME} PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

################################################################################
# Install
################################################################################
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  INCLUDES DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()