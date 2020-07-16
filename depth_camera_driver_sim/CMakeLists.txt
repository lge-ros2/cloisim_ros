###############################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(depth_camera_driver_sim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fstack-protector -O2)
endif()

################################################################################
# Find colcon packages and libraries for colcon and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(image_transport REQUIRED)
find_package(sim_bridge REQUIRED)
find_package(driver_sim REQUIRED)

################################################################################
# Build
################################################################################
include_directories(include)

add_executable(${PROJECT_NAME}
  src/main.cpp
  src/DepthCameraDriverSim.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  camera_info_manager
  image_transport
  sim_bridge
  driver_sim)

################################################################################
# Install
################################################################################
install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

################################################################################
# Test
################################################################################

ament_package()
