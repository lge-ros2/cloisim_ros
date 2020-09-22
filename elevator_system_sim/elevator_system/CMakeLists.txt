###############################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(elevator_system)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

################################################################################
# Find colcon packages and libraries for colcon and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(driver_sim REQUIRED)
find_package(sim_bridge REQUIRED)
find_package(elevator_system_msgs REQUIRED)

################################################################################
# Build
################################################################################
include_directories(include)

add_executable(${PROJECT_NAME}
  src/CElevatorSystem.cpp
  src/main.cpp)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  rosgraph_msgs
  elevator_system_msgs
  driver_sim
  sim_bridge)

################################################################################
# Install
################################################################################
install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

################################################################################
# Test
################################################################################

ament_package()