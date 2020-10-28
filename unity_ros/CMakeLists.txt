###############################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(unity_ros)

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
find_package(rosgraph_msgs REQUIRED)
find_package(driver_sim_interfaces REQUIRED)
find_package(sim_bridge REQUIRED)

set(dependencies
  rclcpp
  rosgraph_msgs
  sim_bridge
  driver_sim_interfaces
)

################################################################################
# Build
################################################################################
include_directories(include)

add_executable(${PROJECT_NAME}
  src/main.cpp
  src/unity_ros_init.cpp)

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

################################################################################
# Install
################################################################################
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()