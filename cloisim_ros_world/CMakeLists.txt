###############################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(cloisim_ros_world)

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
find_package(rosgraph_msgs REQUIRED)
find_package(cloisim_ros_base REQUIRED)

set(dependencies
  rosgraph_msgs
  cloisim_ros_base)

################################################################################
# Build
################################################################################
include_directories(include)

add_executable(${PROJECT_NAME}
  src/main.cpp
  src/world.cpp)

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