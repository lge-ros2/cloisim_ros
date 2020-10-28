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
find_package(driver_sim_interfaces REQUIRED)

set(dependencies
  sim_bridge
  driver_sim_interfaces
)

################################################################################
# Build
################################################################################
include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/driver_sim.cpp)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
  tf2_ros
  ${dependencies})

################################################################################
# Install
################################################################################
ament_export_interfaces(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_libraries(${PROJECT_NAME} ${dependencies})
ament_export_include_directories(include)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()