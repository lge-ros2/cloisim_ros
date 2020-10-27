###############################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(camera_driver_sim)

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
find_package(driver_sim_interfaces REQUIRED)
find_package(driver_sim REQUIRED)

################################################################################
# Build
################################################################################
include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/CameraDriverSim.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  camera_info_manager
  image_transport
  sim_bridge
  driver_sim_interfaces
  driver_sim)


add_executable(${PROJECT_NAME}-bin
  src/main.cpp
)

target_link_libraries(${PROJECT_NAME}-bin ${PROJECT_NAME})

ament_target_dependencies(${PROJECT_NAME}-bin
  rclcpp
  sim_bridge
  driver_sim)

################################################################################
# Install (Execution)
################################################################################
install(TARGETS ${PROJECT_NAME}-bin DESTINATION lib/${PROJECT_NAME})
set_target_properties(${PROJECT_NAME}-bin PROPERTIES OUTPUT_NAME ${PROJECT_NAME})

################################################################################
# Install (Library)
################################################################################
ament_export_interfaces(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
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
  INCLUDES DESTINATION include
)

ament_package()