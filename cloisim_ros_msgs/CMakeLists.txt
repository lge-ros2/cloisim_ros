cmake_minimum_required(VERSION 3.5)
project(cloisim_ros_msgs)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# file(GLOB_RECURSE msg_files RELATIVE ${PROJECT_SOURCE_DIR} msg/*.msg)
file(GLOB_RECURSE srv_files RELATIVE ${PROJECT_SOURCE_DIR} srv/*.srv)

rosidl_generate_interfaces(${PROJECT_NAME}
  # ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces geometry_msgs
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()