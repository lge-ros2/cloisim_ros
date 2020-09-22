cmake_minimum_required(VERSION 3.5)
project(elevator_system_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  msg/ElevatorStatus.msg
)

set(srv_files
  srv/ReturnBool.srv
  srv/StringValue.srv
  srv/CallElevator.srv
  srv/GetElevatorInformation.srv
  srv/RequestDoor.srv
  srv/SelectElevatorFloor.srv
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  ${msg_files}
  DEPENDENCIES builtin_interfaces std_msgs
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()