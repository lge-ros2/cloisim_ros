###############################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(sim_device_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR launch/module)

################################################################################
# Install
################################################################################
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()