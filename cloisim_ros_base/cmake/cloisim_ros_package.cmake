macro(cloisim_ros_package)

  if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to Release as none was specified.")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)

    # Set the possible values of build type for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
  endif()

  # Default to C++17
  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
  endif()

  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wdeprecated -fPIC)
    add_compile_options(-Wno-unused-function)
    add_compile_options(-fstack-protector -O2)
    # add_compile_options(-Werror)
  endif()

  set(STANDALONE_EXEC_NAME "standalone")

endmacro()