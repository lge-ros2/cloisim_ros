# Contributing to cloisim_ros

Thank you for your interest in contributing to cloisim_ros! This document outlines the guidelines for contributing.

## Getting Started

1. Fork the repository on GitHub
2. Clone your fork with submodules:
   ```bash
   git clone --recursive https://github.com/<your-username>/cloisim_ros.git -b jazzy
   ```
3. Set up the development environment (see [copilot-instructions.md](copilot-instructions.md) and [../CLAUDE.md](../CLAUDE.md))

## Development Workflow

### Branching

- Create a feature branch from `jazzy`:
  ```bash
  git checkout -b feature/your-feature-name jazzy
  ```
- Use descriptive branch names: `feature/`, `fix/`, `refactor/`

### Making Changes

1. Build and verify the project compiles:
   ```bash
   colcon build --symlink-install --packages-up-to cloisim_ros_bringup
   ```
2. If modifying a sensor package, test it individually:
   ```bash
   colcon build --packages-select <package_name> --cmake-args -DBUILD_TESTING=ON
   colcon test --packages-select <package_name> --event-handlers console_direct+
   ```
3. Verify all tests pass:
   ```bash
   colcon test-result --all
   ```

### Code Style

- Follow the `.clang-format` configuration (Google-based, 100-column limit, C++17)
- Format your code before committing:
  ```bash
  clang-format -i <file>
  ```
- All packages run `ament_cppcheck` — ensure no warnings are introduced
- Use the logging macros defined in `cloisim_ros_base/base.hpp`: `INFO`, `WARN`, `ERR`, `INFO_ONCE`, etc.

### Commit Messages

- For Harness Engineering work, write commit messages in English only.
- Use this format:
   - first line: `type(scope): summary` or `type: summary`
   - following lines: indented bullet list with `- ` items describing concrete changes and effects
- Keep the subject concise and technical.
- Do not write Harness Engineering commit messages in Korean.

Example:

```text
fix(cloth): improve resting stability on multi-support contacts
      - include contact threshold in ClothGrabberPlugin cache key to avoid stale group activation results
      - reuse cached collider bounds in ClothGrabber to reduce repeated hot-path contact work
      - skip BurstCloth collider updates when collider state is unchanged to avoid needless wake-ups
      - make sleep detection tolerate persistent low-energy resting contact
      - strengthen resting-contact tangential snap and static-friction dead-zone in BurstCloth
      - filter scene collider candidates more conservatively in ClothPlugin to reduce irrelevant contact noise
```

## Adding a New Sensor Package

1. Create a new directory `cloisim_ros_<sensor>/` with:
   ```
   cloisim_ros_<sensor>/
   ├── CMakeLists.txt
   ├── package.xml
   ├── README.md
   ├── include/cloisim_ros_<sensor>/
   │   └── <sensor>.hpp
   └── src/
       ├── <sensor>.cpp
       └── main.cpp
   ```

2. Inherit from `cloisim_ros::Base` and implement:
   - `Initialize()` — set up publishers, create bridges, start workers
   - `Deinitialize()` — cleanup

3. Use the shared cmake macro in your `CMakeLists.txt`:
   ```cmake
   include("../cloisim_ros_base/cmake/cloisim_ros_package.cmake")
   cloisim_ros_package()
   ```

4. Add the package to `cloisim_ros_bringup/CMakeLists.txt` as a dependency

5. Add a test file in `test/` using `ament_cmake_gtest` and the mock bridge server:
   ```cmake
   if(BUILD_TESTING)
     find_package(ament_cmake_gtest REQUIRED)
     ament_add_gtest(test_<sensor>_node test/test_<sensor>_node.cpp TIMEOUT 30)
     target_include_directories(test_<sensor>_node PRIVATE
       ${CMAKE_CURRENT_SOURCE_DIR}/../cloisim_ros_base/test
     )
     target_link_libraries(test_<sensor>_node ${PROJECT_NAME}_core zmq)
   endif()
   ```

## Adding Protobuf Messages

1. Add `.proto` files to `cloisim_ros_protobuf_msgs/msgs/`
2. Register the new message in `cloisim_ros_protobuf_msgs/CMakeLists.txt` by appending to the `PROTOBUF_DEFINITION_FILES` list

## Pull Requests

- Ensure all tests pass before submitting
- Provide a clear description of what changed and why
- Link related issues
- Keep PRs focused — one feature or fix per PR

## Reporting Issues

Use the GitHub issue templates:
- **Bug Report**: Describe the bug, steps to reproduce, and expected behavior
- **Feature Request**: Describe the problem and proposed solution

## License

By contributing, you agree that your contributions will be licensed under the [MIT License](../LICENSE).

## Maintainers

- Hyunseok Yang (hyunseok7.yang@lge.com)
- Sungkyu Kang (sungkyu.kang@lge.com)
