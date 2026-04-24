# Developer Instructions

## Overview

**cloisim_ros** is a collection of ROS 2 (Jazzy) packages that provide virtual sensor and actuator nodes for [CLOiSim](https://github.com/lge-ros2/cloisim), a Unity3D-based multi-robot simulator. Communication between CLOiSim and these ROS nodes happens over **ZMQ** (data/info) and **WebSocket** (service port discovery), using **Protobuf** for message serialization.

## Prerequisites

- **ROS 2 Jazzy** on Ubuntu 24.04
- C++17 compiler (GCC or Clang)
- CMake ≥ 3.5
- System libraries: `libzmq3-dev`, `protobuf-dev`, `libwebsocketpp-dev`, `libjsoncpp-dev`, `asio`
- CLOiSim simulator running (local or remote)

## Workspace Setup

```bash
mkdir -p ~/cloisim_ws/src
cd ~/cloisim_ws/src
git clone --recursive https://github.com/lge-ros2/cloisim_ros.git -b jazzy
```

> **Note:** `--recursive` is required to pull the `cloi_common_interfaces` submodule.

### Install Dependencies

```bash
sudo apt update
sudo apt install -y python3-rosdep
sudo rosdep init        # skip if already done
rosdep update
rosdep install -y -r -q --from-paths src --ignore-src --rosdistro jazzy
```

## Building

```bash
source /opt/ros/jazzy/setup.bash
cd ~/cloisim_ws
colcon build --symlink-install --packages-up-to cloisim_ros_bringup
```

### Build a Single Package

```bash
colcon build --symlink-install --packages-select cloisim_ros_lidar
```

### Build with Tests Enabled

```bash
colcon build --packages-select <package_name> --cmake-args -DBUILD_TESTING=ON
```

## Running

### Environment Variables

| Variable | Default | Description |
|---|---|---|
| `CLOISIM_BRIDGE_IP` | `127.0.0.1` | IP address of the CLOiSim simulator |
| `CLOISIM_SERVICE_PORT` | `8080` | WebSocket service port for port discovery |
| `ROS_DOMAIN_ID` | `0` | ROS 2 domain ID |

### Bringup (All Sensors)

```bash
source ~/cloisim_ws/install/setup.bash
ros2 launch cloisim_ros_bringup bringup_launch.py
# or
ros2 run cloisim_ros_bringup bringup
```

### Single Robot Mode

```bash
ros2 launch cloisim_ros_bringup bringup_single_launch.py
ros2 launch cloisim_ros_bringup bringup_launch.py single_mode:=True target_model:=cloi0
```

### Multi Robot Mode

```bash
ros2 launch cloisim_ros_bringup bringup_multi_launch.py
ros2 launch cloisim_ros_bringup bringup_launch.py single_mode:=False
```

### Target Specific Parts

Available parts types (case-sensitive): `MICOM`, `LASER`, `LIDAR`, `CAMERA`, `DEPTHCAMERA`, `MULTICAMERA`, `REALSENSE`, `GPS`, `ELEVATOR`, `WORLD`

```bash
ros2 launch cloisim_ros_bringup bringup_launch.py target_model:=cloi1 target_parts_type:=LASER
ros2 launch cloisim_ros_bringup bringup_launch.py target_model:=cloi1 target_parts_type:=LASER target_parts_name:=lidar
```

### Standalone Nodes

Each sensor package can also run independently:

```bash
ros2 run cloisim_ros_lidar standalone
ros2 run cloisim_ros_camera standalone
ros2 run cloisim_ros_micom standalone
```

### TF Publishing Control

```bash
ros2 launch cloisim_ros_bringup bringup_launch.py single_mode:=True micom.enable_tf:=False
```

## Testing

Tests use **Google Test** (`ament_cmake_gtest`) with a mock ZMQ bridge server located at `cloisim_ros_base/test/mock_bridge_server.hpp`.

### Run All Tests for a Package

```bash
colcon build --packages-select cloisim_ros_lidar --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select cloisim_ros_lidar --event-handlers console_direct+
colcon test-result --all
```

### Packages with Tests

| Package | Test File |
|---|---|
| `cloisim_ros_lidar` | `test/test_lidar_node.cpp` |
| `cloisim_ros_micom` | `test/test_micom_node.cpp` |
| `cloisim_ros_imu` | `test/test_imu_node.cpp` |
| `cloisim_ros_gps` | `test/test_gps_node.cpp` |
| `cloisim_ros_contact` | `test/test_contact_node.cpp` |

### Static Analysis

`ament_cppcheck` is enabled in `cloisim_ros_base` and runs automatically.

## Docker

### Build the Image

```bash
docker build -t cloisim_ros .
```

### Run

```bash
./launch.sh
# Equivalent to:
docker run -it --rm --net=host -e ROS_DOMAIN_ID=${ROS_DOMAIN_ID} cloisim_ros
```

The container uses `rmw_cyclonedds_cpp` as the default RMW implementation.

## DDS Configuration

- **CycloneDDS**: `cyclonedds_config.xml`
- **Fast-RTPS**: `fastrtps_shared_profile.xml`

## Architecture Overview

```
cloisim_ros_bringup          # Top-level orchestrator — discovers and spawns sensor nodes
  ├── cloisim_ros_bringup_param   # Parameter handling and simulator model discovery
  ├── cloisim_ros_websocket_service  # WebSocket client for port discovery
  └── sensor/actuator packages (lidar, camera, micom, imu, gps, ...)
        └── cloisim_ros_base       # Abstract base class (cloisim_ros::Base)
              ├── cloisim_ros_bridge_zmq  # ZMQ transport layer
              └── cloisim_ros_protobuf_msgs  # Protobuf message definitions
```

### Key Base Class

All sensor nodes inherit from `cloisim_ros::Base` (in `cloisim_ros_base`), which provides:

- ROS 2 node lifecycle (inherits `rclcpp::Node`)
- ZMQ bridge creation and management
- TF / TF static broadcasting
- Worker thread management for data/service bridges
- Protobuf ↔ ROS message conversion helpers

### Communication Flow

1. `cloisim_ros_websocket_service` queries the simulator's WebSocket API to discover available ZMQ ports
2. `cloisim_ros_bringup_param` parses the model/parts configuration
3. `cloisim_ros_bringup` spawns the appropriate sensor nodes
4. Each sensor node creates ZMQ bridges (REP for info, SUB for data) via `cloisim_ros_bridge_zmq`
5. Protobuf messages from CLOiSim are deserialized and published as ROS 2 messages

## Code Style

The project uses a `.clang-format` based on `ament_lint` / Google style with these customizations:

- Column limit: 100
- C++17 standard
- Brace wrapping after class, function, namespace, struct, enum
- Pointer alignment: middle (`int * ptr`)

### Compiler Flags

Set via `cmake/cloisim_ros_package.cmake`:

```
-Wall -Wextra -Wpedantic -Wdeprecated -fPIC -fstack-protector -O2
```

Default build type is `Release` if none is specified.
