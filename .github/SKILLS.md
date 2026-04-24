# Package Reference (Skills)

Complete reference for all packages in the cloisim_ros workspace (v4.10.1).

## Package Dependency Graph

```
cloisim_ros_bringup
├── cloisim_ros_bringup_param
│   ├── cloisim_ros_websocket_service (WebSocket, asio)
│   └── rclcpp, jsoncpp
├── cloisim_ros_micom
├── cloisim_ros_lidar
├── cloisim_ros_camera
├── cloisim_ros_multicamera
├── cloisim_ros_realsense
├── cloisim_ros_gps
├── cloisim_ros_imu
├── cloisim_ros_range
├── cloisim_ros_contact
├── cloisim_ros_elevator_system
├── cloisim_ros_ground_truth
├── cloisim_ros_world
├── cloisim_ros_actor
└── cloisim_ros_joint_control
      └── cloisim_ros_base
            ├── cloisim_ros_bridge_zmq (libzmq)
            ├── cloisim_ros_protobuf_msgs (protobuf)
            ├── rclcpp, tf2_ros
            ├── geometry_msgs
            └── ros_gz_interfaces
```

---

## Infrastructure Packages

### cloisim_ros_base

Base class library that all sensor/actuator nodes inherit from.

| Item | Detail |
|---|---|
| **Type** | Shared library |
| **Key Class** | `cloisim_ros::Base` (inherits `rclcpp::Node`) |
| **Headers** | `base.hpp`, `helper.hpp`, `param_helper.hpp`, `camera_helper.hpp` |
| **Responsibilities** | Node lifecycle, ZMQ bridge management, TF broadcasting, worker threads |
| **Dependencies** | `rclcpp`, `tf2_ros`, `cloisim_ros_bridge_zmq`, `cloisim_ros_protobuf_msgs`, `geometry_msgs`, `ros_gz_interfaces` |
| **Tests** | Provides `test/mock_bridge_server.hpp` used by all other package tests |

### cloisim_ros_bridge_zmq

Low-level ZMQ transport layer for simulator communication.

| Item | Detail |
|---|---|
| **Type** | Shared library |
| **Key Class** | `zmq::Bridge` |
| **Headers** | `bridge.hpp`, `log.h`, `term_color.h` |
| **Dependencies** | `libzmq3-dev` |

### cloisim_ros_protobuf_msgs

Protobuf message definitions shared between CLOiSim and ROS nodes.

| Item | Detail |
|---|---|
| **Type** | Shared library (generated) |
| **Proto files** | 200+ `.proto` files in `msgs/` |
| **Key messages** | `laserscan`, `image`, `imu`, `gps`, `micom`, `pose`, `param`, `contacts`, `joint_state`, `perception` |
| **Dependencies** | `protobuf-dev` |

### cloisim_ros_msgs

ROS 2 interface definitions (IDL).

| Item | Detail |
|---|---|
| **Type** | ROS 2 interface package |
| **Messages** | `ContactsArray.msg` |
| **Services** | `MoveActor.srv` |
| **Dependencies** | `std_msgs`, `geometry_msgs`, `ros_gz_interfaces` |

### cloisim_ros_websocket_service

WebSocket client for discovering ZMQ ports from the simulator's service API.

| Item | Detail |
|---|---|
| **Type** | Shared library |
| **Dependencies** | `libwebsocketpp-dev`, `asio` |

### cloisim_ros_bringup_param

Parameter handling and simulator model/parts discovery.

| Item | Detail |
|---|---|
| **Type** | Shared library |
| **Dependencies** | `rclcpp`, `cloisim_ros_websocket_service`, `libjsoncpp-dev` |

### cloisim_ros_bringup

Top-level orchestrator that auto-discovers and spawns all sensor nodes.

| Item | Detail |
|---|---|
| **Type** | Executable (`bringup`) |
| **Entry point** | `src/main.cpp` → `src/bringup.cpp` |
| **Launch files** | `bringup_launch.py`, `bringup_single_launch.py`, `bringup_multi_launch.py`, `cloisim_launch.py`, `bringup.launch.py`, `cloisim.launch.py` |
| **Parameters** | `single_mode`, `target_model`, `target_parts_type`, `target_parts_name`, `micom.enable_tf` |

---

## Sensor Packages

### cloisim_ros_lidar

| Item | Detail |
|---|---|
| **Description** | Virtual LiDAR sensor |
| **Publishes** | `sensor_msgs/msg/LaserScan` |
| **Protobuf input** | `cloisim::msgs::LaserScan` |
| **Dependencies** | `cloisim_ros_base`, `sensor_msgs`, `geometry_msgs` |
| **Test** | `test/test_lidar_node.cpp` |

### cloisim_ros_camera

| Item | Detail |
|---|---|
| **Description** | Virtual camera / depth camera / segmentation camera |
| **Publishes** | `sensor_msgs/msg/Image`, `sensor_msgs/msg/CameraInfo`, `vision_msgs` |
| **Dependencies** | `cloisim_ros_base`, `sensor_msgs`, `camera_info_manager`, `image_transport`, `vision_msgs` |

### cloisim_ros_multicamera

| Item | Detail |
|---|---|
| **Description** | Virtual multi-camera |
| **Dependencies** | `cloisim_ros_base`, `sensor_msgs`, `camera_info_manager`, `image_transport` |

### cloisim_ros_realsense

| Item | Detail |
|---|---|
| **Description** | Virtual RealSense camera |
| **Dependencies** | `cloisim_ros_base`, `sensor_msgs`, `camera_info_manager`, `image_transport` |

### cloisim_ros_imu

| Item | Detail |
|---|---|
| **Description** | Virtual IMU sensor |
| **Publishes** | `sensor_msgs/msg/Imu` |
| **Dependencies** | `cloisim_ros_base`, `sensor_msgs` |
| **Test** | `test/test_imu_node.cpp` |

### cloisim_ros_gps

| Item | Detail |
|---|---|
| **Description** | Virtual GPS sensor |
| **Publishes** | `sensor_msgs/msg/NavSatFix` |
| **Dependencies** | `cloisim_ros_base`, `sensor_msgs`, `geometry_msgs` |
| **Test** | `test/test_gps_node.cpp` |

### cloisim_ros_range

| Item | Detail |
|---|---|
| **Description** | Virtual range sensor (sonar / IR) |
| **Publishes** | `sensor_msgs/msg/Range` |
| **Dependencies** | `cloisim_ros_base`, `sensor_msgs`, `geometry_msgs` |

### cloisim_ros_contact

| Item | Detail |
|---|---|
| **Description** | Virtual contact sensor |
| **Dependencies** | `cloisim_ros_base`, `sensor_msgs`, `geometry_msgs`, `ros_gz_interfaces` |
| **Test** | `test/test_contact_node.cpp` |

---

## Actuator / Control Packages

### cloisim_ros_micom

| Item | Detail |
|---|---|
| **Description** | Microcontroller interface (motor driver, odometry, battery) |
| **Publishes** | `nav_msgs/msg/Odometry`, `sensor_msgs/msg/BatteryState`, `sensor_msgs/msg/Imu` |
| **Subscribes** | `geometry_msgs/msg/Twist` |
| **Config** | `joy.config.yaml` |
| **Dependencies** | `cloisim_ros_base`, `cloisim_ros_msgs`, `sensor_msgs`, `geometry_msgs`, `nav_msgs`, `std_srvs` |
| **Test** | `test/test_micom_node.cpp` |

### cloisim_ros_joint_control

| Item | Detail |
|---|---|
| **Description** | Joint control interface |
| **Dependencies** | `cloisim_ros_base`, `std_srvs`, `std_msgs`, `sensor_msgs`, `control_msgs` |

---

## World / Environment Packages

### cloisim_ros_world

| Item | Detail |
|---|---|
| **Description** | World clock and utilities |
| **Dependencies** | `cloisim_ros_base`, `rosgraph_msgs`, `std_srvs` |

### cloisim_ros_ground_truth

| Item | Detail |
|---|---|
| **Description** | Ground truth data retrieval from the simulator |
| **Dependencies** | `cloisim_ros_base`, `perception_msgs` |

### cloisim_ros_actor

| Item | Detail |
|---|---|
| **Description** | Actor (animated character) control |
| **Services** | `cloisim_ros_msgs/srv/MoveActor` |
| **Dependencies** | `cloisim_ros_base`, `cloisim_ros_msgs` |

### cloisim_ros_elevator_system

| Item | Detail |
|---|---|
| **Description** | Elevator system simulation |
| **Services** | `CallElevator.srv`, `GetElevatorInformation.srv`, `RequestDoor.srv`, `SelectElevatorFloor.srv`, etc. |
| **Messages** | `ElevatorStatus.msg` |
| **Dependencies** | `cloisim_ros_base`, `elevator_system_msgs` |

---

## External Submodule

### cloi_common_interfaces

| Item | Detail |
|---|---|
| **Source** | https://github.com/lge-ros2/cloi_common_interfaces.git (branch `4.0.0`) |
| **Contains** | `elevator_system_msgs`, `perception_msgs` |
| **elevator_system_msgs** | `ElevatorStatus.msg`, `CallElevator.srv`, `GetElevatorInformation.srv`, `RequestDoor.srv`, `ReturnBool.srv`, `SelectElevatorFloor.srv`, `StringValue.srv` |
