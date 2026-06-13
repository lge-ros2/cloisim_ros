# Package Reference

Use this skill when you need a quick package-level map of the `cloisim_ros` workspace.

## Package Dependency Graph

```text
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

## Infrastructure Packages

### cloisim_ros_base
- Shared library.
- Key class: `cloisim_ros::Base`.
- Responsibilities: node lifecycle, ZMQ bridge management, TF broadcasting, worker threads.

### cloisim_ros_bridge_zmq
- Shared library.
- Key class: `zmq::Bridge`.
- Handles low-level ZMQ transport.

### cloisim_ros_protobuf_msgs
- Generated shared library.
- Contains simulator/ROS bridge protobuf messages.

### cloisim_ros_msgs
- ROS 2 interface package.
- Defines custom messages and services.

### cloisim_ros_websocket_service
- Shared library.
- Discovers simulator ZMQ ports over WebSocket.

### cloisim_ros_bringup_param
- Shared library.
- Parses simulator model and part configuration.

### cloisim_ros_bringup
- Executable orchestrator.
- Spawns sensor and actuator nodes.

## Sensor Packages

- `cloisim_ros_lidar`: publishes `sensor_msgs/msg/LaserScan`
- `cloisim_ros_camera`: camera, depth camera, segmentation camera bridge
- `cloisim_ros_multicamera`: multi-camera bridge
- `cloisim_ros_realsense`: RealSense bridge
- `cloisim_ros_imu`: publishes `sensor_msgs/msg/Imu`
- `cloisim_ros_gps`: publishes `sensor_msgs/msg/NavSatFix`
- `cloisim_ros_range`: sonar / IR range bridge
- `cloisim_ros_contact`: contact sensor bridge

## Actuator / Control Packages

- `cloisim_ros_micom`: motor driver, odometry, battery, IMU bridge
- `cloisim_ros_joint_control`: joint control interface

## World / Environment Packages

- `cloisim_ros_world`: world clock and utilities
- `cloisim_ros_ground_truth`: ground truth bridge
- `cloisim_ros_actor`: actor control bridge
- `cloisim_ros_elevator_system`: elevator simulation bridge
