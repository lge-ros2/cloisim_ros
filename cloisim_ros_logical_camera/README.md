# cloisim_ros_logical_camera

ROS2 node for logical camera sensor in CLOiSim simulator.

Bridges `cloisim::msgs::LogicalCameraImage` protobuf to `vision_msgs::msg::Detection3DArray`.

## Published Topics

- `logical_camera` (`vision_msgs/msg/Detection3DArray`): Detected models with name and pose relative to the camera.

## Standalone Usage

```shell
ros2 run cloisim_ros_logical_camera standalone
```
