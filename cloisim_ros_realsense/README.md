# CLOiSim-ROS Realsense

support ros remapping, --ros-args -r /test:=test

```shell
ros2 run cloisim_ros_realsense standalone
```

## Single mode

```shell
ros2 run cloisim_ros_realsense standalone --ros-args -p single_mode:=True -p target_model:=cloi1 -p target_parts_name:=realsense
```

## Namespaced mode for multi-robot

```shell
ros2 run cloisim_ros_realsense standalone --ros-args -p target_model:=cloi1 -p target_parts_name:=realsense
```

## disable publishing TF(+TF static)

```shell
ros2 run cloisim_ros_realsense standalone --ros-args -p enable_tf:=False -p target_model:=cloi1 -p target_parts_name:=realsense
```
