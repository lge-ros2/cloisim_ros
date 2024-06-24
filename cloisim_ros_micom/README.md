# CLOiSim-ROS Micom

Control Wheel and update odometry info through micom.

support ros remapping, --ros-args -r /test:=test

```shell
ros2 run cloisim_ros_micom standalone
```

## Single mode

```shell
ros2 run cloisim_ros_micom standalone --ros-args -p single_mode:=True -p target_model:=cloi1 -p target_parts_name:=RobotControl
```

## Namespaced mode for multi-robot

```shell
ros2 run cloisim_ros_micom standalone --ros-args -p target_model:=cloi1 -p target_parts_name:=RobotControl
```

## disable publishing TF(+TF static)

```shell
ros2 run cloisim_ros_micom standalone --ros-args -p enable_tf:=False -p target_model:=cloi1 -p target_parts_name:=RobotControl
```
