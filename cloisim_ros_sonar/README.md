# CLOiSim-ROS Sonar

support ros remapping, --ros-args -r /test:=test

Currently noise model is not applied yet.

```shell
ros2 run cloisim_ros_sonar standalone
```

## Single mode

```shell
ros2 run cloisim_ros_sonar standalone --ros-args -p single_mode:=True -p target_model:=cloi1 -p target_parts_name:=sonar
```

## Namespaced mode for multi-robot

```shell
ros2 run cloisim_ros_sonar standalone --ros-args -p target_model:=cloi1 -p target_parts_name:=sonar
```

## disable publishing TF(+TF static)

```shell
ros2 run cloisim_ros_sonar standalone --ros-args -p enable_tf:=False -p target_model:=cloi1 -p target_parts_name:=sonar
```
