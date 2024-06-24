# CLOiSim-ROS GPS

support ros remapping, --ros-args -r /test:=test

Currently noise model is not applied yet.

```shell
ros2 run cloisim_ros_gps standalone
```

## Single mode

```shell
ros2 run cloisim_ros_gps standalone --ros-args -p single_mode:=True -p target_model:=cloi1 -p target_parts_name:=gps
```

## Namespaced mode for multi-robot

```shell
ros2 run cloisim_ros_gps standalone --ros-args -p target_model:=cloi1 -p target_parts_name:=gps
```

## disable publishing TF(+TF static)

```shell
ros2 run cloisim_ros_gps standalone --ros-args -p enable_tf:=False -p target_model:=cloi1 -p target_parts_name:=gps
```
