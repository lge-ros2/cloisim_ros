# CLOiSim-ROS IMU

support ros remapping, --ros-args -r /test:=test

Currently noise model is not applied yet.

```shell
ros2 run cloisim_ros_imu standalone
```

## Single mode

```shell
ros2 run cloisim_ros_imu standalone --ros-args -p single_mode:=True -p target_model:=cloi1 -p target_parts_name:=imu
```

## Namespaced mode for multi-robot

```shell
ros2 run cloisim_ros_imu standalone --ros-args -p target_model:=cloi1 -p target_parts_name:=imu
```

## disable publishing TF(+TF static)

```shell
ros2 run cloisim_ros_imu standalone --ros-args -p enable_tf:=False -p target_model:=cloi1 -p target_parts_name:=imu
```

