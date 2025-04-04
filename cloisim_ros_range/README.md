# CLOiSim-ROS Range(Sonar/IR)

support ros remapping, --ros-args -r /test:=test

Currently noise model is not applied yet.

```shell
ros2 run cloisim_ros_range sonar
ros2 run cloisim_ros_range ir
```

## Single mode

```shell
ros2 run cloisim_ros_range sonar --ros-args -p single_mode:=True -p target_model:=cloi1 -p target_parts_name:=sonar

ros2 run cloisim_ros_range ir --ros-args -p single_mode:=True -p target_model:=cloi1 -p target_parts_name:=ir
```

## Namespaced mode for multi-robot

```shell
ros2 run cloisim_ros_range sonar --ros-args -p target_model:=cloi1 -p target_parts_name:=sonar

ros2 run cloisim_ros_range ir --ros-args -p target_model:=cloi1 -p target_parts_name:=ir
```

## disable publishing TF(+TF static)

```shell
ros2 run cloisim_ros_range sonar --ros-args -p enable_tf:=False -p target_model:=cloi1 -p target_parts_name:=sonar

ros2 run cloisim_ros_range ir --ros-args -p enable_tf:=False -p target_model:=cloi1 -p target_parts_name:=ir
```
