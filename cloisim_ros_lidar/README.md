# CLOiSim-ROS Lidar

support ros remapping, --ros-args -r /test:=test

```shell
ros2 run cloisim_ros_lidar standalone
```

or

```shell
ros2 run cloisim_ros_lidar standalone --ros-args -p single_mode:=True -p target_model:=cloi1 -p target_parts_name:=lidar
```

or

```shell
ros2 run cloisim_ros_lidar standalone --ros-args -p target_model:=cloi1 -p target_parts_name:=lidar
```
