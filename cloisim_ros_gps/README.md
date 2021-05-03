# CLOiSim-ROS GPS

Currently noise model is not applied yet.

```shell
ros2 run cloisim_ros_gps cloisim_ros_gps
```

or

```shell
ros2 run cloisim_ros_gps cloisim_ros_gps --ros-args -p single_mode:=True -p target_model:=cloi1 -p target_parts_name:=gps
```

or

```shell
ros2 run cloisim_ros_gps cloisim_ros_gps --ros-args -p target_model:=cloi1 -p target_parts_name:=gps
```
