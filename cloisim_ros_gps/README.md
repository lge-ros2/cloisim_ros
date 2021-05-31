# CLOiSim-ROS GPS

support ros remapping, --ros-args -r /test:=test

Currently noise model is not applied yet.

```shell
ros2 run cloisim_ros_gps standalone
```

or

```shell
ros2 run cloisim_ros_gps standalone --ros-args -p single_mode:=True -p target_model:=cloi1 -p target_parts_name:=gps
```

or

```shell
ros2 run cloisim_ros_gps standalone --ros-args -p target_model:=cloi1 -p target_parts_name:=gps
```
