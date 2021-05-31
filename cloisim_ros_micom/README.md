# CLOiSim-ROS Micom

Control Wheel and update odometry info through micom.

support ros remapping, --ros-args -r /test:=test

```shell
ros2 run cloisim_ros_micom standalone
```

or

```shell
ros2 run cloisim_ros_micom standalone --ros-args -p single_mode:=True -p target_model:=cloi1 -p target_parts_name:=micom
```

or

```shell
ros2 run cloisim_ros_micom standalone --ros-args -p target_model:=cloi1 -p target_parts_name:=micom
```
