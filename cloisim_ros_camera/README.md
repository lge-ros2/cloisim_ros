# CLOiSim-ROS Camera

support ros remapping, --ros-args -r /test:=test

```shell
ros2 run cloisim_ros_camera cloisim_ros_camera
```

or

```shell
ros2 run cloisim_ros_camera cloisim_ros_camera --ros-args -p single_mode:=True -p target_model:=cloi1 -p target_parts_name:=camera
```

or

```shell
ros2 run cloisim_ros_camera cloisim_ros_camera --ros-args -p target_model:=cloi1 -p target_parts_name:=camera
```
