# CLOiSim-ROS Segmentation Camera

support ros remapping, --ros-args -r /test:=test

```shell
ros2 run cloisim_ros_segmentationcamera standalone
```

or

```shell
ros2 run cloisim_ros_segmentationcamera standalone --ros-args -p single_mode:=True -p target_model:=cloi -p target_parts_name:=segmentationcamera
```

or

```shell
ros2 run cloisim_ros_segmentationcamera standalone --ros-args -p target_model:=cloi -p target_parts_name:=segmentationcamera
```
