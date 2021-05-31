# CLOiSim-ROS Realsense

support ros remapping, --ros-args -r /test:=test

```shell
ros2 run cloisim_ros_realsense standalone
```

or

```shell
ros2 run cloisim_ros_realsense standalone --ros-args -p single_mode:=True -p target_model:=cloi1 -p target_parts_name:=realsense
```

or

```shell
ros2 run cloisim_ros_realsense standalone --ros-args -p target_model:=cloi1 -p target_parts_name:=realsense
```
