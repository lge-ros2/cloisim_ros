# CLOiSim-ROS Multi-Camera

support ros remapping, --ros-args -r /test:=test

```shell
ros2 run cloisim_ros_multicamera standalone
```

or

```shell
ros2 run cloisim_ros_multicamera standalone --ros-args -p single_mode:=True -p target_model:=cloi1 -p target_parts_name:=multicamera
```

or

```shell
ros2 run cloisim_ros_multicamera standalone --ros-args -p target_model:=cloi1 -p target_parts_name:=multicamera
```
