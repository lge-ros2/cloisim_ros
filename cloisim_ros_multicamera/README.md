# CLOiSim-ROS Multi-Camera

support ros remapping, --ros-args -r /test:=test

```shell
ros2 run cloisim_ros_multicamera cloisim_ros_multicamera
```

or

```shell
ros2 run cloisim_ros_multicamera cloisim_ros_multicamera --ros-args -p single_mode:=True -p target_model:=cloi1 -p target_parts_name:=multicamera
```

or

```shell
ros2 run cloisim_ros_multicamera cloisim_ros_multicamera --ros-args -p target_model:=cloi1 -p target_parts_name:=multicamera
```
