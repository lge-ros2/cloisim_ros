# CLOiSim-ROS JoinControl

Control Wheel and update odometry info through joint control.

support ros remapping, --ros-args -r /test:=test

```shell
ros2 run cloisim_ros_joint_control standalone
```

or

```shell
ros2 run cloisim_ros_joint_control standalone --ros-args -p single_mode:=True -p target_model:=cloi1 -p target_parts_name:=joint_control
```

or

```shell
ros2 run cloisim_ros_joint_control standalone --ros-args -p target_model:=cloi1 -p target_parts_name:=joint_control
```
