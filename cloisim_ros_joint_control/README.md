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


## Example Usage for Joint

### State

```shell
ros2 topic echo /franka/joint_states
```

### Command

#### Displacement

```shell
ros2 topic pub -1 /FnB/joint_command control_msgs/msg/JointJog '{joint_names: ["link_Display_JOINT"], displacements: [-1.5708]}'
```

#### Velocity

```shell
ros2 topic pub -1 /FnB/joint_command control_msgs/msg/JointJog '{joint_names: ["link_Display_JOINT"], velocities: [2.0]}'
```

#### Displacement and Velocity


```shell
ros2 topic pub -1 /FnB/joint_command control_msgs/msg/JointJog '{joint_names: ["link_Display_JOINT"], displacements: [-1.5708], velocities: [2.0]}'
```
