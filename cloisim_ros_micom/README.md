# CLOiSim-ROS Micom

Control Wheel and update odometry info through micom.

support ros remapping, --ros-args -r /test:=test

```shell
ros2 run cloisim_ros_micom standalone
```

## Single mode

```shell
ros2 run cloisim_ros_micom standalone --ros-args -p single_mode:=True -p target_model:=cloi1 -p target_parts_name:=RobotControl
```

## Namespaced mode for multi-robot

```shell
ros2 run cloisim_ros_micom standalone --ros-args -p target_model:=cloi1 -p target_parts_name:=RobotControl
```

## disable publishing TF(+TF static)

```shell
ros2 run cloisim_ros_micom standalone --ros-args -p enable_tf:=False -p target_model:=cloi1 -p target_parts_name:=RobotControl
```

## Lawn mowing control

### enable or disable mowing

#### enable mowing

```shell
ros2 topic pub -1 /mowing/blade/rev_speed std_msgs/msg/UInt16 "{data: 1000}"
```

#### disable mowing

```shell
ros2 topic pub -1 /mowing/blade/rev_speed std_msgs/msg/UInt16 "{data: 0}"
```

### control height

for example, adjust height of blade.

```shell
ros2 topic pub -1 /mowing/blade/height std_msgs/msg/Float32 "{data: 0.02}"
ros2 topic pub -1 /mowing/blade/height std_msgs/msg/Float32 "{data: 0.09}"
```
