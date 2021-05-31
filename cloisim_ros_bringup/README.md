# cloisim_ros_bringup

The `cloisim_ros_bringup` package is an bringup system for cloisim_ros.

## ROS2 Launch

```shell
ros2 launch cloisim_ros_bringup cloisim.launch.py
ros2 launch cloisim_ros_bringup bringup.launch.py
```

## ROS2 Run

```shell
ros2 run cloisim_ros_bringup bringup
```

## with parameters

### Turn off single mode (= multi robot mode)

Apply namespaceas each robot as a multi robot mode

```shell
ros2 run cloisim_ros_bringup bringup --ros-args -p single_mode:=False
ros2 run cloisim_ros_bringup bringup
```

or

```shell
ros2 launch cloisim_ros_bringup bringup.launch.py
ros2 launch cloisim_ros_bringup bringup.launch.py single_mode:=False
```

### Specify the target model or target parts

Available Parts(case sensitive): MICOM, LASER, LIDAR, CAMERA, DEPTHCAMERA, MULTICAMERA, REALSENSE, GPS, ELEVATOR, WORLD

Examples

```shell
ros2 launch cloisim_ros_bringup bringup.launch.py target_model:=cloi1
ros2 launch cloisim_ros_bringup bringup.launch.py target_parts_type:=LASER
ros2 launch cloisim_ros_bringup bringup.launch.py target_model:=cloi1 target_parts_type:=LASER
ros2 launch cloisim_ros_bringup bringup.launch.py target_model:=cloi1 target_parts_type:=LASER target_parts_name:=lidar
```

### Turn on single Mode

will NOT apply namespace for robot and the number of robot must BE single in world environment.

```shell
ros2 run cloisim_ros_bringup bringup --ros-args -p single_mode:=True
```

or

```shell
ros2 launch cloisim_ros_bringup bringup.launch.py single_mode:=True
```
