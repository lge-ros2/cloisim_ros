# cloisim_ros_bringup

The `cloisim_ros_bringup` package is an bringup system for cloisim_ros.

## ROS2 Launch

```shell
ros2 launch cloisim_ros_bringup cloisim.launch.py
ros2 launch cloisim_ros_bringup bringup_launch.py
```

## ROS2 Run

```shell
ros2 run cloisim_ros_bringup bringup
```

## with parameters

### Turn off single mode (= multi robot mode)

Apply namespaceas each robot as a multi robot mode

#### ros2 run

```shell
ros2 run cloisim_ros_bringup bringup --ros-args -p single_mode:=False
ros2 run cloisim_ros_bringup bringup
```

#### ros2 launch

```shell
ros2 launch cloisim_ros_bringup bringup_launch.py
ros2 launch cloisim_ros_bringup bringup_launch.py single_mode:=False
ros2 launch cloisim_ros_bringup bringup_multi_launch.py
```

### Specify the target model or target parts

Available Parts(case sensitive): MICOM, LASER, LIDAR, CAMERA, DEPTHCAMERA, MULTICAMERA, REALSENSE, GPS, ELEVATOR, WORLD

Examples

```shell
ros2 launch cloisim_ros_bringup bringup_launch.py target_model:=cloi1
ros2 launch cloisim_ros_bringup bringup_launch.py target_parts_type:=LASER
ros2 launch cloisim_ros_bringup bringup_launch.py target_model:=cloi1 target_parts_type:=LASER
ros2 launch cloisim_ros_bringup bringup_launch.py target_model:=cloi1 target_parts_type:=LASER target_parts_name:=lidar
```

### Turn on single Mode

will NOT apply namespace for robot and the number of robot must BE single in world environment.

```shell
ros2 run cloisim_ros_bringup bringup --ros-args -p single_mode:=True
ros2 launch cloisim_ros_bringup bringup_launch.py single_mode:=True
ros2 launch cloisim_ros_bringup bringup_single_launch.py
```

#### Specific target model without namespace

Specify target model from simulation

```shell
ros2 launch cloisim_ros_bringup bringup_launch.py single_mode:=True target_model:=cloi0
ros2 launch cloisim_ros_bringup bringup_single_launch.py target_model:=cloi0
```

### Enable or disable TF/TF_Static publishing

Currently only support "micom" type in cloisim_ros_bringup.

```shell
ros2 run cloisim_ros_bringup bringup --ros-args -p single_mode:=True -p micom.enable_tf:=False
ros2 launch cloisim_ros_bringup bringup_launch.py single_mode:=True micom.enable_tf:=False
ros2 launch cloisim_ros_bringup bringup_single_launch.py micom.enable_tf:=False
```
