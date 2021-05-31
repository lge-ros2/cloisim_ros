# CLOiSim-ROS Elevator System

elevator system msgs is required.

You need to download interfaces.

Please, refer to [here](https://github.com/lge-ros2/cloi_common_interfaces/tree/foxy)

```shell
ros2 run cloisim_ros_elevatorsystem standalone
```

or

```shell
ros2 run cloisim_ros_elevatorsystem standalone --ros-args -p target_model:=SeochoTower -p target_parts_name:=elevator_system
```

## Example

```shell
ros2 service call /elevator_system/request_door_open elevator_system_msgs/srv/RequestDoor "{elevator_index: 0}"
```
