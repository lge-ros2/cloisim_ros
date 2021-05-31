# CLOiSim-ROS Actor

```shell
ros2 run cloisim_ros_actor standalone
```

or

```shell
ros2 run cloisim_ros_actor standalone --ros-args -p target_model:=SeochoTower -p target_parts_name:=elevator_system
```

## Example

```shell
ros2 service call /elevator_system/request_door_open elevator_system_msgs/srv/RequestDoor "{elevator_index: 0}"
```
