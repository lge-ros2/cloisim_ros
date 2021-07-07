# CLOiSim-ROS Actor

```shell
ros2 run cloisim_ros_actor standalone
```

## Example

match the 'target_name' in actor's name in world file

please refer to [here](https://github.com/lge-ros2/sample_resources/blob/415a700c5280286a28690cf032cd0f4aa826a150/worlds/lg_seocho_with_actors.world#L111)

```shell
ros2 service call /ActorControlPlugin/move_actor cloisim_ros_msgs/srv/MoveActor "{target_name: 'actor3', destination: {x: -40.6301, y: -29.4183, z: -6.6}}"
```

you can get to actors position through this [ground_truth node](https://github.com/lge-ros2/cloisim_ros/tree/main/cloisim_ros_ground_truth)
