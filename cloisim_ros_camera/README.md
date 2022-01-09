# CLOiSim-ROS Camera

## options

support ros remapping, --ros-args -r /test:=test

## how to bringup

you need to edit SDF model file and add plugin in `<sensor>` element. please refer to [here](https://github.com/lge-ros2/sample_resources/blob/2077dfaaf7248d0a930f814db23b1c17f3e79b8c/models/robot_camera/model.sdf#L70) as en exmaple.

This filename in `<plugin>` element enables to connect cloisim_ros_camera.

```xml
<sensor name='camera' type='camera'>
  ....
  ...
  ..
  .
  <plugin name='CameraPlugin' filename='libCameraPlugin.so'>
    <ros2>
      <topic_name>color</topic_name>
      <frame_id>camera_link</frame_id>
    </ros2>
  </plugin>
</sensor>
```


## how to run
```shell
ros2 run cloisim_ros_camera standalone
```

or

```shell
ros2 run cloisim_ros_camera standalone --ros-args -p single_mode:=True -p target_model:=cloi1 -p target_parts_name:=camera
```

or

```shell
ros2 run cloisim_ros_camera standalone --ros-args -p target_model:=cloi1 -p target_parts_name:=camera
```
