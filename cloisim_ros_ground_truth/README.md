# CLOiSim-ROS GroundTruth Plugin

```shell
ros2 run cloisim_ros_ground_truth standalone
ros2 run cloisim_ros_ground_truth standalone --ros-args -p target_parts_name:=tracking
```

## How to configure

if you want to tracking dynamic props, set `enable` attribute in `<props>` element to `true`.

```xml
<plugin name="GroundTruthPlugin" filename="libGroundTruthPlugin.so">
  <publish_frequency>5</publish_frequency>
  <props enable="true">
    <prop class_id="10">box</prop>
    <prop class_id="11">cylinder</prop>
    <prop class_id="12">sphere</prop>
  </props>
  <list>
    <target tracking_id="1" class_id="101">L9S</target>
    <target tracking_id="11" class_id="100">Former1</target>
  </list>
</plugin>
```

### samples

You need to configure the ground truth list from [here](https://github.com/lge-ros2/sample_resources/blob/415a700c5280286a28690cf032cd0f4aa826a150/worlds/lg_seocho_with_actors.world#L301).


## How to monitor the message of topic

```shell
ros2 topic echo /ground_truth
```

### Examples of outputs

```shell
---
header:
  stamp:
    sec: 7
    nanosec: 259999837
  frame_id: ''
objects:
- header:
    stamp:
      sec: 7
      nanosec: 259999837
    frame_id: ''
  tracking_id: 0
  class_id: 101
  position:
    x: -43.8023567199707
    y: -28.77000617980957
    z: -6.612044334411621
  velocity:
    x: 0.0
    y: -0.0
    z: 2.6813422664417885e-05
  size:
    x: 0.8143882751464844
    y: -0.5866775512695312
    z: 1.3645973205566406
  footprint:
    points:
    - x: -0.031848907470703125
      y: 0.2933387756347656
      z: 0.0
    - x: -0.030796051025390625
      y: 0.2933387756347656
      z: 0.0
    - x: -0.030796051025390625
      y: 0.2933387756347656
      z: 0.0
    - '...'
- header:
    stamp:
      sec: 7
      nanosec: 259999837
    frame_id: ''
  tracking_id: 1
  class_id: 100
  position:
    x: -35.91999816894531
    y: -28.219999313354492
    z: -6.590000152587891
  velocity:
    x: 0.0
    y: -0.0
    z: 0.0
  size:
    x: 0.3796844482421875
    y: -0.3796844482421875
    z: 1.5396394729614258
  footprint:
    points:
    - x: 0.0
      y: -0.18984131515026093
      z: 0.0
    - x: 0.032117798924446106
      y: -0.1871047168970108
      z: 0.0
    - x: 0.06330962479114532
      y: -0.17897379398345947
      z: 0.0
    - '...'
- header:
    stamp:
      sec: 7
      nanosec: 259999837
    frame_id: ''
  tracking_id: 2
  class_id: 100
  position:
    x: -41.80641555786133
    y: -20.955291748046875
    z: -6.590000152587891
  velocity:
    x: -0.18361832201480865
    y: -0.09813712537288666
    z: 0.0
  size:
    x: 0.3796844482421875
    y: -0.3796844482421875
    z: 1.2043333053588867
  footprint:
    points:
    - x: 0.0
      y: -0.1898414045572281
      z: 0.0
    - x: 0.0321178138256073
      y: -0.18710480630397797
      z: 0.0
    - x: 0.06330965459346771
      y: -0.17897386848926544
      z: 0.0
    - '...'
---
```
