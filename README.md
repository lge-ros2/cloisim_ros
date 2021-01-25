# cloisim_ros (foxy version)

ROS2 simulation device packages to connect CLOiSim(the unity3D based multi-robot simulator).

## Prerequisite

- Download CLOiSim Simulator
  - CLOiSim: Latest [link](https://github.com/lge-ros2/cloisim/releases/latest), All Releases [link](https://github.com/lge-ros2/cloisim/releases)

```shell
rosdep update
rosdep install -y -r -q --from-paths src --ignore-src --rosdistro foxy
```

## Build

Set up ROS2 environment first

```shell
source /opt/ros2/foxy/setup.bash
colcon build --symlink-install --packages-up-to cloisim_ros_bringup
```

## Usage

Set environment variable, if the server is not localhost

```shell
export CLOISIM_BRIDGE_IP='xxx.xxx.xxx.xxx'
export CLOISIM_SERVICE_PORT=8080
```

check here [details](https://github.com/lge-ros2/cloisim_ros/tree/foxy/cloisim_ros_bringup)

### Run all cloisim_ros (robot + factory)

strongly recommend to use this method.

#### Turn on single Mode

will NOT apply namespace for robot and the number of robot must BE single in world environment.

```shell
ros2 run cloisim_ros_bringup cloisim_ros_bringup --ros-args -p singlemode:=True
```

#### Turn off single mode

apply namespace for each robot)

```shell
ros2 run cloisim_ros_bringup cloisim_ros_bringup --ros-args -p singlemode:=False

ros2 run cloisim_ros_bringup cloisim_ros_bringup
```

### launch cloisim_ros for robot

will be deprecated.

```shell
ros2 launch cloisim_ros_bringup robot.launch.py robot_name:=cloi
```

### launch factory (elevator system and world)

will be deprecated.

```shell
ros2 launch cloisim_ros_bringup factory.launch.py
```

### How to run cloisim_ros with CLOiSim together

#### only simulator

```shell
ros2 launch cloisim_ros_bringup cloisim.launch.py sim_path:=/opt/CLOiSim/CLOiSim-1.10.0 world:=lg_seocho.world
```

#### simulator + cloisim_ros package(clock topic)

```shell
ros2 launch cloisim_ros_bringup cloisim_and_factory.launch.py sim_path:=/opt/CLOiSim/CLOiSim-1.10.0 world:=lg_seocho.world
```

## Using Docker

Run below command after clone this repository(this branch). Only support 'ros2 run' 

### Build Docker image

```shell
git clone https://github.com/lge-ros2/cloisim_ros.git -b foxy
cd cloisim_ros
docker build -t cloisim_ros .
```

### Running container

```shell
docker run -it --rm --net=host -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID cloisim_ros 

docker run -it --rm --net=host -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID cloisim_ros --ros-args -p singlemode:=False
```

or

```shell
docker run -it --rm --net=host -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID cloisim_ros --ros-args -p singlemode:=True
```

## Version info

- Please refer to each branch for ROS2 distro-version you want
  - [dashing](https://github.com/lge-ros2/sim_device/tree/dashing)
  - [foxy](https://github.com/lge-ros2/cloisim_ros/tree/foxy)
