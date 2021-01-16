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
```

check here [details](https://github.com/lge-ros2/cloisim_ros/tree/foxy/cloisim_ros_bringup)

### launch cloisim_ros for robot

```shell
ros2 launch cloisim_ros_bringup robot.launch.py robot_name:=cloi
```

### launch elevator system

```shell
ros2 launch cloisim_ros_bringup elevatorsystem.launch.py
```

### launch factory (elevator system and world)

```shell
ros2 launch cloisim_ros_bringup factory.launch.py
```

### How to run cloisim_ros with CLOiSim together

#### only simulator

```shell
ros2 launch cloisim_ros_bringup cloisim.launch.py sim_path:=/opt/CLOiSim/CLOiSim-1.10.0 world:=lg_seocho.world
```

#### simulator + cloisim_ros  packge(clock topic)

```shell
ros2 launch cloisim_ros_bringup cloisim_and_factory.launch.py sim_path:=/opt/CLOiSim/CLOiSim-1.10.0 world:=lg_seocho.world
```

## Using Docker

Run below command after clone this repository(this branch).

### Build Docker image

```shell
git clone https://github.com/lge-ros2/cloisim_ros.git -b foxy
cd cloisim_ros
docker build -t cloisim_ros .
```

### Running container with laucnher

[here](https://github.com/lge-ros2/cloisim_ros/tree/foxy/cloisim_ros_bringup/launch) to check launchers

```shell
docker run -it --rm --net=host cloisim_ros {launch script} {arguments}
```

#### examples

```shell
docker run -it --rm --net=host cloisim_ros robot.launch.py robot_name:=cloi

docker run -it --rm --net=host cloisim_ros factory.launch.py
```

it requires to mount volume(-v option) for sim_path and resource to execute a CLOiSim.

refer to [here](https://github.com/lge-ros2/cloisim/tree/master/Docker)

```shell
docker run -it --rm --net=host cloisim_ros closim_and_factory.launch.py sim_path:=/opt/CLOiSim-1.10.0 world:=lg_seocho.world
```

## Version info

- Please refer to each branch for ROS2 distro-version you want
  - [dashing](https://github.com/lge-ros2/sim_device/tree/dashing)
  - [foxy](https://github.com/lge-ros2/cloisim_ros/tree/foxy)
