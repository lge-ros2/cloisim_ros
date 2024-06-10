# cloisim_ros (humble version)

ROS2 simulation device packages to connect CLOiSim(the unity3D based multi-robot simulator).

## Download CLOiSim Simulator

- Latest version: [link](https://github.com/lge-ros2/cloisim/releases/latest)
- All Releases: [link](https://github.com/lge-ros2/cloisim/releases)

## Install ROS2 humble

  follow the guideline on below link.

  <https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html>

## Prerequisite

```shell
sudo apt update
sudo apt install -y python3-rosdep
sudo rosdep init
rosdep update
rosdep install -y -r -q --from-paths src --ignore-src --rosdistro jazzy
```

## Build

Set up ROS2 environment first

```shell
source /opt/ros2/jazzy/setup.bash
colcon build --symlink-install --packages-up-to cloisim_ros_bringup
```

## Usage

Set environment variable, if the server is not localhost

```shell
export CLOISIM_BRIDGE_IP='xxx.xxx.xxx.xxx'
export CLOISIM_SERVICE_PORT=8080
```

check here [details](https://github.com/lge-ros2/cloisim_ros/tree/jazzy/cloisim_ros_bringup) for bring-up guide

### Run cloisim_ros (robot + world)

#### Shared Memory DDS with FastRTPS

It's experimental feature for cloisim_ros. Use absolute path in FASTRTPS_DEFAULT_PROFILES_FILE environment variable.

For example,

```shell
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/cloi/src/cloisim_ros/fastrtps_shared_profile.xml
```

#### Turn off single mode(=multi robot mode)

Apply namespaceas each robot as a multi robot mode.

**Strongly recommend to use this method.**

```shell
ros2 launch cloisim_ros_bringup bringup_launch.py
```

or

```shell
ros2 launch cloisim_ros_bringup bringup_launch.py single_mode:=False
```

#### Turn on single Mode

It shall NOT be applied namespace for robot and the number of robot must BE single in world environment.

```shell
ros2 launch cloisim_ros_bringup bringup_launch.py single_mode:=True
```

## Running CLOiSim

It provides a script to run CLOiSim easily.

```shell
ros2 launch cloisim_ros_bringup cloisim_launch.py sim_path:=/opt/CLOiSim/CLOiSim-2.2.0 world:=lg_seocho.world
```

## Using Docker

Run below command after clone this repository(this branch).

### Build Docker image

```shell
git clone https://github.com/lge-ros2/cloisim_ros.git -b jazzy
cd cloisim_ros
docker build -t cloisim_ros .
```

### Running container

You can add possible parameters as described above. ex) target_model, target_parts_type or target_parts_name

#### multi robot mode

```shell
docker run -it --rm --net=host -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID cloisim_ros
```

#### single robot mode

```shell
docker run -it --rm --net=host -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID cloisim_ros single_mode:=True
```

#### multi robot mode with default network driver(bridge)

```shell
docker run -it --rm -e CLOISIM_BRIDGE_IP='192.168.0.125' -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID cloisim_ros
```

#### single robot mode with default network driver(bridge)

```shell
docker run -it --rm -e CLOISIM_BRIDGE_IP='192.168.0.125' -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID cloisim_ros single_mode:=True
```

You can set bridge ip using below command.

```shell
export CLOISIM_BRIDGE_IP=$(ip addr show dev docker0 | grep "inet" | awk 'NR==1{print $2}' | cut -d'/' -f 1)
echo $CLOISIM_BRIDGE_IP
docker run -it --rm -e CLOISIM_BRIDGE_IP=$CLOISIM_BRIDGE_IP -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID cloisim_ros
```

## Version info

- Please refer to each branch for ROS2 distro-version you want
  - [jazzy](https://github.com/lge-ros2/cloisim_ros/tree/jazzy)
  - [humble](https://github.com/lge-ros2/cloisim_ros/tree/humble)
  - [foxy](https://github.com/lge-ros2/cloisim_ros/tree/foxy)
  - [dashing](https://github.com/lge-ros2/sim_device/tree/dashing)
