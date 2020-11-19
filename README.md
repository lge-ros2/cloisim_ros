# sim_device (foxy version)

ROS2 simulation device packages to connect the unity3D based multi-robot simulator(latest version).

## Prerequisite

- Download Simulator release
  - CLOiSim: Unity multi-robot simulator [releases](https://github.com/lge-ros2/multi-robot-simulator/releases)
    - Simulator version: [latest](https://github.com/lge-ros2/multi-robot-simulator/releases/latest)

```shell
## general
sudo apt-get install python3-websocket
sudo apt-get install libzmq3-dev libprotobuf-dev protobuf-compiler

## for camera driver sim
sudo sudo apt-get install ros-foxy-image-transport-plugins ros-foxy-camera-info-manager
```

## Build

Please setup ROS2 environment first!

```shell
source /opt/ros2/foxy/setup.bash
colcon build --packages-up-to sim_device_bringup
```

## Usage

Set environment variable.

```shell
export SIM_BRIDGE_IP='127.0.0.1'
```

### driver sim

check here [details](https://github.com/lge-ros2/sim_device/tree/foxy/bringup)

```shell
ros2 launch sim_device_bringup **driver_sim**.launch.py robot_name:=cloi1
```

### elevator system sim

```shell
ros2 launch sim_device_bringup **elevator_system_sim**.launch.py
```

### CLOiSim(simulator) with world

#### only simulator

```shell
ros2 launch sim_device_bringup cloisim.launch.py sim_path:=/opt/CLOiSim/CLOiSim-1.4.0 world:=lg_seocho.world
```

#### simulator + unity-ros2 packge(clock topic)

```shell
ros2 launch sim_device_bringup cloisim_world.launch.py sim_path:=/opt/CLOiSim/CLOiSim-1.4.0 world:=lg_seocho.world
```

##### examples

```shell
ros2 launch sim_device_bringup driver_sim.launch.py robot_name:=cloi

ros2 launch sim_device_bringup world_sim.launch.py
```

### Using Docker

Run below command after clone this repository(this branch).

#### Build image

```shell
git clone https://github.com/lge-ros2/sim_device.git -b foxy
cd sim_device
docker build -t sim_device .
```

#### Running container with laucnher

```shell
docker run -it --rm --net=host sim_device {launch script} {arguments}
```

##### How to run container

###### launchers

[here](https://github.com/lge-ros2/sim_device/tree/foxy/bringup/launch) to check launchers

```shell
docker run -it --rm --net=host sim_device driver_sim.launch.py robot_name:=cloi1

docker run -it --rm --net=host sim_device world_sim.launch.py
```

###### cloisim_world

it requires to volume mount(-v option) for sim_path and resource
refer to [here](https://github.com/lge-ros2/cloisim/tree/master/Docker)

```shell
docker run -it --rm --net=host sim_device cloisim_world.launch.py sim_path:=/opt/CLOiSim-1.7.1 world:=lg_seocho.world
```

## Version info

- Please refer to each branch for ROS2 distro-version you want
  - [dashing](https://github.com/lge-ros2/sim_device/tree/dashing)
  - [foxy](https://github.com/lge-ros2/sim_device/tree/foxy)
