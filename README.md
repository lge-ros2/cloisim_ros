# sim-device (dashing version)

ROS2 simulation device packages to connect the unity3D based multi-robot simulator(latest version).

## prerequisite

- Download Simulator release
  - CLOiSim: Unity multi-robot simulator [releases](https://github.com/lge-ros2/multi-robot-simulator/releases)
    - Simulator version: [latest](https://github.com/lge-ros2/multi-robot-simulator/releases/latest)

```shell
## general
sudo apt-get install python-websocket
sudo apt-get install libzmq3-dev libprotobuf-dev protobuf-compiler

## for camera driver sim
sudo sudo apt-get install ros-dashing-camera-info-manager
```

## build

Please setup ROS2 environment first!

```shell
source /opt/ros2/dashing/setup.bash
colcon build --packages-up-to simdevice_bringup
```

## Usage

Set environment variable.

```shell
export SIM_BRIDGE_IP='127.0.0.1'
```

### driver sim

check here [details](https://github.com/lge-ros2/sim-device/tree/dashing/bringup)

```shell
ros2 launch simdevice_bringup **driver_sim**.launch.py robot_name:=cloi1
```

### elevator system sim

```shell
ros2 launch simdevice_bringup **elevator_system_sim**.launch.py
```

### simulator(CLOiSim) with world

- only simulator

```shell
ros2 launch simdevice_bringup **cloisim**.launch.py sim_path:=/opt/CLOiSim/CLOiSim-1.4.0 world:=lg_seocho.world
```

- simulator + unity-ros2 packge(clock topic)

```shell
ros2 launch simdevice_bringup **cloisim_world**.launch.py sim_path:=/opt/CLOiSim/CLOiSim-1.4.0 world:=lg_seocho.world
```


### Using Docker

Run below command after clone this repository(this branch)

#### build image

```shell
git clone https://github.com/lge-ros2/sim-device.git -b dashing
cd sim-device
docker build -t simdevice .
```

#### launch packages

```shell
docker run -it --rm --net=host simdevice {launch script} {arguments}
```

##### examples

```shell
docker run -it --rm --net=host simdevice driver_sim.launch.py robot_name:=cloi1

docker run -it --rm --net=host simdevice world_sim.launch.py
```

###### cloisim_world

it requires to volume mount(-v option) for sim_path and resource
refer to [here](https://github.com/lge-ros2/cloisim/tree/master/Docker)

```shell
docker run -it --rm --net=host simdevice cloisim_world.launch.py sim_path:=/opt/CLOiSim-1.7.1 world:=lg_seocho.world
```

or you can just luanch with launch.sh script

```shell
docker run -it --rm --net=host simdevice {launch script} {arguments}
```

##### examples

```shell
./launch.sh driver_sim.launch.py robot_name:=cloi

./launch.sh world_sim.launch.py
```