# sim-device (dashing version)
ROS2 simulation device packages to connect the unity3D based multi-robot simulator(latest version).

# prerequisite

- Download Simulator release
  - CLOiSim: Unity multi-robot simulator [releases](https://github.com/lge-ros2/multi-robot-simulator/releases)
    - Simulator version: [latest](https://github.com/lge-ros2/multi-robot-simulator/releases/latest)
- $ sudo apt-get install libzmq3-dev libprotobuf-dev protobuf-compiler

# build
Please setup ROS2 environment first!
- $ source /opt/ros2/dashing/setup.bash
- $ colcon build --packages-up-to simdevice_bringup

# Usage

## driver sim
- check here [details](https://github.com/lge-ros2/sim-device/tree/dashing/bringup)
- $ ros2 launch simdevice_bringup **driver_sim**.launch.py robot_name:=cloi1 



## elevator system sim
- $ ros2 launch simdevice_bringup **elevator_system_sim**.launch.py

## simulator(CLOiSim) with world
- only simulator
  - $ ros2 launch simdevice_bringup **cloisim**.launch.py sim_path:=/opt/CLOiSim/CLOiSim-1.4.0 world:=lg_seocho.world

- simulator + unity-ros2 packge(clock topic)
  - $ ros2 launch simdevice_bringup **cloisim_world**.launch.py sim_path:=/opt/CLOiSim/CLOiSim-1.4.0 world:=lg_seocho.world
