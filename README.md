# sim-device (dashing version)
ROS2 simulation device packages to connecting unity based multi-robot simulator

# build
Please setup ROS2 environment first!
- source /opt/ros2/dashing/setup.bash
- colcon build --packages-up-to simdevice_bringup

# Usage
## driver sim
- ros2 launch simdevice_bringup **driver_sim**.launch.py robot_name:=cloi1 

## elevator system sim
- ros2 launch simdevice_bringup **elevator_system_sim**.launch.py

## simulator(CLOiSim) with world
- only simulator
  - ros2 launch simdevice_bringup **cloisim**.launch.py sim_path:=/home/yg/Work/lgrs_config/SimulatorInstance/CLOiSimRelease/CLOiSim-1.0.1 world:=lg_seocho.world

- simulator + unity-ros2 packge(clock topic)
  - ros2 launch simdevice_bringup **cloisim_world**.launch.py sim_path:=/home/yg/Work/lgrs_config/SimulatorInstance/CLOiSimRelease/CLOiSim-1.0.1 world:=lg_seocho.world
