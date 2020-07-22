# simdevice_bringup

The `simdevice_bringup` package is an bringup system for sim_bridge.


You need to edit "./launch/driver_sim.launch.py" file to bringup 'driver_sim' package that you want.

    # define launch script name
    # launch script name
    #   ex) lidar_driver_sim.launch.py -> "lidar_driver_sim"
    _launch_list = [
        'lidar_driver_sim',
        'micom_driver_sim',
        # 'camera_driver_sim',
        # 'depth_camera_driver_sim',
        # 'multi_camera_driver_sim'
        # 'gps_driver_sim',
    ]
