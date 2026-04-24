# Protobuf Messages

Describes protobuf messages for communication with CLOiSim (Unity multi-robot simulator).

Inspired from Gazebo Protobuf messages. A total of 231 message types are defined, of which 21 are currently actively used.

## Currently Used Messages (21)

### Core Messages
- **any.proto** - General-purpose parameter/data container
- **param.proto** - Parameter/configuration messages
- **time.proto** - Timestamp messages

### Sensor Data Messages
- **camerasensor.proto** - Camera sensor configuration and parameters
- **image_stamped.proto** - Single timestamped image data
- **images_stamped.proto** - Multiple timestamped images (multi-camera)
- **imu.proto** - IMU (Inertial Measurement Unit) sensor data
- **gps.proto** - GPS/GNSS sensor data
- **laserscan_stamped.proto** - Timestamped LiDAR/laser scan data
- **sonar_stamped.proto** - Timestamped sonar sensor data
- **segmentation.proto** - Image segmentation data

### Control Messages
- **joint_cmd_v.proto** - Joint control commands (vector format)
- **joint_state_v.proto** - Joint state feedback (vector format)
- **twist.proto** - Linear and angular velocity commands
- **joystick.proto** - Joystick/controller input

### Micom/Platform Messages
- **micom.proto** - Micom module interface for mobile platforms

### Perception/Vision Messages
- **perception_v.proto** - Perception data including object detection (vector format)

### Pose and Transform Messages
- **pose.proto** - Position and orientation data
- **transform_stamped.proto** - Timestamped coordinate frame transformations

### World Information
- **world_stats.proto** - World statistics and simulation information

## Message Generation

These protobuf messages are compiled to generate language-specific bindings (C++, Python, etc.) during the build process through CMake.
