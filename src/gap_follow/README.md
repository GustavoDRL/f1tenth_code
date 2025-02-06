
# Gap Follow Navigation Package

A ROS2 package implementing autonomous navigation using laser scan gap detection and PID control. The package uses a threaded architecture to efficiently process laser scan data and generate control commands for autonomous vehicle navigation.

## Description

This package implements a gap-following navigation strategy that:
- Processes laser scan data to detect navigable gaps in the environment
- Uses PID control for steering and speed regulation
- Implements safety features and emergency stopping
- Provides debug visualization capabilities

## Features

- Robust laser scan preprocessing with median filtering
- Thread-safe scan processing and control computation
- Configurable PID controllers for steering and speed
- Gap detection with customizable parameters
- Safety margin enforcement and emergency stop capabilities
- Debug data publishing for monitoring and tuning

## Dependencies

### ROS2 Packages
- rclpy
- sensor_msgs
- laser_geometry
- ackermann_msgs
- visualization_msgs
- geometry_msgs
- std_msgs
- nav_msgs
- builtin_interfaces
- launch_ros
- rviz2

### Python Packages
- numpy
- simple-pid
- scikit-image

## Usage

### Launch the Node

```bash
ros2 launch gap_follow gap_follow.launch.py
```

### Configuration

The node can be configured through ROS2 parameters. Key parameters include:

- `max_range`: Maximum valid laser scan range (default: 10.0 meters)
- `min_gap_width`: Minimum width for navigable gaps (default: 0.8 meters)
- `max_gap_depth`: Maximum allowed gap depth (default: 5.0 meters)
- `max_speed`: Maximum allowed velocity (default: 0.50 m/s)
- `min_speed`: Minimum velocity threshold (default: 0.1 m/s)
- `max_steering_angle`: Maximum steering angle (default: 0.7 radians)
- `safety_margin`: Minimum obstacle clearance (default: 0.5 meters)

PID controller parameters can also be configured:
```yaml
steering_pid:
  p: 2.2
  i: 0.01
  d: 0.3

speed_pid:
  p: 0.8
  i: 0.05
  d: 0.1
```

## Topics

### Subscribed Topics
- `/scan` (sensor_msgs/LaserScan): Laser scan data
- `/odom` (nav_msgs/Odometry): Vehicle odometry

### Published Topics
- `/drive` (ackermann_msgs/AckermannDriveStamped): Control commands
- `/pid_debug` (std_msgs/Float32MultiArray): PID controller debug data

