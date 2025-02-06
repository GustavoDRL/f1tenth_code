# Wall Following Robot using ROS2 and PID Control

## Overview
This project implements a wall-following behavior for a robot using ROS2 (Robot Operating System 2). The robot uses LIDAR sensor data to maintain a constant distance from a wall while moving, utilizing PID (Proportional-Integral-Derivative) control for precise navigation.

## Description
The system works by measuring distances to the wall using LIDAR readings at two specific angles (75° and 85°). These measurements are used to calculate the robot's position and orientation relative to the wall. The PID controller then adjusts the steering angle to maintain a desired distance of 0.7 meters from the wall.

## Prerequisites
- ROS2 installation
- Python 3
- Required ROS2 packages:
  - rclpy
  - sensor_msgs
  - ackermann_msgs
- matplotlib (for real-time plotting)

## Key Features
- Real-time LIDAR data processing
- PID control implementation for smooth wall following
- Dynamic velocity adjustment based on error
- Real-time error visualization using matplotlib
- Ackermann steering control

## Configuration
The system uses the following PID parameters, which can be adjusted for different robot configurations:
```python
kp = 14.0    # Proportional gain
ki = 0.007   # Integral gain
kd = 0.09    # Derivative gain
```

The wall-following distance and LIDAR measurement angles can be modified through these parameters:
```python
angleA = 75  # First LIDAR measurement angle
angleB = 85  # Second LIDAR measurement angle
dist = 0.7   # Desired distance from wall (meters)
```

## How It Works
1. The system subscribes to LIDAR scan data through the `/scan` topic
2. Two LIDAR readings are taken at specified angles to determine the robot's position relative to the wall
3. The error between the desired and actual distance is calculated
4. The PID controller generates steering commands based on this error
5. Ackermann drive commands are published to the `/drive` topic
6. Real-time error plotting is available for monitoring system performance

## Running the Node
To run the wall following node:
```bash
ros2 run wall_follow wall_follow
```

## Implementation Details
The code uses geometric calculations to determine the robot's position relative to the wall. The error is calculated using trigonometry:
1. Two distance measurements (a and b) are taken at different angles
2. The angle between the robot and wall (alpha) is calculated
3. The perpendicular distance to the wall is determined
4. The error is computed as the difference between desired and actual distance

The system adjusts velocity based on the error magnitude:
- 1.0 m/s when error < 0.5
- 0.5 m/s when error ≥ 0.5

## Visualization
The code includes real-time plotting capabilities for monitoring the error over time, which can be useful for:
- Tuning PID parameters
- Monitoring system performance
- Debugging control issues

