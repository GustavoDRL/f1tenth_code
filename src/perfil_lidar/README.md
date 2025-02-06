# Lidar Scan Profile Visualizer

## Description
This ROS2 node visualizes real-time LIDAR scan data in a graphical plot. It subscribes to laser scan messages and displays the range measurements against their corresponding angles, creating a dynamic visualization of the LIDAR's scan profile.

## Features
- Real-time visualization of LIDAR scan data
- Interactive matplotlib plot showing range vs. angle
- Configurable data sampling rate
- Display of minimum and maximum scan angles
- Automatic plot scaling

## Prerequisites
- ROS2 (Robot Operating System 2)
- Python 3
- Required Python packages:
  - rclpy
  - numpy
  - matplotlib
  - sensor_msgs

## Usage
Run the node
```bash
ros2 run perfil_lidar perfil_lidar
```

## Node Details
- Node name: `perfil_lidar`
- Subscribed topics:
  - `/scan` (sensor_msgs/LaserScan): LIDAR scan data

## Configuration
The node includes a configurable parameter:
- `incremento`: Controls the number of data points sampled from the scan (default: 180)

## Output
- Interactive plot showing:
  - X-axis: Angle (radians)
  - Y-axis: Range (meters)
- Console output:
  - Minimum scan angle (degrees)
  - Maximum scan angle (degrees)
  - Number of scan points

