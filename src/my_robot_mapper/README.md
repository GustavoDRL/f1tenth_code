# My Robot Mapper

A ROS 2 package for 2D mapping using Google Cartographer. This package provides the necessary configuration and launch files to create 2D maps from laser scan data.

## Prerequisites

* ROS 2 (tested with Humble)
* Cartographer ROS
* A robot/simulator providing:
  - LaserScan data on `/scan` topic
  - TF transforms between required frames (base_link, odom)

## Dependencies

* rclcpp
* geometry_msgs
* nav_msgs
* sensor_msgs
* tf2
* tf2_ros
* cartographer_ros

## Usage

Launch the mapping node:

```bash
ros2 launch my_robot_mapper mapping.launch.py
```

This will start:
- Cartographer node for SLAM
- Occupancy grid generator
- RViz2 with preconfigured visualization

## Configuration

### Cartographer Configuration

The main Cartographer configuration is located in `config/cartographer_2d.lua`. Key parameters include:
- Map frame: "map"
- Tracking frame: "base_link"
- Published frame: "odom"
- Minimum range: 0.12 meters
- Maximum range: 3.5 meters
- Online correlative scan matching enabled

### RViz Configuration

The package includes a preconfigured RViz setup (`rviz/mapping.rviz`) that displays:
- TF frames
- LaserScan data
- Generated map

## Directory Structure

```
my_robot_mapper/
├── config/
│   ├── cartographer_2d.lua
│   └── mapper_params.yaml
├── launch/
│   └── mapping.launch.py
├── maps/
├── rviz/
│   └── mapping.rviz
├── CMakeLists.txt
└── package.xml
```