# F1TENTH Learning and Development Environment

This repository contains my personal development environment for learning and experimenting with the F1TENTH autonomous racing platform. I've created a containerized setup using Docker to facilitate easy deployment of the F1TENTH simulator along with my custom ROS2 nodes for various autonomous driving functionalities.

## Project Purpose

This project serves as my learning platform for:
- Understanding autonomous vehicle control systems
- Implementing and testing different navigation strategies
- Developing skills in ROS2 and robotics programming
- Experimenting with various autonomous racing algorithms

## Prerequisites

- Docker and Docker Compose installed on your system
- Basic understanding of ROS2 and Docker concepts
- X11 server for GUI applications
- Web browser for NoVNC interface access

## Initial Setup

Before running the simulator, you'll need to create the required Docker networks:

```bash
# Create the network for X11 forwarding
docker network create x11

# Create the ROS network for inter-container communication
docker network create ros_net
```

## Getting Started

1. Clone this repository:
```bash
git clone [your-repository-url]
cd [repository-name]
```

2. Build and launch the environment:
```bash
docker compose up --build
```

3. Access the simulator through your web browser:
```
http://localhost:8081
```

## My Custom Implementations

I've developed several ROS2 packages for different aspects of autonomous vehicle control:

### Manual Control System
The Joy to Ackermann Controller allows manual control of the virtual vehicle using a gamepad, which helps in understanding vehicle dynamics and testing basic maneuvers.

### Autonomous Navigation
I've implemented multiple navigation approaches:

- **Gap Following**: A reactive navigation system that identifies and follows gaps in the environment using LIDAR data
- **Wall Following**: A PID-controlled system for maintaining consistent distance from walls
- **SLAM Implementation**: Using Google Cartographer for creating and maintaining maps of the racing environment

### Development Tools
- **LIDAR Visualization**: Custom tools for real-time sensor data visualization and analysis
- **Performance Monitoring**: Systems for tracking and analyzing vehicle behavior

## Working with the Environment

### Development Workflow
The workspace is set up as a mounted volume, allowing you to:
1. Edit code directly on your host machine
2. See changes reflect immediately in the container
3. Test modifications without container rebuilds

### Running Your Code
Access the container and run nodes:
```bash
# Enter the container
docker compose exec sim2 bash

# Setup ROS2 environment
source /opt/ros/humble/setup.bash
source /sim_ws/install/setup.bash

# Run your nodes
ros2 launch [package_name] [launch_file]
```

## Project Structure

```
.
├── docker-compose.yml    # Container orchestration
├── Dockerfile           # Environment configuration
├── src/
│   ├── joy_control/    # Manual control implementation
│   ├── gap_follow/     # Gap following navigation
│   ├── wall_follow/    # Wall following system
│   ├── mapping/        # SLAM and mapping tools
│   └── visualization/  # Data visualization packages
```

## Resources

- [F1TENTH Official Documentation](http://f1tenth.org)
- [ROS2 Documentation](https://docs.ros.org/en/humble/index.html)


## Acknowledgments

This project builds upon the F1TENTH Simulator and incorporates various open-source tools. Special thanks to the F1TENTH community for providing the base platform for learning autonomous racing concepts.