# Joy to Ackermann Controller

A ROS2 node that converts joystick inputs to Ackermann drive commands for vehicle control. This node subscribes to joy messages and publishes Ackermann drive commands, allowing smooth control of vehicles using a joystick/gamepad.

## Features

- Converts joystick axis inputs to Ackermann steering commands
- Configurable maximum speed and steering angle limits
- Dead zone filtering to prevent unwanted movement from small joystick inputs
- Ability to reset vehicle position using PS button
- Initial pose publishing capability

## Dependencies

- ROS2
- `ackermann_msgs`
- `sensor_msgs`
- `geometry_msgs`
- Python 3

## Usage

Launch the node:
```bash
ros2 run <package_name> joy_to_ackermann.py
```

### Topics

Subscribed:
- `/joy` - Joy messages from the joystick/gamepad

Published:
- `/drive` - Ackermann drive commands
- `/initialpose` - Initial pose for vehicle position reset

### Parameters

- `max_speed`: Maximum vehicle speed (7.0 m/s)
- `max_angle`: Maximum steering angle (0.32 rad)
- `controller_error`: Dead zone threshold for joystick inputs (0.1)

### Controls

- Left stick vertical axis: Controls vehicle speed
- Right stick horizontal axis: Controls steering angle
- PS button: Resets vehicle position to initial pose

