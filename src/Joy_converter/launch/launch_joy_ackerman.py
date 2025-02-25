#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),
        Node(
            package='joy_converter',
            executable='joy_ackerman',
            name='joy_ackerman',
            output='screen'
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
