from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():    
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time if true'
        ),
        Node(
            package='gap_follow',
            executable='gap_follow',
            name='gap_follow',
            output='screen',
            emulate_tty=True,
            arguments=['--ros-args', '--log-level', 'debug'],
            parameters=[
                {'use_sim_time': use_sim_time},
                {'frame_id': 'ego_racecar/laser'}
            ],
        ),
    ])