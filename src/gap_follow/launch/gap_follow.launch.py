from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():    
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time if true'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                get_package_share_directory('gap_follow'),
                'config', 'gap_follow.yaml'
            ),
            description='Path to node parameter YAML file'
        ),
        Node(
            package='gap_follow',
            executable='gap_follow',
            name='gap_follower',
            output='screen',
            emulate_tty=True,
            arguments=['--ros-args', '--log-level', 'debug'],
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time},
                {'frame_id': 'ego_racecar/laser'}
            ],
        ),
    ])