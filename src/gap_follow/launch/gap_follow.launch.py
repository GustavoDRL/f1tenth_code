from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    Creates a launch description that works with the existing transform tree.
    The transform tree already has map -> ego_racecar/base_link -> ego_racecar/laser,
    so we just need to configure our nodes to use these frames.
    """
    pkg_share = get_package_share_directory('gap_follow')
    
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config_path = os.path.join(pkg_share, 'config', 'rviz', 'gap_follow.rviz')
    params_file = os.path.join(pkg_share, 'config', 'gap_follow_params.yaml')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time if true'),

        # Gap Follower Node using existing frames
        Node(
            package='gap_follow',
            executable='gap_follow_node',
            name='gap_follow',
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time},
                {'frame_id': 'ego_racecar/laser'}  # Using existing laser frame
            ],
            remappings=[
                ('scan', '/scan'),
                ('cmd_vel', '/cmd_vel'),
            ]
        ),

        # RViz with specific frame settings
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        )
    ])