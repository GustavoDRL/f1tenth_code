from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_mapper')
    
    # Configuration files
    cartographer_config_dir = os.path.join(pkg_share, 'config')
    configuration_basename = 'cartographer_2d.lua'
    
    # Launch parameters - REAL hardware time
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time - FALSE for real F1Tenth hardware'),

        # QoS Bridge for /scan compatibility
        Node(
            package='topic_tools',
            executable='relay',
            name='scan_qos_bridge',
            arguments=['/scan', '/scan_reliable'],
            parameters=[
                {'input_qos': 'sensor_data'},    # BEST_EFFORT compatible
                {'output_qos': 'default'},       # RELIABLE for Cartographer
            ],
            output='screen'
        ),

        # Cartographer Node with remapped scan topic
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time}
            ],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', configuration_basename
            ],
            remappings=[
                ('/scan', '/scan_reliable'),  # Use bridged reliable scan
            ]),

        # Occupancy Grid Node  
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time}
            ]),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'mapping.rviz')],
            parameters=[
                {'use_sim_time': use_sim_time}
            ],
            output='screen'),
    ])