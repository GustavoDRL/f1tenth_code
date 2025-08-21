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
    qos_config_file = os.path.join(pkg_share, 'config', 'f1tenth_qos.yaml')
    
    # Launch parameters - REAL hardware time
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time - FALSE for real F1Tenth hardware'),

        # Cartographer Node with F1Tenth QoS configuration
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                qos_config_file
            ],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', configuration_basename
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

        # RViz for F1Tenth
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