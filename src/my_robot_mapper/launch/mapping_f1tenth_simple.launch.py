from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_mapper')
    
    # Configuration files
    cartographer_config_dir = os.path.join(pkg_share, 'config')
    configuration_basename = 'cartographer_2d.lua'
    
    # Launch parameters - REAL hardware time
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # QoS override parameters for cross-platform compatibility
    qos_overrides = ParameterValue({
        '/cartographer_node': {
            'subscriptions': {
                '/scan': {
                    'reliability': 'best_effort',
                    'durability': 'volatile',
                    'history': 'keep_last',
                    'depth': 5
                }
            }
        }
    })
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time - FALSE for real F1Tenth hardware'),

        # Cartographer Node with QoS override
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'qos_overrides_global_publisher_policies': qos_overrides}
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