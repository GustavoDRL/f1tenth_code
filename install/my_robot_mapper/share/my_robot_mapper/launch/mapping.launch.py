from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_mapper')
    
    # Arquivos de configuração
    cartographer_config_dir = os.path.join(pkg_share, 'config')
    configuration_basename = 'cartographer_2d.lua'
    
    # Parâmetros do launch
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'),

        # Nó do Cartographer
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', cartographer_config_dir,
                      '-configuration_basename', configuration_basename]),

        # Nó do Occupancy Grid - Corrigido
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',  # Nome correto do executável
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'mapping.rviz')],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])