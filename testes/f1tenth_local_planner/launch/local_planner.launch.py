from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Obter diretório do pacote
    pkg_share = get_package_share_directory('f1tenth_local_planner')
    config_file = os.path.join(pkg_share, 'config', 'planner_params.yaml')

    # Declarar argumento para nível de log
    log_level = LaunchConfiguration('log_level')
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level'
    )

    # Declarar argumento para arquivo de parâmetros
    params_file = LaunchConfiguration('params_file')
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=config_file,
        description='Full path to parameter file'
    )

    return LaunchDescription([
        # Adicionar argumentos
        log_level_arg,
        params_file_arg,

        # Nó do planejador local
        Node(
            package='f1tenth_local_planner',
            executable='local_planner',
            name='local_planner',
            output='screen',
            parameters=[params_file],
            arguments=['--ros-args', '--log-level', log_level]
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_share, 'config', 'local_planner.rviz')]
        )
    ])