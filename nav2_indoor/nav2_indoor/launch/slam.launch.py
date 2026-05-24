import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Directorio del paquete de configuración
    config_dir = get_package_share_directory('nav2_indoor') + '/config/'

    return LaunchDescription([
        DeclareLaunchArgument('slam_params_file', default_value=config_dir + 'slam_params.yaml'),

        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[LaunchConfiguration('slam_params_file')],
            remappings=[('/scan', '/scan')]
        ),
    ])
