import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Directorio del paquete de configuración
    config_dir = get_package_share_directory('nav2_indoor') + '/config/'

    # Declare launch arguments
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='log level'
    )
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack',
    )

    #Activar automaticamente estos nodos
    lifecycle_nodes = [
        'map_server',
        'amcl', 
        'controller_server',
        'smoother_server', 
        'planner_server',
        'behavior_server', 
        'bt_navigator', 
        'waypoint_follower', 
        'velocity_smoother'
    ]

    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml_file',
        default_value=config_dir + 'map_server.yaml',
        description='Full path to map yaml file')

    lifecycle_params_arg = DeclareLaunchArgument(
        'lifecycle_params_file',
        default_value=config_dir + 'lifecycle_params.yaml',
        description='Full path to Lifecycle Manager parameters file')

    amcl_params_arg = DeclareLaunchArgument(
        'amcl_params_file',
        default_value=config_dir + 'amcl_params.yaml',
        description='Full path to AMCL parameters file')


    nav2_params_arg = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=config_dir + 'nav2_params.yaml',
        description='Full path to Nav2 parameters file')

    configured_params = LaunchConfiguration('nav2_params_file')
    log_level = LaunchConfiguration('log_level')
    use_respawn = False 
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # Nodes
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[LaunchConfiguration('map_yaml_file')]
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[LaunchConfiguration('amcl_params_file')]
    )

    controller_server_node =  Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
    )

    smoother_server_node =  Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
    )

    planner_server_node =  Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            )

    behavior_server_node =  Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
            )
    
    bt_navigator_node =  Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            )
    
    waypoint_follower_node =  Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            )

    velocity_smoother_node =  Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
            )

    collision_monitor_node =  Node(
                package='nav2_collision_monitor',
                executable='collision_monitor',
                name='collision_monitor',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            )

    lifecycle_mgr_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{'autostart': LaunchConfiguration('autostart')}, {'node_names': lifecycle_nodes}],
    )

    return LaunchDescription([
        declare_autostart_cmd,
        declare_log_level_cmd,
        map_yaml_arg,
        amcl_params_arg,
        nav2_params_arg,
        lifecycle_params_arg,
        map_server_node,
        amcl_node,
        lifecycle_mgr_node,
        controller_server_node, 
        smoother_server_node, 
        planner_server_node, 
        behavior_server_node, 
        bt_navigator_node, 
        waypoint_follower_node, 
        velocity_smoother_node
    ])
