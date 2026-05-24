from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import FindExecutable, PathJoinSubstitution, LaunchConfiguration, Command


def generate_launch_description():
    # Load robot model
    robot_description = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare("nav2_indoor"), "urdf/fwmini.xacro"]),
    ])

    # Launch robot state publisher
    return LaunchDescription([
        # Configure launcher
        DeclareLaunchArgument(
            'use_sim_time', 
            default_value='False',
            description='Use simulation clock if true'
        ),
        LogInfo(msg=['use_sim_time: ', LaunchConfiguration('use_sim_time')]),
        
        # Invoke ROS node
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='both',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_description': robot_description
            }]
        ),
    ])
