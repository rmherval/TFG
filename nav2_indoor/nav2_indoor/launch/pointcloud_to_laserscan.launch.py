# file: launch/pointcloud_to_laserscan_launch.py
import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan_node',
            output='screen',
            parameters=[
                {
                    'cloud_in': '/rslidar_points',        # ← reemplazar con tu topic
                    'scan': '/scan_to_pc2',                    # ← topic de salida LaserScan
                    'min_height': -0.3,
                    'max_height': 5.0,
                    'angle_min': -1.5708,   # -90°
                    'angle_max': 1.5708,    # +90°
                    'angle_increment': 0.0174533,                  # ~1° en rad
                    'range_min': 0.3,
                    'range_max': 10.0,
                    'target_frame': 'base_link',                 # ← tu frame
                    'transform_tolerance': 0.1,
                    'use_inf': False
                }
            ],
            remappings=[('/cloud_in', '/rslidar_points')]
        ),

    ])
