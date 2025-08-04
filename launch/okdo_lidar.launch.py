#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Declare arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for OKDO LiDAR'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='laser_frame',
        description='Frame ID for LiDAR data'
    )
    
    scan_frequency_arg = DeclareLaunchArgument(
        'scan_frequency',
        default_value='10.0',
        description='LiDAR scan frequency in Hz'
    )
    
    range_min_arg = DeclareLaunchArgument(
        'range_min',
        default_value='0.15',
        description='Minimum range for LiDAR measurements'
    )
    
    range_max_arg = DeclareLaunchArgument(
        'range_max',
        default_value='8.0',
        description='Maximum range for LiDAR measurements'
    )

    # OKDO LiDAR Node
    # Note: This assumes you have an OKDO LiDAR ROS2 driver package
    # You may need to install or build the appropriate driver
    okdo_lidar_node = Node(
        package='okdo_lidar_ros2',  # Replace with actual package name
        executable='okdo_lidar_node',  # Replace with actual executable name
        name='okdo_lidar',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'frame_id': LaunchConfiguration('frame_id'),
            'scan_frequency': LaunchConfiguration('scan_frequency'),
            'range_min': LaunchConfiguration('range_min'),
            'range_max': LaunchConfiguration('range_max'),
            'angle_min': -3.14159,  # -180 degrees
            'angle_max': 3.14159,   # +180 degrees
            'angle_increment': 0.0174533,  # 1 degree
            'time_increment': 0.0,
            'scan_time': 0.1,
        }]
    )
    
    # Alternative: If no specific OKDO driver exists, you might use a generic serial LiDAR driver
    # Uncomment and modify the following if needed:
    
    # generic_lidar_node = Node(
    #     package='serial_lidar',  # Generic serial LiDAR package
    #     executable='serial_lidar_node',
    #     name='okdo_lidar',
    #     output='screen',
    #     parameters=[{
    #         'port': LaunchConfiguration('serial_port'),
    #         'frame_id': LaunchConfiguration('frame_id'),
    #         'baud_rate': 230400,  # Common for LiDAR sensors
    #         'scan_frequency': LaunchConfiguration('scan_frequency'),
    #         'range_min': LaunchConfiguration('range_min'),
    #         'range_max': LaunchConfiguration('range_max'),
    #     }]
    # )

    return LaunchDescription([
        LogInfo(msg='Starting OKDO LiDAR...'),
        serial_port_arg,
        frame_id_arg,
        scan_frequency_arg,
        range_min_arg,
        range_max_arg,
        okdo_lidar_node,
    ])
