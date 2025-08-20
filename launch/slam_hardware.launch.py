#!/usr/bin/env python3
"""
SLAM Launch File for Hardware Robot with LD19 LiDAR
====================================================

This launch file starts SLAM (Simultaneous Localization and Mapping) on the actual hardware robot.
It integrates:
- Your existing motor driver and teleop bridge
- LD19 LiDAR for environment sensing  
- SLAM Toolbox for real-time mapping
- RViz for visualization

Usage:
    ros2 launch pharma_bot slam_hardware.launch.py

What this does:
1. Starts SLAM Toolbox configured for your LD19 LiDAR
2. Launches RViz with mapping configuration
3. Sets up proper coordinate frames and timing

Prerequisites:
- Motor driver should be running: ros2 run serial_motor_demo driver
- Teleop bridge should be running: ros2 run serial_motor_demo teleop_bridge  
- LiDAR driver should be running: ros2 launch ldlidar_node ldlidar_bringup.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import HasNodeParams


def generate_launch_description():
    # Launch Configuration Variables
    # These can be overridden when launching: ros2 launch pharma_bot slam_hardware.launch.py use_sim_time:=false
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    
    # Default parameter file for SLAM - contains our LD19 LiDAR configuration
    default_params_file = os.path.join(
        get_package_share_directory("pharma_bot"),
        'config', 'mapper_params_online_async.yaml'
    )

    # Default RViz configuration for mapping visualization
    rviz_config_file = os.path.join(
        get_package_share_directory("pharma_bot"),
        'config', 'map.rviz'  # Use the mapping-specific RViz config
    )

    # Launch Arguments - these define the configurable parameters
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',  # Set to false for hardware (real time)
        description='Use simulation/Gazebo clock if true, real time if false'
    )
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file for SLAM configuration'
    )

    # Parameter file validation
    # This ensures we're using a valid SLAM configuration file
    has_node_params = HasNodeParams(
        source_file=params_file,
        node_name='slam_toolbox'
    )

    actual_params_file = PythonExpression([
        '"', params_file, '" if ', has_node_params,
        ' else "', default_params_file, '"'
    ])

    # Log if we fall back to default parameters
    log_param_change = LogInfo(
        msg=['Provided params_file ', params_file,
             ' does not contain slam_toolbox parameters. Using default: ',
             default_params_file],
        condition=UnlessCondition(has_node_params)
    )

    # SLAM Toolbox Node - The core mapping algorithm
    # This node subscribes to /ldlidar_node/scan and publishes /map
    start_async_slam_toolbox_node = Node(
        parameters=[
            actual_params_file,
            {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        remappings=[
            # Ensure SLAM uses your LD19 LiDAR data
            ('/scan', '/ldlidar_node/scan'),
        ]
    )

    # RViz Node - For real-time visualization of mapping process
    # Shows the robot, LiDAR scans, and the map being built
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Static Transform Publishers - Define coordinate frame relationships
    
    # 1. Base footprint to base link transform
    # base_footprint is on the ground, base_link is at robot center
    base_footprint_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_base_link',
        arguments=[
            '0.0', '0.0', '0.05',   # x, y, z (lift base_link 5cm above ground)
            '0.0', '0.0', '0.0',    # roll, pitch, yaw
            'base_footprint',       # Parent frame (on ground)
            'base_link'             # Child frame (robot center)
        ],
        output='screen'
    )
    
    # 2. Base link to LiDAR transform
    # This tells ROS where the LiDAR is mounted relative to the robot center
    # Adjust these values based on your actual LiDAR mounting position
    base_link_to_lidar_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_lidar_publisher',
        arguments=[
            '0.0', '0.0', '0.15',   # x, y, z translation (LiDAR 15cm above base_link)
            '0.0', '0.0', '0.0',    # roll, pitch, yaw rotation (LiDAR orientation)
            'base_link',            # Parent frame (robot center)
            'ldlidar_link'          # Child frame (LiDAR frame from your driver)
        ],
        output='screen'
    )
    
    # 3. Alternative: If you need base_link to ldlidar_base transform
    base_link_to_ldlidar_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_ldlidar_base',
        arguments=[
            '0.0', '0.0', '0.15',   # Same position as LiDAR
            '0.0', '0.0', '0.0',    # No rotation
            'base_link',            # Parent frame
            'ldlidar_base'          # Child frame (if this is what your driver publishes)
        ],
        output='screen'
    )

    # Build the complete launch description
    ld = LaunchDescription()

    # Add all launch arguments
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(log_param_change)
    
    # Add all nodes
    ld.add_action(start_async_slam_toolbox_node)
    ld.add_action(rviz_node)
    ld.add_action(base_footprint_to_base_link)
    ld.add_action(base_link_to_lidar_transform)
    ld.add_action(base_link_to_ldlidar_base)

    return ld
