#!/usr/bin/env python3
"""
Dev Machine SLAM Launch File
============================

This launch file starts SLAM mapping components on the dev machine:
- SLAM Toolbox with lifecycle management
- Topic remapping from /ldlidar_node/scan to /scan

Usage on dev machine (after dev_test_launch.py is running):
    ros2 launch pharma_bot dev_slam_launch.py

Make sure to set ROS_DOMAIN_ID=30 to match the Pi.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode


def generate_launch_description():
    # Package Directories  
    pkg_pharma_bot = get_package_share_directory('pharma_bot')
    
    # Launch Configuration Variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Launch Arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time for real robot'
    )

    # Lifecycle manager for SLAM Toolbox (same approach as ldlidar vendors)
    slam_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[
            '/home/ubuntu/dev_ws/src/pharma_bot/config/lifecycle_mgr_slam.yaml'
        ]
    )

    # SLAM Toolbox Node - Maps environment using LiDAR data (EXACT vendor approach)
    slam_toolbox_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        namespace='',
        name='slam_toolbox',
        output='screen',
        parameters=[
            '/home/ubuntu/dev_ws/src/pharma_bot/config/mapper_params_online_async.yaml'
        ],
        remappings=[
            ('/scan', '/ldlidar_node/scan')  # Remap from standard /scan to actual LiDAR topic
        ]          
    )

    # Build Launch Description  
    ld = LaunchDescription()

    # Add arguments
    ld.add_action(declare_use_sim_time)
    
    # Add SLAM components
    ld.add_action(slam_lifecycle_manager)        # Lifecycle manager for SLAM (vendor approach)
    ld.add_action(slam_toolbox_node)             # SLAM mapping with topic remapping

    return ld
