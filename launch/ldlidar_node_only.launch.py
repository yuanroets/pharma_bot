#!/usr/bin/env python3
"""
LiDAR Node Only Launch - NO Robot State Publisher
================================================

This launch file starts ONLY the LiDAR node without the robot_state_publisher
that conflicts with our main robot URDF. Based on ldlidar_bringup.launch.py
but with the robot_state_publisher removed.

Usage on Pi:
    ros2 launch pharma_bot ldlidar_node_only.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # Launch Configuration Variables
    node_ns = LaunchConfiguration('node_namespace')
    node_name = LaunchConfiguration('node_name')
    container_name = LaunchConfiguration('container_name')
    
    # Launch Arguments
    declare_node_namespace = DeclareLaunchArgument(
        'node_namespace',
        default_value='',
        description='Namespace for the LiDAR node'
    )
    
    declare_node_name = DeclareLaunchArgument(
        'node_name',
        default_value='ldlidar_node',
        description='Name for the LiDAR node'
    )
    
    declare_container_name = DeclareLaunchArgument(
        'container_name',
        default_value='ldlidar_container',
        description='Name for the component container'
    )

    # LiDAR node configuration file
    lidar_config_path = os.path.join(
        get_package_share_directory('ldlidar_node'),
        'params',
        'ldlidar.yaml'
    )

    # LiDAR Component Container
    ldlidar_container = ComposableNodeContainer(
        name=container_name,
        namespace=node_ns,
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='ldlidar_node',
                plugin='ldlidar::LdLidarComponent',
                name=node_name,
                namespace=node_ns,
                parameters=[lidar_config_path],
            )
        ],
        output='screen',
    )

    # Build Launch Description
    ld = LaunchDescription()

    # Add arguments
    ld.add_action(declare_node_namespace)
    ld.add_action(declare_node_name)
    ld.add_action(declare_container_name)
    
    # Add LiDAR node (NO robot_state_publisher)
    ld.add_action(ldlidar_container)

    return ld
