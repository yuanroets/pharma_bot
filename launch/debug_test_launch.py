#!/usr/bin/env python3
"""
Debug Test Launch File - DIAGNOSTIC LAUNCH FOR TROUBLESHOOTING
==============================================================

This launch file helps debug common issues:
- Tests robot description publishing
- Tests transform tree
- Tests LiDAR data flow
- Minimal setup for easier debugging

Usage:
    ros2 launch pharma_bot debug_test_launch.py

"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    # Package Directories  
    pkg_pharma_bot = get_package_share_directory('pharma_bot')
    
    # URDF File Path
    robot_description_file = os.path.join(pkg_pharma_bot, 'description', 'robot.urdf.xacro')
    
    # RViz Configuration
    rviz_config_file = os.path.join(pkg_pharma_bot, 'config', 'pharma_bot.rviz')
    
    # Launch Configuration Variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Launch Arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time for real robot'
    )

    # Robot State Publisher - Loads and publishes robot URDF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', robot_description_file]),
            'use_sim_time': use_sim_time,
        }]
    )

    # Joint State Publisher - Publishes wheel joint positions
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )

    # CRITICAL TRANSFORMS for coordinate frame chain
    # Complete chain: odom → base_link → ldlidar_base → ldlidar_link
    
    # Static Transform: odom -> base_link (robot position in odometry frame)
    static_tf_odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )
    
    # Static Transform: base_link -> ldlidar_base (LiDAR mounting position)
    static_tf_base_to_lidar_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_lidar_base',
        arguments=['0.122', '0', '0.212', '0', '0', '0', 'base_link', 'ldlidar_base']
    )

    # Static Transform: ldlidar_base -> ldlidar_link (temporary for testing)
    static_tf_lidar_base_to_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_lidar_base_to_link',
        arguments=['0', '0', '0', '0', '0', '0', 'ldlidar_base', 'ldlidar_link']
    )

    # RViz2 - 3D visualization with robot model
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )

    # TF debugging commands (delayed)
    tf_debug_tree = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'tf2_tools', 'view_frames'],
                output='screen',
                name='tf_debug_tree'
            )
        ]
    )

    tf_debug_echo = TimerAction(
        period=6.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'topic', 'echo', '/robot_description', '--once'],
                output='screen',
                name='robot_description_check'
            )
        ]
    )

    # Build Launch Description  
    ld = LaunchDescription()

    # Add arguments
    ld.add_action(declare_use_sim_time)
    
    # Add core components
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(static_tf_odom_to_base)
    ld.add_action(static_tf_base_to_lidar_base)
    ld.add_action(static_tf_lidar_base_to_link)  # Temporary for testing
    ld.add_action(rviz2)
    
    # Add debug commands
    ld.add_action(tf_debug_tree)
    ld.add_action(tf_debug_echo)

    return ld
