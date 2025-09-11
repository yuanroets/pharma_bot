#!/usr/bin/env python3
"""
Dev Machine Test Launch File - TELEOP + BASIC VISUALIZATION
==========================================================

This launch file starts teleop + basic RViz visualization on the dev machine:
- Teleop keyboard control
- Robot state publisher (URDF)
- Static transforms (coordinate frame linking)
- RViz2 with robot model and LiDAR data

Usage on dev machine:
    ros2 launch pharma_bot dev_test_launch.py

Make sure to set ROS_DOMAIN_ID=30 to match the Pi.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package and file paths
    pkg_pharma_bot = get_package_share_directory('pharma_bot')
    robot_description_file = os.path.join(pkg_pharma_bot, 'description', 'robot.urdf.xacro')
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

    # CRITICAL TRANSFORMS for LiDAR integration
    # Static Transform: base_link -> ldlidar_link
    # This links the robot model to the LiDAR coordinate frame (matches LiDAR config)
    static_tf_base_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_lidar',
        arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'ldlidar_link'],
        output='screen'
    )

    # Static Transform: odom -> base_link (robot position in odometry frame)
    # NOTE: This is temporary - in future, motor driver should publish odometry
    static_tf_odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_to_base', 
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        output='screen'
    )

    # Teleop Keyboard Control - For driving the robot
    teleop_keyboard = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='gnome-terminal --',  # Opens in new terminal window
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        remappings=[('/cmd_vel', '/cmd_vel')]  # Direct to serial_motor_demo teleop_bridge
    )

    # RViz2 - 3D visualization with robot model and LiDAR data
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

    # Delayed start for teleop (gives time for everything to initialize)
    delayed_teleop = TimerAction(
        period=3.0,
        actions=[teleop_keyboard]
    )

    # Build Launch Description  
    ld = LaunchDescription()

    # Add arguments
    ld.add_action(declare_use_sim_time)
    
    # Add core visualization components (start immediately)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(static_tf_base_to_lidar)  # CRITICAL for LiDAR
    ld.add_action(static_tf_odom_to_base)   # CRITICAL for transforms
    ld.add_action(rviz2)
    
    # Add delayed teleop
    ld.add_action(delayed_teleop)

    return ld
