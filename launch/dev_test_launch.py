#!/usr/bin/env python3
"""
Dev Machine Test Launch File - TELEOP + BASIC VISUALIZATION
===========================================================

This launch file starts teleop + basic RViz visualization on the dev machine:
- Teleop keyboard control
- Robot state publisher (URDF)
- LiDAR state publisher (for ldlidar_base → ldlidar_link transform)
- Static transforms (coordinate frame linking)
- RViz2 with robot model and LiDAR data

Usage on dev machine:
    ros2 launch pharma_bot dev_test_launch.py

Make sure to set ROS_DOMAIN_ID=30 to match the Pi.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
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
    
    # LiDAR Robot State Publisher for ldlidar_base -> ldlidar_link transform
    # This is CRITICAL - provides the final link in the transform chain
    ldlidar_urdf_file = os.path.join(
        get_package_share_directory('ldlidar_node'),
        'urdf',
        'ldlidar_descr.urdf.xml'
    )
    
    with open(ldlidar_urdf_file, 'r') as file:
        ldlidar_robot_description = file.read()
    
    ldlidar_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ldlidar_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ldlidar_robot_description,
            'use_sim_time': use_sim_time,
        }]
    )

    # NOTE: Joint states will come from Pi's motor_to_joint_state_bridge
    # This provides dynamic wheel rotation based on real encoder feedback

    # CRITICAL TRANSFORMS for coordinate frame chain (WITHOUT SLAM)
    # Complete chain: odom → base_link → ldlidar_base → ldlidar_link
    # NOTE: Use 'odom' as Fixed Frame in RViz (not 'map' since we're not using SLAM yet)
    
    # Static Transform: odom -> base_link (robot position in odometry frame)
    # NOTE: This is temporary - in future, motor driver should publish odometry
    static_tf_odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )
    
    # Static Transform: base_link -> ldlidar_base (LiDAR mounting position)
    # Position matches simulation: x=0.122m (forward), z=0.212m (up)
    static_tf_base_to_lidar_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_lidar_base',
        arguments=['0.122', '0', '0.212', '0', '0', '0', 'base_link', 'ldlidar_base']
    )
    
    # NOTE: ldlidar_base -> ldlidar_link transform is provided by ldlidar_state_publisher above

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
    ld.add_action(ldlidar_state_publisher)       # CRITICAL: ldlidar_base → ldlidar_link
    # NOTE: joint_state_publisher removed - Pi provides dynamic joint states
    ld.add_action(static_tf_odom_to_base)        # CRITICAL: odom → base_link
    ld.add_action(static_tf_base_to_lidar_base)  # CRITICAL: base_link → ldlidar_base
    ld.add_action(rviz2)
    
    # Add delayed teleop
    ld.add_action(delayed_teleop)

    return ld
