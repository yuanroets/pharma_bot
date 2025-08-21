#!/usr/bin/env python3
"""
Real Robot Hardware Launch File - FOR RASPBERRY PI
==================================================

This launch file starts ALL hardware components on the Pi:
- Motor driver (communicates with Arduino via USB)
- Teleop bridge (converts cmd_vel to motor commands)
- LiDAR driver and activation

Usage on Pi:
    ros2 launch pharma_bot real_robot_pi.launch.py

This replaces 4 separate terminal commands with just 1!
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch Configuration Variables
    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')
    encoder_cpr = LaunchConfiguration('encoder_cpr')
    loop_rate = LaunchConfiguration('loop_rate')
    
    # Launch Arguments
    declare_serial_port = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for Arduino motor controller'
    )
    
    declare_baud_rate = DeclareLaunchArgument(
        'baud_rate',
        default_value='57600',
        description='Baud rate for Arduino communication'
    )
    
    declare_encoder_cpr = DeclareLaunchArgument(
        'encoder_cpr',
        default_value='1860',
        description='Encoder counts per revolution'
    )
    
    declare_loop_rate = DeclareLaunchArgument(
        'loop_rate',
        default_value='30',
        description='Arduino loop rate in Hz'
    )

    # Motor Driver Node - Communicates with Arduino
    motor_driver_node = Node(
        package='serial_motor_demo',
        executable='driver',
        name='motor_driver',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'baud_rate': baud_rate,
            'encoder_cpr': encoder_cpr,
            'loop_rate': loop_rate,
        }]
    )

    # Teleop Bridge Node - Converts cmd_vel to motor commands
    teleop_bridge_node = Node(
        package='serial_motor_demo',
        executable='teleop_bridge',
        name='teleop_bridge',
        output='screen',
        parameters=[{
            'wheel_separation': 0.297,
            'wheel_radius': 0.033,
            'max_linear_speed': 1.0,
            'max_angular_speed': 2.0,
        }]
    )

    # LiDAR Launch - Starts the LD19 LiDAR driver
    lidar_launch = ExecuteProcess(
        cmd=['ros2', 'launch', 'ldlidar_node', 'ldlidar_bringup.launch.py'],
        output='screen',
        name='lidar_launch'
    )

    # LiDAR Lifecycle Commands - Configure and activate LiDAR
    lidar_configure = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/ldlidar_node', 'configure'],
        output='screen',
        name='lidar_configure'
    )

    lidar_activate = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/ldlidar_node', 'activate'],
        output='screen', 
        name='lidar_activate'
    )

    # Event Handlers for Sequenced Startup
    # Configure LiDAR 3 seconds after launch starts
    configure_after_launch = RegisterEventHandler(
        OnProcessStart(
            target_action=lidar_launch,
            on_start=[
                TimerAction(
                    period=3.0,
                    actions=[lidar_configure]
                )
            ]
        )
    )

    # Activate LiDAR 2 seconds after configure
    activate_after_configure = TimerAction(
        period=5.0,  # Total 5 seconds (3 for launch + 2 for configure)
        actions=[lidar_activate]
    )

    # Build Launch Description
    ld = LaunchDescription()

    # Add arguments
    ld.add_action(declare_serial_port)
    ld.add_action(declare_baud_rate)
    ld.add_action(declare_encoder_cpr)
    ld.add_action(declare_loop_rate)
    
    # Add nodes
    ld.add_action(motor_driver_node)
    ld.add_action(teleop_bridge_node)
    ld.add_action(lidar_launch)
    
    # Add lifecycle management
    ld.add_action(configure_after_launch)
    ld.add_action(activate_after_configure)

    return ld
