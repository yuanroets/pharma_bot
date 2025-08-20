#!/usr/bin/env python3
"""
Complete Hardware Launch File for Pi
===================================

This launch file starts ALL hardware components on the Pi:
- Motor driver (connects to Arduino)
- Teleop bridge (converts cmd_vel to motor commands)  
- LD19 LiDAR driver (reads laser data)
- LiDAR lifecycle management (auto-activates the LiDAR)

Usage on Pi:
    cd /home/ubuntu/dev_ws
    source install/setup.bash
    ros2 launch pharma_bot pi_hardware.launch.py

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
    lidar_port = LaunchConfiguration('lidar_port')
    
    # Launch Arguments
    declare_serial_port = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',  # Default Arduino port
        description='Serial port for Arduino motor controller'
    )
    
    declare_lidar_port = DeclareLaunchArgument(
        'lidar_port', 
        default_value='/dev/ttyAMA0',  # Default LiDAR port (GPIO UART)
        description='Serial port for LD19 LiDAR'
    )

    # Motor Driver Node
    # Communicates with Arduino for robot movement control
    motor_driver_node = Node(
        package='serial_motor_demo',
        executable='driver',
        name='motor_driver',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            # Add any other motor driver parameters here
        }]
    )

    # Teleop Bridge Node  
    # Converts /cmd_vel (from keyboard/navigation) to /motor_command (for Arduino)
    teleop_bridge_node = Node(
        package='serial_motor_demo',
        executable='teleop_bridge',
        name='teleop_bridge',
        output='screen',
        # This node subscribes to /cmd_vel and publishes /motor_command
    )

    # LD19 LiDAR Launch
    # Note: Using ExecuteProcess to launch the LiDAR launch file
    # This starts the LiDAR driver and robot state publisher
    lidar_launch = ExecuteProcess(
        cmd=['ros2', 'launch', 'ldlidar_node', 'ldlidar_bringup.launch.py'],
        output='screen',
        name='lidar_launch'
    )

    # LiDAR Lifecycle Configuration Command
    # This configures the LiDAR node (must run after LiDAR starts)
    lidar_configure = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/ldlidar_node', 'configure'],
        output='screen',
        name='lidar_configure'
    )

    # LiDAR Lifecycle Activation Command  
    # This activates the LiDAR to start publishing scan data
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
                    period=3.0,  # Wait 3 seconds for LiDAR to start
                    actions=[lidar_configure]
                )
            ]
        )
    )

    # Activate LiDAR 1 second after configure
    activate_after_configure = TimerAction(
        period=5.0,  # Wait 5 seconds total (3 + 2 for configure)
        actions=[lidar_activate]
    )

    # Build Launch Description
    ld = LaunchDescription()

    # Add arguments
    ld.add_action(declare_serial_port)
    ld.add_action(declare_lidar_port)
    
    # Add nodes in startup order
    ld.add_action(motor_driver_node)
    ld.add_action(teleop_bridge_node) 
    ld.add_action(lidar_launch)
    
    # Add lifecycle management
    ld.add_action(configure_after_launch)
    ld.add_action(activate_after_configure)

    return ld
