#!/usr/bin/env python3
"""
Pi Test Launch File - MOTOR CONTROL + LIDAR
===========================================

This launch file starts motor control + LiDAR components on the Pi:
- Motor driver (communicates with Arduino via USB)
- Teleop bridge (converts cmd_vel to motor commands)
- LiDAR driver with lifecycle management (/dev/ttyAMA0)

Usage on Pi:
    ros2 launch pharma_bot pi_test_launch.py

This is the working setup for keyboard motor control + LiDAR testing.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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

    # Motor to Joint State Bridge - Converts encoder data to joint states for RViz
    # Find the workspace source directory and run the Python script directly
    pharma_bot_share = get_package_share_directory('pharma_bot')
    # Navigate from install/pharma_bot/share/pharma_bot back to src/pharma_bot/pharma_bot
    workspace_src = os.path.join(os.path.dirname(pharma_bot_share), '..', '..', '..', 'src')
    bridge_script = os.path.join(workspace_src, 'pharma_bot', 'pharma_bot', 'motor_to_joint_state_bridge.py')
    
    joint_state_bridge = ExecuteProcess(
        cmd=['python3', bridge_script],
        output='screen',
        name='motor_to_joint_state_bridge'
    )

    # LiDAR Component Node - Direct launch without robot_state_publisher conflict
    lidar_node = Node(
        package='ldlidar_component',
        executable='ldlidar_component',
        name='ldlidar_node',
        output='screen',
        parameters=[{
            'product_name': 'LDLiDAR_LD19',
            'topic_name': 'scan',
            'frame_id': 'ldlidar_link',
            'port_name': '/dev/ttyAMA0',
            'port_baudrate': 230400,
            'laser_scan_dir': True,
            'enable_angle_crop_func': False,
            'angle_crop_min': 135.0,
            'angle_crop_max': 225.0
        }]
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

    # Event Handlers for Sequenced LiDAR Startup
    # Configure LiDAR 3 seconds after node starts
    configure_after_launch = TimerAction(
        period=3.0,
        actions=[lidar_configure]
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
    
    # Add motor nodes
    ld.add_action(motor_driver_node)
    ld.add_action(teleop_bridge_node)
    ld.add_action(joint_state_bridge)          # Dynamic joint states for wheel rotation
    
    # Add LiDAR components
    ld.add_action(lidar_node)
    ld.add_action(configure_after_launch)
    ld.add_action(activate_after_configure)

    return ld
