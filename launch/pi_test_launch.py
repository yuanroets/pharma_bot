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
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    # Package directory
    pkg_pharma_bot = get_package_share_directory('pharma_bot')
    
    # URDF file path
    robot_description_file = os.path.join(pkg_pharma_bot, 'description', 'robot.urdf.xacro')
    
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
            'wheel_separation': 0.115,  # 115mm actual wheel separation (matches URDF)
            'wheel_radius': 0.025,      # 25mm radius (real hardware)
            'max_linear_speed': 1.0,
            'max_angular_speed': 2.0,
        }]
    )

    # Joint State Bridge - Converts motor encoder data to joint states for wheel visualization
    joint_state_bridge = Node(
        package='pharma_bot',
        executable='joint_state_bridge',
        name='joint_state_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }]
    )

    # Simple Odometry Node - Converts encoder data to odom->base_link transform for SLAM
    simple_odometry = Node(
        package='serial_motor_demo',
        executable='simple_odometry',
        name='simple_odometry',
        output='screen',
        parameters=[{
            'encoder_cpr': encoder_cpr,
            'wheel_separation': 0.166,  # CALIBRATION TEST: Increased from 0.115 to fix rotation accuracy
            'wheel_radius': 0.025,      # 25mm radius (real hardware)
        }]
    )

    # LiDAR Launch - Uses official ldlidar_bringup (with robot_state_publisher commented out)
    # Note: All TF transforms now come from our main robot URDF
    lidar_launch = ExecuteProcess(
        cmd=['ros2', 'launch', 'ldlidar_node', 'ldlidar_bringup.launch.py'],
        output='screen',
        name='lidar_launch'
    )

    # LiDAR Lifecycle Manager - Robust lifecycle management
    lidar_lifecycle_manager = Node(
        package='pharma_bot',
        executable='lidar_lifecycle_manager',
        name='lidar_lifecycle_manager',
        output='screen'
    )

    # Delayed start for lifecycle manager (give LiDAR node time to start)
    delayed_lifecycle_manager = TimerAction(
        period=3.0,
        actions=[lidar_lifecycle_manager]
    )

    # Static Transform: odom -> base_link (robot position in odometry frame)
    # NOTE: DISABLED for SLAM - SLAM will handle odom->base_link transform
    # static_tf_odom_to_base = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_tf_odom_to_base',
    #     arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    # )

    # Build Launch Description
    ld = LaunchDescription()

    # Add arguments
    ld.add_action(declare_serial_port)
    ld.add_action(declare_baud_rate)
    ld.add_action(declare_encoder_cpr)
    ld.add_action(declare_loop_rate)
    
    # Add robot components - Pi handles sensors and joint states only
    ld.add_action(joint_state_bridge)  # Real encoder data → joint states
    ld.add_action(simple_odometry)     # Real encoder data → odometry for SLAM
    # ld.add_action(static_tf_odom_to_base)  # DISABLED - SLAM handles odom->base_link
    
    # Add motor nodes
    ld.add_action(motor_driver_node)
    ld.add_action(teleop_bridge_node)
    
        # Add LiDAR components
    ld.add_action(lidar_launch)
    ld.add_action(delayed_lifecycle_manager)

    return ld
