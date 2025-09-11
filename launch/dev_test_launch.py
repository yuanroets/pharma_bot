#!/usr/bin/env python3
"""
Dev Machine Test Launch File - MINIMAL TELEOP ONLY
==================================================

This launch file starts ONLY the teleop keyboard control on the dev machine.

Usage on dev machine:
    ros2 launch pharma_bot dev_test_launch.py

This is the minimal working setup for keyboard motor control testing.
Make sure to set ROS_DOMAIN_ID=30 to match the Pi.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch Configuration Variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Launch Arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time for real robot'
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

    # Delayed start for teleop (gives time for Pi to initialize)
    delayed_teleop = TimerAction(
        period=2.0,
        actions=[teleop_keyboard]
    )

    # Build Launch Description  
    ld = LaunchDescription()

    # Add arguments
    ld.add_action(declare_use_sim_time)
    
    # Add teleop ONLY
    ld.add_action(delayed_teleop)

    return ld
