#!/usr/bin/env python3
"""
Pharma Bot Navigation Launch File
================================

This launch file starts Nav2 navigation with proper topic remapping for the pharma_bot.
Uses a relay node to remap /ldlidar_node/scan to /scan for Nav2 compatibility.

Usage:
    ros2 launch pharma_bot pharma_bot_navigation_launch.py use_sim_time:=false map_subscribe_transient_local:=true

Prerequisites:
    - Robot hardware running (odometry, lidar)
    - Localization already running (map_server + AMCL)
    - Initial pose set in RViz
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Package Directories
    pkg_pharma_bot = get_package_share_directory('pharma_bot')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    
    # Parameters file
    nav2_params_file = os.path.join(pkg_pharma_bot, 'config', 'nav2_params.yaml')
    
    # Launch Configuration Variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local')
    
    # Launch Arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params_file,
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', 
        default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', 
        default_value='True',
        description='Whether to use composed bringup')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', 
        default_value='False',
        description='Whether to respawn if a node crashes')

    declare_map_subscribe_transient_local_cmd = DeclareLaunchArgument(
        'map_subscribe_transient_local',
        default_value='true',
        description='Whether to use transient local QoS for map topic')

    # Relay node to remap scan topic from /ldlidar_node/scan to /scan
    scan_relay_node = Node(
        package='topic_tools',
        executable='relay',
        name='scan_relay',
        arguments=['/ldlidar_node/scan', '/scan'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Include the Nav2 navigation launch
    nav2_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_pharma_bot, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': use_composition,
            'use_respawn': use_respawn,
            'map_subscribe_transient_local': map_subscribe_transient_local,
        }.items()
    )

    # Build Launch Description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_map_subscribe_transient_local_cmd)

    # Add relay node first (so /scan topic exists when nav2 starts)
    ld.add_action(scan_relay_node)
    
    # Add Nav2 navigation
    ld.add_action(nav2_navigation_launch)

    return ld