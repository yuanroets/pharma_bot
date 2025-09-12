#!/usr/bin/env python3
"""
SLAM Launch for Real Robot - MAPPING MODE
=========================================

This launch file starts SLAM mapping on the real robot using:
- SLAM Toolbox (async mode)
- RViz2 with SLAM configuration
- All necessary transforms

Usage:
On Pi:    ros2 launch pharma_bot pi_test_launch.py
On Dev:   ros2 launch pharma_bot slam_real.launch.py

Drive around with teleop to build map, then save with:
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'my_map'}}"
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package Directories
    pkg_pharma_bot = get_package_share_directory('pharma_bot')
    
    # File paths
    robot_description_file = os.path.join(pkg_pharma_bot, 'description', 'robot.urdf.xacro')
    slam_config_file = os.path.join(pkg_pharma_bot, 'config', 'mapper_params_online_async.yaml')
    rviz_config_file = os.path.join(pkg_pharma_bot, 'config', 'slam.rviz')
    
    # Launch Configuration Variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Launch Arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time (false for real robot)'
    )

    # Robot State Publisher - Publishes robot URDF for visualization
    robot_description_config = Command(['xacro ', robot_description_file])
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description_config, value_type=str),
            'use_sim_time': use_sim_time,
        }]
    )

    # SLAM Toolbox - Async mapping mode
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config_file,
            {
                'use_sim_time': use_sim_time,
                'base_frame': 'base_link',              # Real robot base frame
                'scan_topic': '/ldlidar_node/scan',     # Real LiDAR topic
                'mode': 'mapping',                      # Mapping mode (not localization)
                'map_file_name': '/tmp/my_map',         # Where to save maps
                'map_start_at_dock': False,             # Start mapping from current pose
            }
        ]
    )

    # Teleop Keyboard Control
    teleop_keyboard = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='gnome-terminal --',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        remappings=[('/cmd_vel', '/cmd_vel')]
    )

    # RViz2 for SLAM visualization
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )

    # Build Launch Description
    ld = LaunchDescription()

    # Add arguments
    ld.add_action(declare_use_sim_time)
    
    # Add nodes
    ld.add_action(robot_state_publisher)
    ld.add_action(slam_toolbox)
    ld.add_action(teleop_keyboard)
    ld.add_action(rviz2)

    return ld
