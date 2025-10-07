#!/usr/bin/env python3
"""
Dev Machine LiDAR-Heavy SLAM + Navigation Launch File
=====================================================

This launch file starts LiDAR-optimized SLAM + Nav2 navigation on the dev machine:
- Teleop keyboard control  
- Robot state publisher (URDF)
- SLAM Toolbox MAPPING mode (LiDAR-heavy, dynamic obstacle oriented)
- Nav2 navigation stack with optimized costmaps
- Static transforms (coordinate frame linking)
- RViz2 with robot model, LiDAR data, and navigation visualization

Key optimizations:
- Very frequent SLAM updates for dynamic obstacles
- High LiDAR reliance vs odometry
- Faster costmap updates for reactive obstacle avoidance
- Conservative navigation for safety around new obstacles

Usage on dev machine:
    ros2 launch pharma_bot dev_test_lidar_slam_launch.py

Make sure to set ROS_DOMAIN_ID=30 to match the Pi.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node, LifecycleNode
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Package Directories  
    pkg_pharma_bot = get_package_share_directory('pharma_bot')
    
    # URDF File Path
    robot_description_file = os.path.join(pkg_pharma_bot, 'description', 'robot.urdf.xacro')
    
    # RViz Configuration
    rviz_config_file = os.path.join(pkg_pharma_bot, 'config', 'pharma_bot.rviz')
    
    # SLAM Configuration (optimized for LiDAR)
    slam_params_file = os.path.join(pkg_pharma_bot, 'config', 'mapper_params_online_async.yaml')
    
    # Launch Configuration Variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    encoder_cpr = LaunchConfiguration('encoder_cpr')
    
    # Launch Arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time for real robot'
    )

    declare_encoder_cpr = DeclareLaunchArgument(
        'encoder_cpr',
        default_value='1859',
        description='Encoder counts per revolution (measured: L=1866, R=1851, avg=1859)'
    )

    # Robot State Publisher - Loads and publishes robot URDF
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
    
    # Static Transforms: base_link -> wheel frames (for wheel visualization)
    static_tf_base_to_left_wheel = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_left_wheel',
        arguments=['0', '0.0865', '0.0', '0', '0', '1.5708', 'base_link', 'left_wheel']
    )
    
    static_tf_base_to_right_wheel = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_right_wheel',
        arguments=['0', '-0.0865', '0.0', '0', '0', '-1.5708', 'base_link', 'right_wheel']
    )
    
    # Simple Odometry Node (with optimized parameters)
    simple_odometry = Node(
        package='serial_motor_demo',
        executable='simple_odometry',
        name='simple_odometry',
        output='screen',
        parameters=[{
            'encoder_cpr': encoder_cpr,
            'wheel_separation': 0.170,
            'wheel_radius': 0.02569,    # 25.69mm radius (calibrated from 1m test)
        }]
    )

    # SLAM Toolbox - Async mapping mode with LiDAR-heavy configuration
    async_slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {
            'use_sim_time': use_sim_time
        }],
        remappings=[('/scan', '/scan')]
    )

    # Teleop Keyboard Control - For driving the robot
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

    # Relay node for cmd_vel_smoothed to cmd_vel (if using velocity smoother)
    relay_cmd_vel = Node(
        package='topic_tools',
        executable='relay',
        name='relay_cmd_vel_smoothed',
        arguments=['/cmd_vel_smoothed', '/cmd_vel'],
        output='screen'
    )

    # Relay node for /ldlidar_node/scan to /scan
    scan_relay_node = Node(
        package='topic_tools',
        executable='relay',
        name='scan_relay',
        arguments=['/ldlidar_node/scan', '/scan'],
        output='screen'
    )

    # Include Nav2 navigation launch (with optimized costmaps)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_pharma_bot, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'autostart': 'true',
            'map_subscribe_transient_local': 'true'
        }.items()
    )

    # Build Launch Description  
    ld = LaunchDescription()

    # Add arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_encoder_cpr)
    
    # Add core components (start immediately)
    ld.add_action(robot_state_publisher)
    ld.add_action(simple_odometry)
    ld.add_action(static_tf_base_to_left_wheel)
    ld.add_action(static_tf_base_to_right_wheel)
    ld.add_action(scan_relay_node)
    ld.add_action(async_slam_toolbox_node)
    ld.add_action(rviz2)
    ld.add_action(relay_cmd_vel)
    ld.add_action(nav2_launch)
    
    # Add delayed teleop
    ld.add_action(delayed_teleop)

    return ld
