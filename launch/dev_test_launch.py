#!/usr/bin/env python3
"""
Dev Machine Test Launch File - TELEOP + SLAM VISUALIZATION
==========================================================

This launch file starts teleop + SLAM mapping on the dev machine:
- Teleop keyboard control
- Robot state publisher (URDF)
- SLAM Toolbox with proper topic remapping
- Static transforms (coordinate frame linking)
- RViz2 with robot model, LiDAR data, and SLAM visualization

Usage on dev machine:
    ros2 launch pharma_bot dev_test_launch.py

Make sure to set ROS_DOMAIN_ID=30 to match the Pi.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node, LifecycleNode
from launch_ros.parameter_descriptions import ParameterValue


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
    
    # Joint State Publisher - DISABLED (Pi publishes joint states from encoders)
    # joint_state_publisher = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time,
    #     }]
    # )

    # CRITICAL TRANSFORMS for coordinate frame chain (WITHOUT SLAM)
    # Complete chain: odom → base_link → chassis → ldlidar_base → ldlidar_link
    # NOTE: Use 'odom' as Fixed Frame in RViz
    
    # Static Transform: odom -> base_link (robot position in odometry frame)
    # NOTE: DISABLED for SLAM - SLAM toolbox will publish odom->base_link transform
    # static_tf_odom_to_base = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_tf_odom_to_base',
    #     arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    # )
    
    # Static Transforms: base_link -> wheel frames (for wheel visualization)
    # Note: These should come from robot_state_publisher, but adding as backup
    static_tf_base_to_left_wheel = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_left_wheel',
        arguments=['0', '0.052', '0.0', '0', '0', '0', 'base_link', 'left_wheel']  # Updated to actual dimensions
    )
    
    static_tf_base_to_right_wheel = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_right_wheel',
        arguments=['0', '-0.052', '0.0', '0', '0', '0', 'base_link', 'right_wheel']  # Updated to actual dimensions
    )
    
    # NOTE: base_link → chassis → ldlidar_base transforms are provided by robot_state_publisher
    # NOTE: ldlidar_base → ldlidar_link transform is provided by Pi

    # Lifecycle manager for SLAM Toolbox (same approach as ldlidar vendors)
    slam_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[
            '/home/ubuntu/dev_ws/src/pharma_bot/config/lifecycle_mgr_slam.yaml'
        ]
    )

    # SLAM Toolbox Node - Maps environment using LiDAR data (EXACT vendor approach)
    slam_toolbox_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        namespace='',
        name='slam_toolbox',
        output='screen',
        parameters=[
            '/home/ubuntu/dev_ws/src/pharma_bot/config/mapper_params_online_async.yaml'
        ],
        remappings=[
            ('/scan', '/ldlidar_node/scan')  # Remap from standard /scan to actual LiDAR topic
        ]          
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
    ld.add_action(robot_state_publisher)  # Dev machine publishes URDF
    # ld.add_action(joint_state_publisher)  # DISABLED - Pi publishes joint states
    # ld.add_action(static_tf_odom_to_base)        # DISABLED - SLAM handles odom → base_link
    ld.add_action(static_tf_base_to_left_wheel)  # Backup wheel transforms
    ld.add_action(static_tf_base_to_right_wheel) # Backup wheel transforms
    ld.add_action(slam_lifecycle_manager)        # Lifecycle manager for SLAM (vendor approach)
    ld.add_action(slam_toolbox_node)             # SLAM mapping with topic remapping
    ld.add_action(rviz2)
    
    # Add delayed teleop
    ld.add_action(delayed_teleop)

    return ld
