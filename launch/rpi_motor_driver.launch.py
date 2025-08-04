import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    i2c_address = LaunchConfiguration('i2c_address')
    encoder_cpr = LaunchConfiguration('encoder_cpr')
    loop_rate = LaunchConfiguration('loop_rate')
    wheel_separation = LaunchConfiguration('wheel_separation')
    wheel_radius = LaunchConfiguration('wheel_radius')
    debug_mode = LaunchConfiguration('debug_mode')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        
        DeclareLaunchArgument(
            'i2c_address',
            default_value='0x40',
            description='I2C address for PCA9685 motor driver'),
        
        DeclareLaunchArgument(
            'encoder_cpr',
            default_value='1440',
            description='Encoder counts per revolution (determine experimentally)'),
        
        DeclareLaunchArgument(
            'loop_rate',
            default_value='20',
            description='PID update rate in Hz'),
        
        DeclareLaunchArgument(
            'wheel_separation',
            default_value='0.17',
            description='Distance between wheels in meters'),
        
        DeclareLaunchArgument(
            'wheel_radius',
            default_value='0.033',
            description='Wheel radius in meters'),
        
        DeclareLaunchArgument(
            'debug_mode',
            default_value='true',
            description='Enable debug logging'),
        
        # RPI Motor Driver Node
        Node(
            package='pharma_bot',
            executable='rpi_motor_driver.py',
            name='rpi_motor_driver',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'i2c_address': i2c_address,
                'encoder_cpr': encoder_cpr,
                'loop_rate': loop_rate,
                'wheel_separation': wheel_separation,
                'wheel_radius': wheel_radius,
                'debug_mode': debug_mode
            }],
            remappings=[
                ('/cmd_vel', '/cmd_vel')
            ]
        )
    ])
