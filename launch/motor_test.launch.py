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
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        
        DeclareLaunchArgument(
            'i2c_address',
            default_value='0x40',
            description='I2C address for PCA9685 motor driver'),
        
        # Motor Encoder Test Node
        Node(
            package='pharma_bot',
            executable='motor_encoder_test.py',
            name='motor_encoder_test',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'i2c_address': i2c_address
            }]
        )
    ])
