#!/usr/bin/env python3
"""
Joint State Bridge Node - MOTOR ENCODER TO JOINT STATES
=======================================================

This node bridges motor encoder data to ROS joint states for wheel visualization.
It subscribes to motor encoder data and publishes joint states for the wheels.

Subscribes to:
    /motor_encoder_left  (std_msgs/Float64)
    /motor_encoder_right (std_msgs/Float64)

Publishes to:
    /joint_states (sensor_msgs/JointState)

Usage:
    ros2 run pharma_bot joint_state_bridge

Or include in a launch file:
    Node(
        package='pharma_bot',
        executable='joint_state_bridge',
        name='joint_state_bridge',
        output='screen'
    )
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import math


class JointStateBridge(Node):
    def __init__(self):
        super().__init__('joint_state_bridge')
        
        # Publishers
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Subscribers
        self.left_encoder_sub = self.create_subscription(
            Float64, '/motor_encoder_left', self.left_encoder_callback, 10)
        self.right_encoder_sub = self.create_subscription(
            Float64, '/motor_encoder_right', self.right_encoder_callback, 10)
        
        # State variables
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0
        self.encoder_cpr = 1860.0  # Encoder counts per revolution
        
        # Timer for publishing joint states
        self.timer = self.create_timer(0.1, self.publish_joint_states)  # 10Hz
        
        self.get_logger().info('Joint State Bridge node initialized')

    def left_encoder_callback(self, msg):
        """Convert left encoder count to wheel position in radians"""
        # Convert encoder counts to radians
        self.left_wheel_pos = (msg.data / self.encoder_cpr) * 2.0 * math.pi

    def right_encoder_callback(self, msg):
        """Convert right encoder count to wheel position in radians"""
        # Convert encoder counts to radians
        self.right_wheel_pos = (msg.data / self.encoder_cpr) * 2.0 * math.pi

    def publish_joint_states(self):
        """Publish joint states for wheel visualization"""
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = ""
        
        # Joint names matching URDF
        joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
        
        # Joint positions (in radians)
        joint_state.position = [self.left_wheel_pos, self.right_wheel_pos]
        
        # Joint velocities (optional, set to 0 for now)
        joint_state.velocity = [0.0, 0.0]
        
        # Joint efforts (optional, set to 0 for now)  
        joint_state.effort = [0.0, 0.0]
        
        self.joint_state_pub.publish(joint_state)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
