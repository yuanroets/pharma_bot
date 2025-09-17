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
from serial_motor_demo_msgs.msg import EncoderVals
from sensor_msgs.msg import JointState
import math


class JointStateBridge(Node):
    def __init__(self):
        super().__init__('joint_state_bridge')
        
        # Publishers
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Subscribers
        self.encoder_sub = self.create_subscription(
            EncoderVals, '/encoder_vals', self.encoder_callback, 10)
        
        # State variables
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0
        self.encoder_cpr = 1859.0  # Encoder counts per revolution (measured: L=1866, R=1851, avg=1859)
        
        # Previous encoder values for position calculation
        self.prev_left_enc = 0
        self.prev_right_enc = 0
        self.left_cumulative_pos = 0.0
        self.right_cumulative_pos = 0.0
        
        # Timer for publishing joint states
        self.timer = self.create_timer(0.1, self.publish_joint_states)  # 10Hz
        
        self.get_logger().info('Joint State Bridge node initialized - using encoder_vals topic')

    def encoder_callback(self, msg):
        """Convert encoder counts to wheel positions in radians"""
        # Calculate change in encoder counts
        left_delta = msg.mot_1_enc_val - self.prev_left_enc
        right_delta = msg.mot_2_enc_val - self.prev_right_enc
        
        # Handle encoder rollover (if applicable)
        # For now, assume no rollover issues
        
        # Update cumulative positions
        self.left_cumulative_pos += (left_delta / self.encoder_cpr) * 2.0 * math.pi
        self.right_cumulative_pos += (right_delta / self.encoder_cpr) * 2.0 * math.pi
        
        # Store current encoder values
        self.prev_left_enc = msg.mot_1_enc_val
        self.prev_right_enc = msg.mot_2_enc_val
        
        # Update wheel positions
        self.left_wheel_pos = self.left_cumulative_pos
        self.right_wheel_pos = self.right_cumulative_pos

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
