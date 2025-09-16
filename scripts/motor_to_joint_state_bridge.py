#!/usr/bin/env python3
"""
Motor to Joint State Bridge
===========================

Converts serial_motor_demo encoder data to sensor_msgs/JointState messages
for proper wheel visualization in RViz.

This replaces the static joint_state_publisher with dynamic wheel rotation
based on actual motor encoder feedback.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from serial_motor_demo_msgs.msg import EncoderVals
import math


class MotorToJointStateBridge(Node):
    def __init__(self):
        super().__init__('motor_to_joint_state_bridge')
        
        # Parameters
        self.declare_parameter('encoder_cpr', 1860)  # Encoder counts per revolution
        self.declare_parameter('wheel_joint_names', ['left_wheel_joint', 'right_wheel_joint'])
        
        self.encoder_cpr = self.get_parameter('encoder_cpr').get_parameter_value().integer_value
        self.wheel_joint_names = self.get_parameter('wheel_joint_names').get_parameter_value().string_array_value
        
        # Initialize wheel positions (cumulative)
        self.left_wheel_position = 0.0
        self.right_wheel_position = 0.0
        self.last_left_encoder = 0
        self.last_right_encoder = 0
        self.encoder_initialized = False
        
        # Publishers and subscribers
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.encoder_sub = self.create_subscription(
            EncoderVals,
            '/motor_driver/encoder_vals',
            self.encoder_callback,
            10
        )
        
        self.get_logger().info(f'Motor to Joint State Bridge started')
        self.get_logger().info(f'Encoder CPR: {self.encoder_cpr}')
        self.get_logger().info(f'Joint names: {self.wheel_joint_names}')

    def encoder_callback(self, msg):
        """Convert encoder values to joint state positions"""
        
        # Initialize encoder values on first message
        if not self.encoder_initialized:
            self.last_left_encoder = msg.mot_1_enc_val
            self.last_right_encoder = msg.mot_2_enc_val
            self.encoder_initialized = True
            return
        
        # Calculate encoder deltas (handle potential rollover)
        left_delta = msg.mot_1_enc_val - self.last_left_encoder
        right_delta = msg.mot_2_enc_val - self.last_right_encoder
        
        # Handle encoder rollover (if using 32-bit signed integers)
        if left_delta > 2**30:
            left_delta -= 2**32
        elif left_delta < -2**30:
            left_delta += 2**32
            
        if right_delta > 2**30:
            right_delta -= 2**32
        elif right_delta < -2**30:
            right_delta += 2**32
        
        # Convert encoder counts to radians and accumulate
        left_position_delta = (left_delta / self.encoder_cpr) * 2 * math.pi
        right_position_delta = (right_delta / self.encoder_cpr) * 2 * math.pi
        
        self.left_wheel_position += left_position_delta
        self.right_wheel_position += right_position_delta
        
        # Update last encoder values
        self.last_left_encoder = msg.mot_1_enc_val
        self.last_right_encoder = msg.mot_2_enc_val
        
        # Create and publish joint state message
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.wheel_joint_names
        joint_state.position = [self.left_wheel_position, self.right_wheel_position]
        joint_state.velocity = []  # We could calculate this if needed
        joint_state.effort = []    # Not applicable for our setup
        
        self.joint_state_pub.publish(joint_state)


def main(args=None):
    rclpy.init(args=args)
    bridge = MotorToJointStateBridge()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
