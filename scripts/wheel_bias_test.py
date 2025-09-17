#!/usr/bin/env python3
"""
Wheel Bias Test - Detect systematic wheel speed differences
===========================================================

This script monitors encoder values during straight-line movement
to detect if one wheel consistently turns faster than the other.

Usage:
    ros2 run pharma_bot wheel_bias_test.py

Instructions:
1. Run this script
2. Use teleop to drive straight forward for several seconds
3. Stop and check the results
"""

import rclpy
from rclpy.node import Node
from serial_motor_demo_msgs.msg import EncoderVals
import math


class WheelBiasTest(Node):
    def __init__(self):
        super().__init__('wheel_bias_test')
        
        # Subscribe to encoder data
        self.encoder_sub = self.create_subscription(
            EncoderVals, '/encoder_vals', self.encoder_callback, 10)
        
        # Data storage
        self.start_left = None
        self.start_right = None
        self.samples = []
        self.recording = False
        
        # Timer for periodic analysis
        self.timer = self.create_timer(1.0, self.analyze_bias)
        
        self.get_logger().info("Wheel Bias Test Started")
        self.get_logger().info("Drive straight forward to collect data...")
        
    def encoder_callback(self, msg):
        if self.start_left is None:
            self.start_left = msg.mot_1_enc_val
            self.start_right = msg.mot_2_enc_val
            self.get_logger().info(f"Starting values - Left: {self.start_left}, Right: {self.start_right}")
            return
            
        # Calculate differences from start
        left_diff = msg.mot_1_enc_val - self.start_left
        right_diff = msg.mot_2_enc_val - self.start_right
        
        # Only record if wheels are moving
        if abs(left_diff) > 10 and abs(right_diff) > 10:
            self.samples.append({
                'left_diff': left_diff,
                'right_diff': right_diff,
                'bias': left_diff - right_diff
            })
            
    def analyze_bias(self):
        if len(self.samples) < 5:
            return
            
        # Calculate average bias over recent samples
        recent_samples = self.samples[-10:]  # Last 10 samples
        
        avg_left = sum(s['left_diff'] for s in recent_samples) / len(recent_samples)
        avg_right = sum(s['right_diff'] for s in recent_samples) / len(recent_samples)
        avg_bias = sum(s['bias'] for s in recent_samples) / len(recent_samples)
        
        # Calculate bias percentage
        if avg_right != 0:
            bias_percent = (avg_bias / avg_right) * 100
        else:
            bias_percent = 0
            
        self.get_logger().info(f"Wheel Analysis:")
        self.get_logger().info(f"  Left wheel:  {avg_left:.1f} counts")
        self.get_logger().info(f"  Right wheel: {avg_right:.1f} counts")
        self.get_logger().info(f"  Bias: {avg_bias:.1f} counts ({bias_percent:.2f}%)")
        
        if abs(bias_percent) > 1.0:
            if avg_bias > 0:
                self.get_logger().warn("LEFT wheel turning faster - robot curves RIGHT")
            else:
                self.get_logger().warn("RIGHT wheel turning faster - robot curves LEFT")
                
            # Suggest correction factor
            correction_factor = avg_right / avg_left if avg_left != 0 else 1.0
            self.get_logger().info(f"Suggested left wheel correction factor: {correction_factor:.4f}")


def main(args=None):
    rclpy.init(args=args)
    node = WheelBiasTest()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
