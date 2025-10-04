#!/usr/bin/env python3
"""
Odometry Calibration Test - Advanced wheel parameter calibration
================================================================

This script helps you calibrate wheel parameters by:
1. Recording encoder values during known movements
2. Calculating actual wheel radius and separation
3. Identifying systematic errors in odometry

Usage:
    ros2 run pharma_bot odometry_calibration_test.py

Instructions:
1. Place robot at a known starting position
2. Drive robot in a straight line for exactly 1 meter
3. Record the results
4. Drive robot in a complete circle (360 degrees)
5. Record the results
"""

import rclpy
from rclpy.node import Node
import math
from serial_motor_demo_msgs.msg import EncoderVals
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class OdometryCalibrationTest(Node):
    def __init__(self):
        super().__init__('odometry_calibration_test')
        
        # Subscribers
        self.encoder_sub = self.create_subscription(
            EncoderVals, '/encoder_vals', self.encoder_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # Publisher for cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Data storage
        self.start_encoders = None
        self.current_encoders = None
        self.start_odom = None
        self.current_odom = None
        
        # Current parameters (from launch file)
        self.encoder_cpr = 1859
        self.wheel_radius = 0.02569  # 25.69mm
        self.wheel_separation = 0.160  # Decreased from 0.173 to reduce negative yaw drift
        
        self.get_logger().info("Odometry Calibration Test Started")
        self.get_logger().info("Commands:")
        self.get_logger().info("  's' - Set starting position")
        self.get_logger().info("  'l' - Test linear movement (straight line)")
        self.get_logger().info("  'r' - Test rotational movement (360° turn)")
        self.get_logger().info("  'c' - Calculate wheel parameters")
        self.get_logger().info("  'q' - Quit")
        
        # Timer for user input
        self.timer = self.create_timer(0.1, self.user_input_handler)
        
    def encoder_callback(self, msg):
        self.current_encoders = msg
        
    def odom_callback(self, msg):
        self.current_odom = msg
        
    def user_input_handler(self):
        # Non-blocking input check would be ideal, but for simplicity, we'll use a different approach
        pass
        
    def set_start_position(self):
        """Record starting encoder and odometry values"""
        if self.current_encoders and self.current_odom:
            self.start_encoders = self.current_encoders
            self.start_odom = self.current_odom
            self.get_logger().info(f"Start position set:")
            self.get_logger().info(f"  Left encoder: {self.start_encoders.mot_1_enc_val}")
            self.get_logger().info(f"  Right encoder: {self.start_encoders.mot_2_enc_val}")
            self.get_logger().info(f"  Odom position: ({self.start_odom.pose.pose.position.x:.3f}, {self.start_odom.pose.pose.position.y:.3f})")
        else:
            self.get_logger().warn("No encoder or odometry data available")
            
    def test_linear_movement(self, actual_distance=1.0):
        """Calculate wheel radius based on linear movement"""
        if not self.start_encoders or not self.current_encoders:
            self.get_logger().warn("No starting position set")
            return
            
        # Calculate encoder differences
        left_diff = self.current_encoders.mot_1_enc_val - self.start_encoders.mot_1_enc_val
        right_diff = self.current_encoders.mot_2_enc_val - self.start_encoders.mot_2_enc_val
        avg_diff = (left_diff + right_diff) / 2.0
        
        # Calculate odometry distance
        if self.start_odom and self.current_odom:
            dx = self.current_odom.pose.pose.position.x - self.start_odom.pose.pose.position.x
            dy = self.current_odom.pose.pose.position.y - self.start_odom.pose.pose.position.y
            odom_distance = math.sqrt(dx*dx + dy*dy)
        else:
            odom_distance = 0
            
        # Calculate wheel radius
        if avg_diff != 0:
            calculated_radius = actual_distance / (avg_diff / self.encoder_cpr * 2 * math.pi)
            radius_error = (calculated_radius - self.wheel_radius) / self.wheel_radius * 100
            
            self.get_logger().info(f"Linear Movement Test Results:")
            self.get_logger().info(f"  Actual distance: {actual_distance:.3f} m")
            self.get_logger().info(f"  Odometry distance: {odom_distance:.3f} m")
            self.get_logger().info(f"  Encoder diff (L,R,Avg): {left_diff}, {right_diff}, {avg_diff:.1f}")
            self.get_logger().info(f"  Current wheel radius: {self.wheel_radius:.5f} m")
            self.get_logger().info(f"  Calculated radius: {calculated_radius:.5f} m")
            self.get_logger().info(f"  Radius error: {radius_error:.2f}%")
            
            if abs(radius_error) > 5:
                self.get_logger().warn(f"Large radius error detected! Consider updating wheel_radius to {calculated_radius:.5f}")
        else:
            self.get_logger().warn("No encoder movement detected")
            
    def test_rotational_movement(self, actual_angle=360.0):
        """Calculate wheel separation based on rotational movement"""
        if not self.start_encoders or not self.current_encoders:
            self.get_logger().warn("No starting position set")
            return
            
        # Calculate encoder differences
        left_diff = self.current_encoders.mot_1_enc_val - self.start_encoders.mot_1_enc_val
        right_diff = self.current_encoders.mot_2_enc_val - self.start_encoders.mot_2_enc_val
        
        # For rotation, wheels should move in opposite directions
        wheel_diff = abs(left_diff - right_diff) / 2.0
        
        # Calculate wheel separation
        if wheel_diff != 0:
            # Distance traveled by each wheel during rotation
            wheel_distance = wheel_diff / self.encoder_cpr * 2 * math.pi * self.wheel_radius
            # Calculate separation: arc_length = radius * angle
            calculated_separation = wheel_distance / (math.radians(actual_angle))
            separation_error = (calculated_separation - self.wheel_separation) / self.wheel_separation * 100
            
            self.get_logger().info(f"Rotational Movement Test Results:")
            self.get_logger().info(f"  Actual angle: {actual_angle:.1f}°")
            self.get_logger().info(f"  Encoder diff (L,R): {left_diff}, {right_diff}")
            self.get_logger().info(f"  Wheel differential: {wheel_diff:.1f}")
            self.get_logger().info(f"  Current separation: {self.wheel_separation:.3f} m")
            self.get_logger().info(f"  Calculated separation: {calculated_separation:.3f} m")
            self.get_logger().info(f"  Separation error: {separation_error:.2f}%")
            
            if abs(separation_error) > 5:
                self.get_logger().warn(f"Large separation error! Consider updating wheel_separation to {calculated_separation:.3f}")
        else:
            self.get_logger().warn("No rotational movement detected")


def main(args=None):
    rclpy.init(args=args)
    node = OdometryCalibrationTest()
    
    print("\nOdometry Calibration Test")
    print("========================")
    print("Commands:")
    print("  s - Set starting position")
    print("  l - Test linear movement (drive 1m straight, then press 'l')")
    print("  r - Test rotational movement (turn 360°, then press 'r')")
    print("  q - Quit")
    
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            
            # Simple input handling
            try:
                cmd = input().strip().lower()
                if cmd == 's':
                    node.set_start_position()
                elif cmd == 'l':
                    distance = float(input("Enter actual distance traveled (meters): ") or "1.0")
                    node.test_linear_movement(distance)
                elif cmd == 'r':
                    angle = float(input("Enter actual angle turned (degrees): ") or "360.0")
                    node.test_rotational_movement(angle)
                elif cmd == 'q':
                    break
                else:
                    print("Unknown command")
            except (EOFError, KeyboardInterrupt):
                break
            except ValueError:
                print("Invalid number entered")
                
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
