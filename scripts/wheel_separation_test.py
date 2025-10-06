#!/usr/bin/env python3
"""
Wheel Separation Test - Test rotation accuracy for wheel separation calibration
==============================================================================

This script helps you test wheel separation by monitoring odometry during rotations.

Usage:
    ros2 run pharma_bot wheel_separation_test.py

Instructions:
1. Run this script
2. Press 's' to set starting position
3. Rotate your robot exactly 360° (or 180°, or 720°)
4. Press 'r' and enter the actual rotation angle
5. The script will calculate the actual wheel separation

The robot should turn the exact amount you specify. If it over-rotates, 
wheel_separation is too small. If it under-rotates, wheel_separation is too large.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math


class WheelSeparationTest(Node):
    def __init__(self):
        super().__init__('wheel_separation_test')
        
        # Subscribe to odometry
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # Data storage
        self.start_odom = None
        self.current_odom = None
        
        # Current wheel separation from launch file
        self.current_wheel_separation = 0.153  # Optimized through circle testing for best odometry
        
        self.get_logger().info("Wheel Separation Test Started")
        self.get_logger().info("Current wheel separation: 155mm")
        self.get_logger().info("Commands:")
        self.get_logger().info("  's' - Set starting position")
        self.get_logger().info("  'r' - Calculate rotation results")
        self.get_logger().info("  'q' - Quit")
        self.get_logger().info("")
        self.get_logger().info("Instructions:")
        self.get_logger().info("1. Press 's' to set start position")
        self.get_logger().info("2. Use teleop to rotate robot (try 360° or 180°)")
        self.get_logger().info("3. Press 'r' and enter actual rotation angle")
        
        # Timer for user input handling
        self.timer = self.create_timer(0.1, self.user_input_handler)
        
    def odom_callback(self, msg):
        self.current_odom = msg
        
    def user_input_handler(self):
        pass  # Input handling done in main loop
        
    def set_start_position(self):
        """Record starting odometry values"""
        if self.current_odom:
            self.start_odom = self.current_odom
            
            # Extract starting orientation (quaternion to yaw)
            quat = self.start_odom.pose.pose.orientation
            start_yaw = math.atan2(2.0 * (quat.w * quat.z + quat.x * quat.y),
                                  1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z))
            
            self.get_logger().info(f"Start position set:")
            self.get_logger().info(f"  Position: ({self.start_odom.pose.pose.position.x:.3f}, {self.start_odom.pose.pose.position.y:.3f})")
            self.get_logger().info(f"  Orientation: {math.degrees(start_yaw):.1f}°")
            self.get_logger().info("Now rotate your robot and press 'r' when done.")
        else:
            self.get_logger().warn("No odometry data available")
            
    def calculate_rotation_results(self, actual_angle_degrees):
        """Calculate wheel separation based on rotation test"""
        if not self.start_odom or not self.current_odom:
            self.get_logger().warn("No starting position set or no current data")
            return
            
        # Extract orientations (quaternion to yaw)
        start_quat = self.start_odom.pose.pose.orientation
        start_yaw = math.atan2(2.0 * (start_quat.w * start_quat.z + start_quat.x * start_quat.y),
                              1.0 - 2.0 * (start_quat.y * start_quat.y + start_quat.z * start_quat.z))
        
        current_quat = self.current_odom.pose.pose.orientation  
        current_yaw = math.atan2(2.0 * (current_quat.w * current_quat.z + current_quat.x * current_quat.y),
                                1.0 - 2.0 * (current_quat.y * current_quat.y + current_quat.z * current_quat.z))
        
        # Calculate rotation difference
        angle_diff = current_yaw - start_yaw
        
        # Normalize to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
            
        measured_angle_degrees = math.degrees(angle_diff)
        
        # Calculate position drift (should be minimal for pure rotation)
        dx = self.current_odom.pose.pose.position.x - self.start_odom.pose.pose.position.x
        dy = self.current_odom.pose.pose.position.y - self.start_odom.pose.pose.position.y
        position_drift = math.sqrt(dx*dx + dy*dy)
        
        # Calculate corrected wheel separation
        if measured_angle_degrees != 0:
            correction_factor = actual_angle_degrees / measured_angle_degrees
            corrected_wheel_separation = self.current_wheel_separation * correction_factor
            error_percent = ((measured_angle_degrees - actual_angle_degrees) / actual_angle_degrees) * 100
            
            self.get_logger().info(f"Rotation Test Results:")
            self.get_logger().info(f"  Actual rotation: {actual_angle_degrees:.1f}°")
            self.get_logger().info(f"  Measured rotation: {measured_angle_degrees:.1f}°")
            self.get_logger().info(f"  Error: {error_percent:.2f}%")
            self.get_logger().info(f"  Position drift: {position_drift:.3f}m")
            self.get_logger().info(f"  Current wheel separation: {self.current_wheel_separation:.3f}m")
            self.get_logger().info(f"  Suggested wheel separation: {corrected_wheel_separation:.3f}m")
            
            if abs(error_percent) > 2.0:
                if measured_angle_degrees < actual_angle_degrees:
                    self.get_logger().warn("Robot under-rotated - increase wheel_separation")
                else:
                    self.get_logger().warn("Robot over-rotated - decrease wheel_separation")
                    
                self.get_logger().info(f"Update launch file: wheel_separation: {corrected_wheel_separation:.3f}")
            else:
                self.get_logger().info("Wheel separation looks good!")
                
            if position_drift > 0.1:
                self.get_logger().warn(f"Large position drift ({position_drift:.3f}m) - check wheel bias correction")
                
        else:
            self.get_logger().warn("No rotation detected")


def main(args=None):
    rclpy.init(args=args)
    node = WheelSeparationTest()
    
    print("\nWheel Separation Test")
    print("====================")
    print("Commands:")
    print("  s - Set starting position")
    print("  r - Calculate rotation results")
    print("  q - Quit")
    print()
    
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            
            # Simple input handling
            try:
                cmd = input().strip().lower()
                if cmd == 's':
                    node.set_start_position()
                elif cmd == 'r':
                    angle = float(input("Enter actual rotation angle in degrees (e.g., 360, 180, -180): "))
                    node.calculate_rotation_results(angle)
                elif cmd == 'q':
                    break
                else:
                    print("Unknown command")
            except (EOFError, KeyboardInterrupt):
                break
            except ValueError:
                print("Invalid angle entered")
                
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
