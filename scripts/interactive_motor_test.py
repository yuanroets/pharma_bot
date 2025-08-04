#!/usr/bin/env python3

"""
Interactive Motor Test Script
Simple command-line interface for testing motors and determining CPR
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32MultiArray
import sys
import threading
import time


class InteractiveMotorTest(Node):
    def __init__(self):
        super().__init__('interactive_motor_test')
        
        # Publishers
        self.pwm_pub = self.create_publisher(Float64MultiArray, 'test_motor_pwm', 10)
        
        # Subscribers  
        self.encoder_sub = self.create_subscription(
            Int32MultiArray, 'encoder_counts', self.encoder_callback, 10)
        
        self.last_encoder_data = [0, 0]
        self.initial_encoder_data = [0, 0]
        self.encoder_data_received = False
        
        self.get_logger().info("Interactive Motor Test Ready")
    
    def encoder_callback(self, msg):
        """Store latest encoder data"""
        self.last_encoder_data = msg.data
        if not self.encoder_data_received:
            self.initial_encoder_data = msg.data[:]
            self.encoder_data_received = True
    
    def send_pwm_command(self, left_pwm, right_pwm):
        """Send PWM command to motors"""
        msg = Float64MultiArray()
        msg.data = [float(left_pwm), float(right_pwm)]
        self.pwm_pub.publish(msg)
        self.get_logger().info(f"Sent PWM: Left={left_pwm}, Right={right_pwm}")
    
    def stop_motors(self):
        """Stop both motors"""
        self.send_pwm_command(0, 0)
    
    def get_encoder_diff(self):
        """Get encoder count differences from start"""
        if self.encoder_data_received:
            left_diff = self.last_encoder_data[0] - self.initial_encoder_data[0]
            right_diff = self.last_encoder_data[1] - self.initial_encoder_data[1]
            return left_diff, right_diff
        return 0, 0
    
    def reset_encoder_reference(self):
        """Reset the reference point for encoder counting"""
        if self.encoder_data_received:
            self.initial_encoder_data = self.last_encoder_data[:]
            self.get_logger().info("Encoder reference reset")
        else:
            self.get_logger().warn("No encoder data received yet")


def ros_spin_thread(node):
    """Thread function to spin ROS"""
    rclpy.spin(node)


def main():
    rclpy.init()
    
    test_node = InteractiveMotorTest()
    
    # Start ROS spinning in a separate thread
    ros_thread = threading.Thread(target=ros_spin_thread, args=(test_node,))
    ros_thread.daemon = True
    ros_thread.start()
    
    # Wait a moment for connections
    time.sleep(1)
    
    print("\n" + "="*60)
    print("INTERACTIVE MOTOR TEST")
    print("="*60)
    print("Commands:")
    print("  f <pwm>     - Move forward (both motors) at PWM value")
    print("  b <pwm>     - Move backward (both motors) at PWM value")
    print("  l <pwm>     - Turn left (right motor only) at PWM value")
    print("  r <pwm>     - Turn right (left motor only) at PWM value")
    print("  m <l> <r>   - Manual control: left PWM, right PWM")
    print("  s           - Stop motors")
    print("  e           - Show current encoder counts")
    print("  reset       - Reset encoder reference point")
    print("  cpr <revs>  - Calculate CPR after <revs> manual rotations")
    print("  q           - Quit")
    print("="*60)
    print("PWM Range: -4095 to 4095 (start with low values like 500-1000)")
    print("For CPR determination: use 'reset', then manually turn wheels,")
    print("then use 'cpr <number_of_revolutions>' to calculate CPR")
    print("="*60)
    
    try:
        while True:
            try:
                command = input("\nEnter command: ").strip().split()
                
                if not command:
                    continue
                
                cmd = command[0].lower()
                
                if cmd == 'q' or cmd == 'quit':
                    break
                
                elif cmd == 's' or cmd == 'stop':
                    test_node.stop_motors()
                
                elif cmd == 'f' or cmd == 'forward':
                    if len(command) > 1:
                        pwm = int(command[1])
                        test_node.send_pwm_command(pwm, pwm)
                    else:
                        print("Usage: f <pwm_value>")
                
                elif cmd == 'b' or cmd == 'backward':
                    if len(command) > 1:
                        pwm = -int(command[1])
                        test_node.send_pwm_command(pwm, pwm)
                    else:
                        print("Usage: b <pwm_value>")
                
                elif cmd == 'l' or cmd == 'left':
                    if len(command) > 1:
                        pwm = int(command[1])
                        test_node.send_pwm_command(0, pwm)
                    else:
                        print("Usage: l <pwm_value>")
                
                elif cmd == 'r' or cmd == 'right':
                    if len(command) > 1:
                        pwm = int(command[1])
                        test_node.send_pwm_command(pwm, 0)
                    else:
                        print("Usage: r <pwm_value>")
                
                elif cmd == 'm' or cmd == 'manual':
                    if len(command) > 2:
                        left_pwm = int(command[1])
                        right_pwm = int(command[2])
                        test_node.send_pwm_command(left_pwm, right_pwm)
                    else:
                        print("Usage: m <left_pwm> <right_pwm>")
                
                elif cmd == 'e' or cmd == 'encoders':
                    if test_node.encoder_data_received:
                        left_diff, right_diff = test_node.get_encoder_diff()
                        print(f"Current encoder counts: L={test_node.last_encoder_data[0]}, R={test_node.last_encoder_data[1]}")
                        print(f"Difference from reference: L={left_diff}, R={right_diff}")
                    else:
                        print("No encoder data received yet")
                
                elif cmd == 'reset':
                    test_node.reset_encoder_reference()
                
                elif cmd == 'cpr':
                    if len(command) > 1:
                        try:
                            revolutions = float(command[1])
                            if test_node.encoder_data_received:
                                left_diff, right_diff = test_node.get_encoder_diff()
                                if revolutions > 0:
                                    left_cpr = abs(left_diff) / revolutions
                                    right_cpr = abs(right_diff) / revolutions
                                    print(f"CPR Calculation for {revolutions} revolutions:")
                                    print(f"  Left motor CPR: {left_cpr:.0f}")
                                    print(f"  Right motor CPR: {right_cpr:.0f}")
                                    print(f"  Average CPR: {(left_cpr + right_cpr) / 2:.0f}")
                                    print(f"  Recommended value: {int((left_cpr + right_cpr) / 2)}")
                                else:
                                    print("Number of revolutions must be greater than 0")
                            else:
                                print("No encoder data received yet")
                        except ValueError:
                            print("Invalid number of revolutions")
                    else:
                        print("Usage: cpr <number_of_revolutions>")
                
                else:
                    print(f"Unknown command: {cmd}")
                    print("Type 'q' to quit or see command list above")
            
            except ValueError:
                print("Invalid input. Please enter valid numbers.")
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"Error: {e}")
    
    finally:
        print("\nStopping motors and shutting down...")
        test_node.stop_motors()
        time.sleep(0.5)
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
