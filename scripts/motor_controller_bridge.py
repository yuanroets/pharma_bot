#!/usr/bin/env python3
"""
Motor Controller Bridge
======================

This node bridges the diff_cont controller to serial_motor_demo:
- Subscribes to /diff_cont/cmd_vel_unstamped (from teleop/Nav2)
- Converts to /cmd_vel for serial_motor_demo teleop_bridge
- Allows real robot to match simulation topic structure

This solves the mismatch between:
- Simulation: teleop -> /diff_cont/cmd_vel_unstamped -> diff_cont controller
- Real robot: teleop -> /cmd_vel -> serial_motor_demo

Now both use /diff_cont/cmd_vel_unstamped!
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MotorControllerBridge(Node):
    def __init__(self):
        super().__init__('motor_controller_bridge')
        
        # Subscribe to diff_cont controller topic (like simulation)
        self.diff_cont_subscriber = self.create_subscription(
            Twist,
            '/diff_cont/cmd_vel_unstamped',
            self.diff_cont_callback,
            10
        )
        
        # Publish to serial_motor_demo teleop_bridge topic
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        self.get_logger().info('Motor controller bridge started: /diff_cont/cmd_vel_unstamped -> /cmd_vel')
        
    def diff_cont_callback(self, msg):
        """Forward diff_cont commands to serial_motor_demo"""
        # Just pass through the message unchanged
        self.cmd_vel_publisher.publish(msg)
        
        self.get_logger().debug(
            f'Bridging: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)
    
    bridge = MotorControllerBridge()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    
    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
