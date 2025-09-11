#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelRelay(Node):
    """
    Relay node that handles both Nav2 and teleop commands:
    - Subscribes to /cmd_vel (Nav2 navigation)
    - Subscribes to /cmd_vel_teleop (manual teleop)
    - Publishes to /diff_cont/cmd_vel_unstamped (controller)
    
    Priority: Teleop commands override Nav2 commands when present.
    """
    
    def __init__(self):
        super().__init__('cmd_vel_relay')
        
        # Subscribe to Nav2's cmd_vel topic
        self.nav_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.nav_callback,
            10
        )
        
        # Subscribe to teleop cmd_vel topic
        self.teleop_subscription = self.create_subscription(
            Twist,
            '/cmd_vel_teleop',
            self.teleop_callback,
            10
        )
        
        # Publish to the controller's expected topic
        self.publisher = self.create_publisher(
            Twist,
            '/diff_cont/cmd_vel_unstamped',
            10
        )
        
        # Track last received messages and times
        self.last_nav_msg = Twist()
        self.last_teleop_msg = Twist()
        self.last_nav_time = self.get_clock().now()
        self.last_teleop_time = self.get_clock().now()
        
        # Timeout for considering a command "active" (0.5 seconds)
        self.timeout_duration = 0.5
        
        self.get_logger().info('CMD Vel Relay started: Nav2 + Teleop -> Controller')
        
    def nav_callback(self, msg):
        """Handle Nav2 velocity commands."""
        self.last_nav_msg = msg
        self.last_nav_time = self.get_clock().now()
        self.decide_and_publish()
        
    def teleop_callback(self, msg):
        """Handle teleop velocity commands."""
        self.last_teleop_msg = msg
        self.last_teleop_time = self.get_clock().now()
        self.decide_and_publish()
        
    def decide_and_publish(self):
        """Decide which command to use and publish it."""
        current_time = self.get_clock().now()
        
        # Check if teleop is active (has recent non-zero commands)
        teleop_timeout = (current_time - self.last_teleop_time).nanoseconds / 1e9
        teleop_active = (teleop_timeout < self.timeout_duration and 
                        (abs(self.last_teleop_msg.linear.x) > 0.01 or 
                         abs(self.last_teleop_msg.angular.z) > 0.01))
        
        # Check if nav is active
        nav_timeout = (current_time - self.last_nav_time).nanoseconds / 1e9
        nav_active = (nav_timeout < self.timeout_duration and 
                     (abs(self.last_nav_msg.linear.x) > 0.01 or 
                      abs(self.last_nav_msg.angular.z) > 0.01))
        
        # Priority: Teleop > Nav2
        if teleop_active:
            self.publisher.publish(self.last_teleop_msg)
            self.get_logger().info(
                f'Using TELEOP: linear={self.last_teleop_msg.linear.x:.3f}, angular={self.last_teleop_msg.angular.z:.3f}',
                throttle_duration_sec=1.0
            )
        elif nav_active:
            self.publisher.publish(self.last_nav_msg)
            self.get_logger().info(
                f'Using NAV2: linear={self.last_nav_msg.linear.x:.3f}, angular={self.last_nav_msg.angular.z:.3f}',
                throttle_duration_sec=1.0
            )
        else:
            # No active commands, publish stop
            stop_msg = Twist()
            self.publisher.publish(stop_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
