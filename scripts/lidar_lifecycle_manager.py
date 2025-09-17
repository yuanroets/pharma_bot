#!/usr/bin/env python3
"""
LiDAR Lifecycle Manager - ROBUST LIDAR ACTIVATION
=================================================

This script properly manages LiDAR lifecycle by:
1. Waiting for the node to be available
2. Checking current state before transitions
3. Handling errors gracefully

Usage:
    ros2 run pharma_bot lidar_lifecycle_manager

Or include in a launch file:
    Node(
        package='pharma_bot',
        executable='lidar_lifecycle_manager',
        name='lidar_lifecycle_manager',
        output='screen'
    )
"""

import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import GetState, ChangeState
from lifecycle_msgs.msg import Transition
import time


class LidarLifecycleManager(Node):
    def __init__(self):
        super().__init__('lidar_lifecycle_manager')
        
        # Service clients for lifecycle management
        self.get_state_client = self.create_client(GetState, '/ldlidar_node/get_state')
        self.change_state_client = self.create_client(ChangeState, '/ldlidar_node/change_state')
        
        # Timer to manage lifecycle
        self.timer = self.create_timer(2.0, self.manage_lifecycle)
        self.activation_attempted = False
        
        self.get_logger().info('LiDAR Lifecycle Manager started')

    def manage_lifecycle(self):
        """Manage LiDAR lifecycle states"""
        if self.activation_attempted:
            return
            
        # Check if services are available
        if not self.get_state_client.service_is_ready():
            self.get_logger().info('Waiting for LiDAR node to be available...')
            return
            
        if not self.change_state_client.service_is_ready():
            self.get_logger().info('Waiting for LiDAR change_state service...')
            return
        
        # Get current state
        request = GetState.Request()
        future = self.get_state_client.call_async(request)
        
        # Add callback for when service completes
        future.add_done_callback(self.handle_get_state)

    def handle_get_state(self, future):
        """Handle the response from get_state service"""
        try:
            response = future.result()
            current_state = response.current_state.label
            
            self.get_logger().info(f'LiDAR current state: {current_state}')
            
            if current_state == 'unconfigured':
                self.configure_lidar()
            elif current_state == 'inactive':
                self.activate_lidar()
            elif current_state == 'active':
                self.get_logger().info('LiDAR is already active!')
                self.activation_attempted = True
            else:
                self.get_logger().warn(f'LiDAR in unexpected state: {current_state}')
                
        except Exception as e:
            self.get_logger().error(f'Failed to get LiDAR state: {e}')

    def configure_lidar(self):
        """Configure the LiDAR node"""
        self.get_logger().info('Configuring LiDAR...')
        
        request = ChangeState.Request()
        request.transition.id = Transition.TRANSITION_CONFIGURE
        
        future = self.change_state_client.call_async(request)
        future.add_done_callback(self.handle_configure_response)

    def handle_configure_response(self, future):
        """Handle configure response"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('LiDAR configured successfully')
                # Wait a bit then activate
                time.sleep(1.0)
                self.activate_lidar()
            else:
                self.get_logger().error('Failed to configure LiDAR')
        except Exception as e:
            self.get_logger().error(f'Configure request failed: {e}')

    def activate_lidar(self):
        """Activate the LiDAR node"""
        self.get_logger().info('Activating LiDAR...')
        
        request = ChangeState.Request()
        request.transition.id = Transition.TRANSITION_ACTIVATE
        
        future = self.change_state_client.call_async(request)
        future.add_done_callback(self.handle_activate_response)

    def handle_activate_response(self, future):
        """Handle activate response"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('LiDAR activated successfully!')
                self.activation_attempted = True
            else:
                self.get_logger().error('Failed to activate LiDAR')
        except Exception as e:
            self.get_logger().error(f'Activate request failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = LidarLifecycleManager()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
