#!/usr/bin/env python3
"""
PID Response Plotter for Pharma Bot (Dual Motor)
================================================

Subscribes to /motor_vels and records both motor velocities as you move the robot or apply a step input.
Plots both left and right motor speed vs. time.

Usage:
    python3 pid_plotter.py
"""

import rclpy
from rclpy.node import Node
from serial_motor_demo_msgs.msg import MotorVels
import matplotlib.pyplot as plt
import time

class PIDPlotter(Node):
    def __init__(self):
        super().__init__('pid_plotter')
        self.left_speeds = []
        self.right_speeds = []
        self.times = []
        self.start_time = time.time()
        self.subscription = self.create_subscription(
            MotorVels,
            'motor_vels',
            self.listener_callback,
            10
        )
        self.get_logger().info('Subscribed to /motor_vels. Move the robot or apply a step input.')

    def listener_callback(self, msg):
        t = time.time() - self.start_time
        self.times.append(t)
        self.left_speeds.append(msg.mot_1_rad_sec)
        self.right_speeds.append(msg.mot_2_rad_sec)


def main(args=None):
    rclpy.init(args=args)
    node = PIDPlotter()
    print("Recording motor velocities... Move the robot or apply a step input.")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Plotting results...")
        plt.figure(figsize=(10, 6))
        plt.plot(node.times, node.left_speeds, label='Left Motor Speed (rad/s)')
        plt.plot(node.times, node.right_speeds, label='Right Motor Speed (rad/s)')
        plt.xlabel('Time (s)')
        plt.ylabel('Speed (rad/s)')
        plt.title('PID Step Response (Left & Right Motors)')
        plt.grid()
        plt.legend()
        plt.show()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
