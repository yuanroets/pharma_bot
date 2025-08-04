#!/usr/bin/env python3

"""
ROS 2 Motor Driver for Raspberry Pi with PCA9685 Motor Driver Hat
Based on the Arduino differential drive motor control code
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, Int32MultiArray, Bool
import time
import math
import board
import busio
from adafruit_pca9685 import PCA9685
import RPi.GPIO as GPIO
from threading import Lock
import signal
import sys


class MotorDriver(Node):
    def __init__(self):
        super().__init__('rpi_motor_driver')
        
        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Motor direction pins
        self.dir1_pin = 5
        self.dir2_pin = 6
        
        # Encoder pins (hall effect rotary encoders)
        self.enc1_a_pin = 23
        self.enc1_b_pin = 24
        self.enc2_a_pin = 27
        self.enc2_b_pin = 22
        
        # Setup GPIO pins
        GPIO.setup(self.dir1_pin, GPIO.OUT)
        GPIO.setup(self.dir2_pin, GPIO.OUT)
        GPIO.setup(self.enc1_a_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.enc1_b_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.enc2_a_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.enc2_b_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        # Parameters
        self.declare_parameter('i2c_address', 0x40)
        self.declare_parameter('max_pwm', 4095)  # 12-bit resolution for PCA9685
        self.declare_parameter('encoder_cpr', 1440)  # Will be determined experimentally
        self.declare_parameter('loop_rate', 20)  # Hz
        self.declare_parameter('wheel_separation', 0.17)  # meters
        self.declare_parameter('wheel_radius', 0.033)  # meters
        self.declare_parameter('debug_mode', True)
        
        # Get parameters
        self.i2c_address = int(self.get_parameter('i2c_address').value)
        self.max_pwm = self.get_parameter('max_pwm').value
        self.encoder_cpr = self.get_parameter('encoder_cpr').value
        self.loop_rate = self.get_parameter('loop_rate').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.debug_mode = self.get_parameter('debug_mode').value
        
        # Setup PCA9685
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.pca = PCA9685(i2c, address=self.i2c_address)
            self.pca.frequency = 100  # 100Hz PWM, suitable for most motor drivers
            self.pwm_channel1 = 0  # PWM0 for Motor 1
            self.pwm_channel2 = 1  # PWM1 for Motor 2
            self.get_logger().info(f"PCA9685 initialized at address 0x{self.i2c_address:02X}")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize PCA9685: {e}")
            raise
        
        # PID Parameters (from Arduino code)
        self.kp = 20
        self.kd = 12
        self.ki = 0
        self.ko = 50
        
        # PID State structures (like Arduino SetPointInfo)
        self.left_pid = {
            'target_ticks_per_frame': 0.0,
            'encoder': 0,
            'prev_enc': 0,
            'prev_input': 0,
            'i_term': 0,
            'output': 0
        }
        
        self.right_pid = {
            'target_ticks_per_frame': 0.0,
            'encoder': 0,
            'prev_enc': 0,
            'prev_input': 0,
            'i_term': 0,
            'output': 0
        }
        
        # Encoder variables
        self.left_encoder_count = 0
        self.right_encoder_count = 0
        self.last_left_state = 0
        self.last_right_state = 0
        
        # Motion state
        self.moving = False
        
        # Thread safety
        self.mutex = Lock()
        
        # Setup encoder interrupts
        self._setup_encoder_interrupts()
        
        # Topics
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        # For manual PWM testing
        self.pwm_sub = self.create_subscription(
            Float64MultiArray, 'test_motor_pwm', self.pwm_callback, 10)
        
        # Publishers for debugging and monitoring
        self.encoder_pub = self.create_publisher(
            Int32MultiArray, 'encoder_counts', 10)
        
        self.motor_speeds_pub = self.create_publisher(
            Float64MultiArray, 'motor_speeds', 10)
        
        self.pid_debug_pub = self.create_publisher(
            Float64MultiArray, 'pid_debug', 10)
        
        # Timer for PID updates
        self.pid_timer = self.create_timer(1.0 / self.loop_rate, self.update_pid)
        
        # Timer for encoder publishing (slower rate for monitoring)
        self.encoder_timer = self.create_timer(0.1, self.publish_encoder_data)
        
        self.get_logger().info("RPI Motor Driver Node Started")
        self.get_logger().info(f"Parameters: CPR={self.encoder_cpr}, Rate={self.loop_rate}Hz")
        
        # Setup cleanup on shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
    
    def _setup_encoder_interrupts(self):
        """Setup GPIO interrupts for encoders"""
        try:
            # Setup interrupts for both edges of both encoder channels
            GPIO.add_event_detect(self.enc1_a_pin, GPIO.BOTH, callback=self._left_encoder_callback)
            GPIO.add_event_detect(self.enc1_b_pin, GPIO.BOTH, callback=self._left_encoder_callback)
            GPIO.add_event_detect(self.enc2_a_pin, GPIO.BOTH, callback=self._right_encoder_callback)
            GPIO.add_event_detect(self.enc2_b_pin, GPIO.BOTH, callback=self._right_encoder_callback)
            self.get_logger().info("Encoder interrupts setup successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to setup encoder interrupts: {e}")
    
    def _left_encoder_callback(self, channel):
        """Left encoder interrupt callback - quadrature decoding"""
        a_state = GPIO.input(self.enc1_a_pin)
        b_state = GPIO.input(self.enc1_b_pin)
        
        current_state = (a_state << 1) | b_state
        
        # Simple quadrature decoding
        if self.last_left_state == 0b00:
            if current_state == 0b01:
                self.left_encoder_count += 1
            elif current_state == 0b10:
                self.left_encoder_count -= 1
        elif self.last_left_state == 0b01:
            if current_state == 0b11:
                self.left_encoder_count += 1
            elif current_state == 0b00:
                self.left_encoder_count -= 1
        elif self.last_left_state == 0b11:
            if current_state == 0b10:
                self.left_encoder_count += 1
            elif current_state == 0b01:
                self.left_encoder_count -= 1
        elif self.last_left_state == 0b10:
            if current_state == 0b00:
                self.left_encoder_count += 1
            elif current_state == 0b11:
                self.left_encoder_count -= 1
        
        self.last_left_state = current_state
    
    def _right_encoder_callback(self, channel):
        """Right encoder interrupt callback - quadrature decoding"""
        a_state = GPIO.input(self.enc2_a_pin)
        b_state = GPIO.input(self.enc2_b_pin)
        
        current_state = (a_state << 1) | b_state
        
        # Simple quadrature decoding
        if self.last_right_state == 0b00:
            if current_state == 0b01:
                self.right_encoder_count += 1
            elif current_state == 0b10:
                self.right_encoder_count -= 1
        elif self.last_right_state == 0b01:
            if current_state == 0b11:
                self.right_encoder_count += 1
            elif current_state == 0b00:
                self.right_encoder_count -= 1
        elif self.last_right_state == 0b11:
            if current_state == 0b10:
                self.right_encoder_count += 1
            elif current_state == 0b01:
                self.right_encoder_count -= 1
        elif self.last_right_state == 0b10:
            if current_state == 0b00:
                self.right_encoder_count += 1
            elif current_state == 0b11:
                self.right_encoder_count -= 1
        
        self.last_right_state = current_state
    
    def set_motor_speed(self, motor, speed):
        """Set motor speed with direction control"""
        # Clamp speed to valid range
        speed = max(-self.max_pwm, min(self.max_pwm, speed))
        
        try:
            # Convert to 16-bit duty cycle like the working driver
            if motor == 0:  # Left motor
                if speed >= 0:
                    GPIO.output(self.dir1_pin, GPIO.HIGH)
                    self.pca.channels[self.pwm_channel1].duty_cycle = int((abs(speed) / self.max_pwm) * 0xFFFF)
                else:
                    GPIO.output(self.dir1_pin, GPIO.LOW)
                    self.pca.channels[self.pwm_channel1].duty_cycle = int((abs(speed) / self.max_pwm) * 0xFFFF)
            else:  # Right motor
                if speed >= 0:
                    GPIO.output(self.dir2_pin, GPIO.HIGH)
                    self.pca.channels[self.pwm_channel2].duty_cycle = int((abs(speed) / self.max_pwm) * 0xFFFF)
                else:
                    GPIO.output(self.dir2_pin, GPIO.LOW)
                    self.pca.channels[self.pwm_channel2].duty_cycle = int((abs(speed) / self.max_pwm) * 0xFFFF)
            
            if self.debug_mode:
                pwm_value = int((abs(speed) / self.max_pwm) * 0xFFFF)
                self.get_logger().info(f"Motor {motor}: speed={speed}, PWM={pwm_value}")
                
        except Exception as e:
            self.get_logger().error(f"PWM command failed: {str(e)}")
    
    def stop_motors(self):
        """Stop both motors"""
        self.pca.channels[self.pwm_channel1].duty_cycle = 0
        self.pca.channels[self.pwm_channel2].duty_cycle = 0
        self.moving = False
    
    def reset_pid(self):
        """Reset PID controllers (like Arduino resetPID function)"""
        with self.mutex:
            self.left_pid['target_ticks_per_frame'] = 0.0
            self.left_pid['encoder'] = self.left_encoder_count
            self.left_pid['prev_enc'] = self.left_pid['encoder']
            self.left_pid['output'] = 0
            self.left_pid['prev_input'] = 0
            self.left_pid['i_term'] = 0
            
            self.right_pid['target_ticks_per_frame'] = 0.0
            self.right_pid['encoder'] = self.right_encoder_count
            self.right_pid['prev_enc'] = self.right_pid['encoder']
            self.right_pid['output'] = 0
            self.right_pid['prev_input'] = 0
            self.right_pid['i_term'] = 0
    
    def do_pid(self, pid_state):
        """PID routine (like Arduino doPID function)"""
        input_val = pid_state['encoder'] - pid_state['prev_enc']
        error = pid_state['target_ticks_per_frame'] - input_val
        
        # PID calculation
        output = (self.kp * error - self.kd * (input_val - pid_state['prev_input']) + pid_state['i_term']) / self.ko
        pid_state['prev_enc'] = pid_state['encoder']
        
        output += pid_state['output']
        
        # Limit output and handle integral windup
        if output >= self.max_pwm:
            output = self.max_pwm
        elif output <= -self.max_pwm:
            output = -self.max_pwm
        else:
            pid_state['i_term'] += self.ki * error
        
        pid_state['output'] = output
        pid_state['prev_input'] = input_val
        
        return output
    
    def update_pid(self):
        """Update PID controllers (like Arduino updatePID function)"""
        with self.mutex:
            # Read encoders
            self.left_pid['encoder'] = self.left_encoder_count
            self.right_pid['encoder'] = self.right_encoder_count
            
            # If not moving, reset PIDs to prevent startup spikes
            if not self.moving:
                if self.left_pid['prev_input'] != 0 or self.right_pid['prev_input'] != 0:
                    self.reset_pid()
                return
            
            # Compute PID for each motor
            left_output = self.do_pid(self.left_pid)
            right_output = self.do_pid(self.right_pid)
            
            # Set motor speeds
            self.set_motor_speed(0, left_output)  # Left motor
            self.set_motor_speed(1, right_output)  # Right motor
            
            # Publish debug information
            if self.debug_mode:
                debug_msg = Float64MultiArray()
                debug_msg.data = [
                    float(self.left_pid['target_ticks_per_frame']),
                    float(self.right_pid['target_ticks_per_frame']),
                    float(left_output),
                    float(right_output),
                    float(self.left_pid['encoder'] - self.left_pid['prev_enc']),
                    float(self.right_pid['encoder'] - self.right_pid['prev_enc'])
                ]
                self.pid_debug_pub.publish(debug_msg)
    
    def cmd_vel_callback(self, msg):
        """Handle cmd_vel messages and convert to motor commands"""
        linear_vel = msg.linear.x  # m/s
        angular_vel = msg.angular.z  # rad/s
        
        # Convert to wheel velocities
        left_wheel_vel = linear_vel - (angular_vel * self.wheel_separation / 2.0)
        right_wheel_vel = linear_vel + (angular_vel * self.wheel_separation / 2.0)
        
        # Convert to ticks per frame
        # wheel_vel (m/s) -> rad/s -> revs/s -> ticks/s -> ticks/frame
        left_rad_per_sec = left_wheel_vel / self.wheel_radius
        right_rad_per_sec = right_wheel_vel / self.wheel_radius
        
        left_ticks_per_sec = left_rad_per_sec * self.encoder_cpr / (2 * math.pi)
        right_ticks_per_sec = right_rad_per_sec * self.encoder_cpr / (2 * math.pi)
        
        left_ticks_per_frame = left_ticks_per_sec / self.loop_rate
        right_ticks_per_frame = right_ticks_per_sec / self.loop_rate
        
        with self.mutex:
            self.left_pid['target_ticks_per_frame'] = left_ticks_per_frame
            self.right_pid['target_ticks_per_frame'] = right_ticks_per_frame
            
            # Set moving flag
            self.moving = abs(linear_vel) > 0.01 or abs(angular_vel) > 0.01
        
        if self.debug_mode:
            self.get_logger().debug(f"Target ticks/frame: L={left_ticks_per_frame:.2f}, R={right_ticks_per_frame:.2f}")
    
    def pwm_callback(self, msg):
        """Handle manual PWM commands for testing"""
        if len(msg.data) >= 2:
            left_pwm = int(msg.data[0])
            right_pwm = int(msg.data[1])
            
            self.get_logger().info(f"Manual PWM command: L={left_pwm}, R={right_pwm}")
            
            try:
                # Disable PID control
                self.moving = False
                
                # Set motors directly
                self.set_motor_speed(0, left_pwm)
                self.set_motor_speed(1, right_pwm)
                
            except Exception as e:
                self.get_logger().error(f"PWM callback failed: {str(e)}")
    
    def publish_encoder_data(self):
        """Publish encoder counts for monitoring"""
        with self.mutex:
            encoder_msg = Int32MultiArray()
            encoder_msg.data = [self.left_encoder_count, self.right_encoder_count]
            self.encoder_pub.publish(encoder_msg)
    
    def _signal_handler(self, signum, frame):
        """Clean shutdown handler"""
        self.get_logger().info("Shutting down motor driver...")
        self.cleanup()
        sys.exit(0)
    
    def cleanup(self):
        """Cleanup GPIO and PCA9685"""
        try:
            self.stop_motors()
            self.pca.deinit()
            GPIO.cleanup()
            self.get_logger().info("Cleanup completed")
        except Exception as e:
            self.get_logger().error(f"Error during cleanup: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        motor_driver = MotorDriver()
        rclpy.spin(motor_driver)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'motor_driver' in locals():
            motor_driver.cleanup()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
