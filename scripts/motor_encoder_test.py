#!/usr/bin/env python3

"""
Simple Motor and Encoder Test Script for determining CPR
This script allows manual motor control and displays encoder counts
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32MultiArray
import time
import board
import busio
from adafruit_pca9685 import PCA9685
import RPi.GPIO as GPIO
from threading import Lock


class MotorEncoderTest(Node):
    def __init__(self):
        super().__init__('motor_encoder_test')
        
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
        self.i2c_address = self.get_parameter('i2c_address').value
        
        # Setup PCA9685
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.pca = PCA9685(i2c, address=self.i2c_address)
            self.pca.frequency = 100  # 100Hz PWM, suitable for most motor drivers
            self.pwm_channel1 = 0  # PWM0 for Motor 1 (Left)
            self.pwm_channel2 = 1  # PWM1 for Motor 2 (Right)
            self.get_logger().info(f"PCA9685 initialized at address 0x{self.i2c_address:02X}")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize PCA9685: {e}")
            raise
        
        # Encoder variables
        self.left_encoder_count = 0
        self.right_encoder_count = 0
        self.last_left_state = 0
        self.last_right_state = 0
        
        # Thread safety
        self.mutex = Lock()
        
        # Setup encoder interrupts
        self._setup_encoder_interrupts()
        
        # Subscribers
        self.pwm_sub = self.create_subscription(
            Float64MultiArray, 'test_motor_pwm', self.pwm_callback, 10)
        
        # Publishers
        self.encoder_pub = self.create_publisher(
            Int32MultiArray, 'encoder_counts', 10)
        
        # Timer for encoder publishing
        self.encoder_timer = self.create_timer(0.5, self.publish_encoder_data)
        
        # Timer for logging encoder counts
        self.log_timer = self.create_timer(2.0, self.log_encoder_counts)
        
        self.get_logger().info("Motor Encoder Test Node Started")
        self.get_logger().info("Send PWM commands to /test_motor_pwm topic [left_pwm, right_pwm]")
        self.get_logger().info("PWM range: -4095 to 4095")
        self.get_logger().info("Monitoring encoder counts...")
        
        # Store initial time for CPR calculation
        self.start_time = time.time()
        self.initial_left_count = 0
        self.initial_right_count = 0
    
    def _setup_encoder_interrupts(self):
        """Setup GPIO interrupts for encoders"""
        try:
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
    
    def set_motor_pwm(self, motor, pwm_value):
        """Set motor PWM with direction control"""
        # Clamp PWM to valid range
        pwm_value = max(-4095, min(4095, pwm_value))
        
        try:
            # Convert to 16-bit duty cycle like the working driver
            if motor == 0:  # Left motor
                if pwm_value >= 0:
                    GPIO.output(self.dir1_pin, GPIO.HIGH)
                    self.pca.channels[self.pwm_channel1].duty_cycle = int((abs(pwm_value) / 4095.0) * 0xFFFF)
                else:
                    GPIO.output(self.dir1_pin, GPIO.LOW)
                    self.pca.channels[self.pwm_channel1].duty_cycle = int((abs(pwm_value) / 4095.0) * 0xFFFF)
            else:  # Right motor
                if pwm_value >= 0:
                    GPIO.output(self.dir2_pin, GPIO.HIGH)
                    self.pca.channels[self.pwm_channel2].duty_cycle = int((abs(pwm_value) / 4095.0) * 0xFFFF)
                else:
                    GPIO.output(self.dir2_pin, GPIO.LOW)
                    self.pca.channels[self.pwm_channel2].duty_cycle = int((abs(pwm_value) / 4095.0) * 0xFFFF)
                    
        except Exception as e:
            self.get_logger().error(f"PWM command failed: {str(e)}")
    
    def stop_motors(self):
        """Stop both motors"""
        self.pca.channels[self.pwm_channel1].duty_cycle = 0
        self.pca.channels[self.pwm_channel2].duty_cycle = 0
    
    def pwm_callback(self, msg):
        """Handle PWM commands"""
        if len(msg.data) >= 2:
            left_pwm = int(msg.data[0])
            right_pwm = int(msg.data[1])
            
            self.get_logger().info(f"Setting PWM: Left={left_pwm}, Right={right_pwm}")
            
            self.set_motor_pwm(0, left_pwm)
            self.set_motor_pwm(1, right_pwm)
            
            # If this is the first movement command, reset counters
            if abs(left_pwm) > 0 or abs(right_pwm) > 0:
                if self.initial_left_count == 0 and self.initial_right_count == 0:
                    with self.mutex:
                        self.initial_left_count = self.left_encoder_count
                        self.initial_right_count = self.right_encoder_count
                        self.start_time = time.time()
                    self.get_logger().info("Starting encoder count measurement...")
    
    def publish_encoder_data(self):
        """Publish encoder counts"""
        with self.mutex:
            encoder_msg = Int32MultiArray()
            encoder_msg.data = [self.left_encoder_count, self.right_encoder_count]
            self.encoder_pub.publish(encoder_msg)
    
    def log_encoder_counts(self):
        """Log current encoder counts and progress"""
        with self.mutex:
            left_diff = self.left_encoder_count - self.initial_left_count
            right_diff = self.right_encoder_count - self.initial_right_count
            elapsed_time = time.time() - self.start_time
        
        self.get_logger().info(f"Encoders - Left: {self.left_encoder_count} (Δ{left_diff}), "
                              f"Right: {self.right_encoder_count} (Δ{right_diff}), "
                              f"Time: {elapsed_time:.1f}s")
        
        # If significant counts accumulated, suggest CPR calculation
        if abs(left_diff) > 100 or abs(right_diff) > 100:
            if abs(left_diff) > 0:
                left_cpr_estimate = abs(left_diff) / (elapsed_time / 60.0) if elapsed_time > 0 else 0
                self.get_logger().info(f"Left motor estimated CPR (if 1 rev/min): {left_cpr_estimate:.0f}")
            
            if abs(right_diff) > 0:
                right_cpr_estimate = abs(right_diff) / (elapsed_time / 60.0) if elapsed_time > 0 else 0
                self.get_logger().info(f"Right motor estimated CPR (if 1 rev/min): {right_cpr_estimate:.0f}")
    
    def reset_counters(self):
        """Reset encoder counters"""
        with self.mutex:
            self.left_encoder_count = 0
            self.right_encoder_count = 0
            self.initial_left_count = 0
            self.initial_right_count = 0
            self.start_time = time.time()
        self.get_logger().info("Encoder counters reset")
    
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
        test_node = MotorEncoderTest()
        
        print("\n" + "="*60)
        print("MOTOR ENCODER TEST NODE")
        print("="*60)
        print("To test motors and determine CPR:")
        print("1. Publish PWM commands to /test_motor_pwm topic")
        print("   Example: ros2 topic pub /test_motor_pwm std_msgs/Float64MultiArray '{data: [1000, 1000]}'")
        print("2. Manually rotate wheels and observe encoder counts")
        print("3. For CPR determination:")
        print("   - Turn motor shaft exactly 30 rotations by hand")
        print("   - Note the encoder count difference")
        print("   - CPR = count_difference / 30")
        print("4. To stop motors: ros2 topic pub /test_motor_pwm std_msgs/Float64MultiArray '{data: [0, 0]}'")
        print("="*60)
        print("Ctrl+C to exit\n")
        
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        print("\nShutting down...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'test_node' in locals():
            test_node.cleanup()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
