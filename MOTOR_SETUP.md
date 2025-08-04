# Pharma Bot Motor Control Setup

This setup provides direct motor and encoder control through the Raspberry Pi using a PCA9685 motor driver hat.

## Hardware Setup

- **Motor Driver**: PCA9685 PWM/Servo Hat (I2C address: 0x40)
- **Motors**: DC motors with TB6612FNG drivers
- **Encoders**: Hall effect rotary encoders with 45:1 gear reduction
- **GPIO Connections**:
  - Motor 1 Direction: GPIO 5
  - Motor 2 Direction: GPIO 6
  - Encoder 1A: GPIO 23
  - Encoder 1B: GPIO 24
  - Encoder 2A: GPIO 27
  - Encoder 2B: GPIO 22

## Required Dependencies

Install the following Python packages on your Raspberry Pi:

```bash
pip3 install adafruit-circuitpython-pca9685
pip3 install RPi.GPIO
sudo apt-get install python3-smbus
```

## Determining Encoder CPR (Counts Per Revolution)

Before using the full motor driver, you need to determine the encoder counts per revolution:

### Step 1: Run the Motor Encoder Test

```bash
# Build the package
cd ~/dev_ws
colcon build --packages-select pharma_bot

# Source the workspace
source install/setup.bash

# Launch the test node
ros2 launch pharma_bot motor_test.launch.py
```

### Step 2: Test Motor Control

In another terminal:

```bash
# Test motors with low PWM (be careful!)
ros2 topic pub /test_motor_pwm std_msgs/Float64MultiArray '{data: [500, 500]}'

# Stop motors
ros2 topic pub /test_motor_pwm std_msgs/Float64MultiArray '{data: [0, 0]}'
```

### Step 3: Manual CPR Determination

1. **Set motors to low PWM or turn them off**
2. **Manually rotate each wheel exactly 30 full rotations**
3. **Observe the encoder count differences in the terminal output**
4. **Calculate CPR**: `CPR = total_count_difference / 30`

Example: If after 30 rotations you see a count difference of 43,200:
```
CPR = 43,200 / 30 = 1,440 counts per revolution
```

### Step 4: Update Configuration

Update the `encoder_cpr` parameter in:
- `config/motor_driver_params.yaml`
- Launch files

## Running the Full Motor Driver

Once you have determined the CPR:

```bash
# Launch the motor driver with your determined CPR
ros2 launch pharma_bot rpi_motor_driver.launch.py encoder_cpr:=1440

# Test with cmd_vel commands
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.1}, angular: {z: 0.0}}'
```

## Monitoring and Debugging

Monitor encoder counts:
```bash
ros2 topic echo /encoder_counts
```

Monitor PID debug info:
```bash
ros2 topic echo /pid_debug
```

Monitor motor speeds:
```bash
ros2 topic echo /motor_speeds
```

## Integration with Existing Robot

To integrate with your existing pharma_bot:

1. **Update launch_robot.launch.py** to include the motor driver:
```python
# Add to your launch_robot.launch.py
motor_driver = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('pharma_bot'), 'launch', 'rpi_motor_driver.launch.py'
    )])
)
```

2. **Connect to ros2_control** (optional):
   - The motor driver publishes to `/cmd_vel`
   - You can remap topics as needed for integration with navigation stack

## Troubleshooting

### I2C Issues
```bash
# Check I2C devices
sudo i2cdetect -y 1

# Enable I2C if not already enabled
sudo raspi-config
# Navigate to Interfacing Options -> I2C -> Enable
```

### GPIO Permissions
```bash
# Add user to gpio group
sudo usermod -a -G gpio $USER
# Logout and login again
```

### PWM Range
- PCA9685 uses 12-bit PWM (0-4095)
- Start with low values (500-1000) for testing
- Maximum safe PWM depends on your motor specifications

## PID Tuning

The default PID values from the Arduino code are:
- Kp = 20
- Kd = 12  
- Ki = 0
- Ko = 50

Adjust these in `config/motor_driver_params.yaml` if needed.
