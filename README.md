# Pharma Bot

A ROS 2 robot package for the Pharma Bot with integrated motor control, navigation, and sensing capabilities. This robot implements direct motor control through the Raspberry Pi using a PCA9685 PWM driver hat, replacing the traditional Arduino-based approach.

## 🚀 Features

- **Direct Motor Control**: Raspberry Pi-based motor control using PCA9685 PWM hat
- **Encoder Feedback**: Hall effect rotary encoder support with quadrature decoding  
- **PID Control**: Arduino-compatible PID implementation for precise motor control
- **Navigation**: Nav2 integration for autonomous navigation
- **Sensing**: Camera and LiDAR support
- **Simulation**: Gazebo simulation support

## 🔧 Hardware Configuration

### Required Hardware
- **Compute**: Raspberry Pi 4
- **Motor Driver**: PCA9685 PWM/Servo Hat (Default I2C address: 0x40)
- **Motors**: DC motors with TB6612FNG drivers and 45:1 gear reduction
- **Encoders**: Hall effect rotary encoders
- **Additional Sensors**: Camera, LiDAR

### GPIO Pin Assignments
```
Motor Control:
- Left Motor Direction:  GPIO 5
- Right Motor Direction: GPIO 6
- Left Motor PWM:        PCA9685 Channel 0  
- Right Motor PWM:       PCA9685 Channel 1

Encoders:
- Left Encoder A:        GPIO 23
- Left Encoder B:        GPIO 24
- Right Encoder A:       GPIO 27
- Right Encoder B:       GPIO 22
```

## 📦 Installation & Setup

### Step 1: System Dependencies

Enable I2C and install system packages:
```bash
# Enable I2C interface
sudo raspi-config
# Navigate to: Interfacing Options -> I2C -> Enable

# Install system packages
sudo apt-get update
sudo apt-get install python3-smbus i2c-tools python3-rpi.gpio

# Verify I2C is working (should show device at 0x40)
sudo i2cdetect -y 1
```

### Step 2: Python Dependencies

Install required Python packages:
```bash
# Install Adafruit PCA9685 library
pip3 install adafruit-circuitpython-pca9685

# Or install from requirements file
pip3 install -r requirements.txt
```

### Step 3: Build the ROS 2 Package

```bash
# Navigate to workspace
cd ~/dev_ws

# Build the package
colcon build --packages-select pharma_bot

# Source the workspace
source install/setup.bash
```

## 🎯 Motor Control Scripts Overview

This package includes three main scripts for motor control and testing:

### 1. `motor_encoder_test.py` - Hardware Interface Script
**Purpose**: Low-level hardware interface for motor control and encoder reading
**Use Case**: Background process that handles hardware communication

**Features**:
- PCA9685 PWM motor control
- GPIO-based encoder reading with interrupts
- Publishes encoder counts to `/encoder_counts` topic
- Subscribes to `/test_motor_pwm` for manual motor control

### 2. `interactive_motor_test.py` - User Interface Script  
**Purpose**: Interactive command-line interface for testing and CPR determination
**Use Case**: Primary tool for initial testing and encoder calibration

**Features**:
- Easy-to-use command interface
- Real-time encoder monitoring
- Automatic CPR calculation
- Motor safety controls

### 3. `rpi_motor_driver.py` - Full Motor Driver
**Purpose**: Complete motor driver with PID control and ROS 2 integration
**Use Case**: Production motor control for navigation and robot operation

**Features**:
- PID-based closed-loop control
- cmd_vel subscription for navigation integration
- Encoder feedback and odometry
- Debug and monitoring topics

## 🧪 Testing and Calibration Process

### Phase 1: Hardware Verification

First, verify that your hardware is properly connected and functioning:

```bash
# Launch the hardware test
ros2 launch pharma_bot interactive_motor_test.launch.py
```

This launches both the hardware interface (`motor_encoder_test.py`) and interactive interface (`interactive_motor_test.py`).

**Interactive Commands**:
```
Commands available in the interactive terminal:
  f <pwm>     - Move forward (both motors) at PWM value
  b <pwm>     - Move backward (both motors) at PWM value  
  l <pwm>     - Turn left (right motor only) at PWM value
  r <pwm>     - Turn right (left motor only) at PWM value
  m <l> <r>   - Manual control: left PWM, right PWM
  s           - Stop motors
  e           - Show current encoder counts
  reset       - Reset encoder reference point
  cpr <revs>  - Calculate CPR after <revs> manual rotations
  q           - Quit
```

**Initial Testing Steps**:
1. Start with low PWM values (e.g., `f 500`) to test motor direction
2. Verify both motors rotate in the correct direction
3. Check that encoders are counting (use `e` command)
4. Stop motors with `s` command

### Phase 2: Encoder CPR Determination

The most critical step is determining your encoder's Counts Per Revolution (CPR):

1. **Reset encoder reference**:
   ```
   reset
   ```

2. **Turn off motors** (if running):
   ```
   s
   ```

3. **Manually rotate wheels**: Carefully turn each wheel by hand exactly 30 full rotations
   - Count rotations carefully
   - Rotate slowly and steadily
   - Both wheels should be rotated the same amount

4. **Calculate CPR**:
   ```
   cpr 30
   ```

**Example Output**:
```
CPR Calculation for 30.0 revolutions:
  Left motor CPR: 1440
  Right motor CPR: 1436  
  Average CPR: 1438
  Recommended value: 1438
```

5. **Record the CPR value** - you'll need this for the full motor driver

### Phase 3: Motor Control Testing

Test basic motor control with known PWM values:

```bash
# Test various motor commands
f 1000    # Forward at medium speed
b 800     # Backward at lower speed
l 600     # Turn left
r 600     # Turn right
m 500 -500 # Left forward, right backward (rotate in place)
s         # Stop
```

**Safety Notes**:
- PWM range: -4095 to 4095
- Start with low values (500-1000) for initial testing
- Always have `s` (stop) command ready
- Monitor encoder counts with `e` command

### Phase 4: Full Motor Driver Testing

Once you have determined the CPR, test the full motor driver:

```bash
# Launch full motor driver with your determined CPR
ros2 launch pharma_bot rpi_motor_driver.launch.py encoder_cpr:=1438
```

**Test with cmd_vel commands**:
```bash
# In another terminal, test navigation commands

# Move forward slowly
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.1}, angular: {z: 0.0}}'

# Rotate in place
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.0}, angular: {z: 0.5}}'

# Stop
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.0}, angular: {z: 0.0}}'
```

## 📊 Monitoring and Debugging

### Available Topics for Monitoring

```bash
# Monitor encoder counts
ros2 topic echo /encoder_counts

# Monitor PID debug information  
ros2 topic echo /pid_debug

# Monitor motor speeds
ros2 topic echo /motor_speeds

# Check all active topics
ros2 topic list
```

### Debug Information

The full motor driver publishes debug information when `debug_mode: true`:

- **`/encoder_counts`**: Raw encoder count values `[left_count, right_count]`
- **`/pid_debug`**: PID values `[left_target, right_target, left_output, right_output, left_input, right_input]`
- **`/motor_speeds`**: Calculated motor speeds in appropriate units

## ⚙️ Configuration

### Parameter Files

Edit `config/motor_driver_params.yaml` to adjust:

```yaml
motor_driver:
  ros__parameters:
    # Hardware
    i2c_address: 0x40
    encoder_cpr: 1438        # YOUR DETERMINED VALUE
    
    # Control  
    loop_rate: 20            # PID update rate (Hz)
    
    # Robot dimensions
    wheel_separation: 0.17   # Distance between wheels (m)
    wheel_radius: 0.033      # Wheel radius (m)
    
    # PID tuning
    kp: 20
    kd: 12  
    ki: 0
    ko: 50
```

### Launch File Parameters

You can override parameters at launch time:

```bash
ros2 launch pharma_bot rpi_motor_driver.launch.py \
  encoder_cpr:=1438 \
  wheel_separation:=0.17 \
  wheel_radius:=0.033 \
  debug_mode:=true
```

## 🔧 Troubleshooting

### Common Issues

**I2C Device Not Found**:
```bash
# Check I2C is enabled
sudo raspi-config

# Scan for I2C devices
sudo i2cdetect -y 1
# Should show device at 0x40
```

**Permission Denied (GPIO)**:
```bash
# Add user to gpio group
sudo usermod -a -G gpio $USER
# Logout and login again
```

**Motors Don't Move**:
- Check power supply to motors
- Verify GPIO connections
- Start with higher PWM values (1000-2000)
- Check motor driver wiring

**Encoders Not Counting**:
- Verify encoder power (3.3V or 5V as required)
- Check GPIO pull-up resistors are enabled
- Test with multimeter on encoder outputs
- Ensure proper A/B channel connections

**PID Oscillation**:
- Reduce Kp value
- Increase Kd value
- Check encoder CPR is correct

### Log Analysis

Monitor logs for debugging:
```bash
# Run with verbose logging
ros2 launch pharma_bot rpi_motor_driver.launch.py --ros-args --log-level DEBUG
```

## 🚀 Integration with Navigation

Once motor control is working, integrate with your existing navigation:

```bash
# Update your launch_robot.launch.py to include motor driver
ros2 launch pharma_bot launch_robot.launch.py
```

The motor driver subscribes to `/cmd_vel` and is compatible with Nav2 and other navigation frameworks.

## 📁 File Structure

```
pharma_bot/
├── scripts/
│   ├── motor_encoder_test.py      # Hardware interface
│   ├── interactive_motor_test.py  # Interactive testing tool  
│   └── rpi_motor_driver.py        # Full motor driver
├── launch/
│   ├── motor_test.launch.py       # Basic hardware test
│   ├── interactive_motor_test.launch.py  # Interactive test
│   └── rpi_motor_driver.launch.py # Full motor driver
├── config/
│   └── motor_driver_params.yaml   # Configuration parameters
├── requirements.txt               # Python dependencies
└── MOTOR_SETUP.md                # Detailed setup guide
```

## 🎯 Next Steps

1. **Complete hardware testing** using the interactive tool
2. **Determine accurate encoder CPR** for your specific motors
3. **Test closed-loop control** with the full motor driver
4. **Integrate with navigation stack** for autonomous operation
5. **Tune PID parameters** for optimal performance

## 📖 Additional Resources

- See `MOTOR_SETUP.md` for additional detailed setup information
- Arduino code reference in provided attachments
- PCA9685 datasheet for hardware specifications
- ROS 2 Navigation documentation for integration guidance