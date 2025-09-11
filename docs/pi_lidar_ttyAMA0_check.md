# Raspberry Pi /dev/ttyAMA0 LiDAR Setup Check

## 1. Check if /dev/ttyAMA0 exists and permissions:
```bash
ls -la /dev/ttyAMA*
```

## 2. Check if GPIO UART is enabled:
```bash
cat /boot/config.txt | grep -E "uart|serial"
```

Should show:
```
enable_uart=1
```

## 3. Check if user is in dialout group:
```bash
groups $USER | grep dialout
```

If not in dialout group:
```bash
sudo usermod -a -G dialout $USER
# Then logout and login again
```

## 4. Test if device responds:
```bash
# This will show any data coming from LiDAR (Ctrl+C to stop)
sudo cat /dev/ttyAMA0
```

## 5. Check system messages for UART:
```bash
sudo dmesg | grep -i uart
```

## 6. Check if LiDAR config is using correct device:
```bash
cat /home/ubuntu/dev_ws/src/Lidar/ldlidar_node/params/ldlidar.yaml | grep ttyAMA
```

Should show: `serial_port: '/dev/ttyAMA0'`

## 7. Enable GPIO UART if needed:
Add to `/boot/config.txt`:
```
enable_uart=1
dtoverlay=disable-bt
```

Then reboot Pi.

## 8. Test LiDAR launch manually:
```bash
cd ~/dev_ws
source install/setup.bash
ros2 launch ldlidar_node ldlidar_bringup.launch.py
```

Check if it starts without errors, then activate:
```bash
ros2 lifecycle set /ldlidar_node configure
ros2 lifecycle set /ldlidar_node activate
```

## 9. Verify LiDAR data:
```bash
ros2 topic echo /ldlidar_node/scan --once
```

Should show LaserScan data.

## Quick test sequence on Pi:
```bash
# 1. Check device
ls -la /dev/ttyAMA0

# 2. Check config
grep ttyAMA0 /home/ubuntu/dev_ws/src/Lidar/ldlidar_node/params/ldlidar.yaml

# 3. Test launch
cd ~/dev_ws && source install/setup.bash
ros2 launch ldlidar_node ldlidar_bringup.launch.py

# 4. In another terminal, activate
ros2 lifecycle set /ldlidar_node configure
ros2 lifecycle set /ldlidar_node activate

# 5. Check data
ros2 topic echo /ldlidar_node/scan --once
```
