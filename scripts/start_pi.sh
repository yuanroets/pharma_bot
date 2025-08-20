#!/bin/bash
# Pi Hardware Startup Script
# =========================
# Run this on the Pi to start all hardware components

echo "🤖 Starting Pharma Bot Hardware Components on Pi..."
echo "=================================================="

cd /home/ubuntu/dev_ws
source install/setup.bash

echo ""
echo "🔧 Starting motor driver, teleop bridge, and LiDAR..."
echo "📡 LiDAR will auto-configure and activate after startup"
echo ""

ros2 launch pharma_bot pi_hardware.launch.py

echo ""
echo "✅ Hardware startup complete!"
echo "Next: Run the dev machine script on your development computer"
