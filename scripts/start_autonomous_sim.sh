#!/bin/bash
# Autonomous Navigation Simulation Startup
# ========================================
# Click-to-navigate simulation with your real-world maps

echo "🚀 Starting Autonomous Navigation Simulation"
echo "============================================="

cd /home/ubuntu/dev_ws
source install/setup.bash

echo ""
echo "🗺️  Loading your SLAM-generated map for navigation testing"
echo "🎮 Once loaded, use RViz tools:"
echo "   1. Click '2D Goal Pose' tool (green arrow icon)"
echo "   2. Click anywhere on the map"
echo "   3. Robot will autonomously navigate there!"
echo ""
echo "📍 Available maps:"
if [ -f "/home/ubuntu/dev_ws/my_robot_map.yaml" ]; then
    echo "   ✅ Found your SLAM map: my_robot_map.yaml"
    MAP_FILE="/home/ubuntu/dev_ws/my_robot_map.yaml"
else
    echo "   ❌ No SLAM map found yet"
    echo "   💡 First create a map using: ./src/pharma_bot/scripts/start_dev.sh"
    echo "   📝 Will use default test map for now"
    MAP_FILE=""
fi

echo ""
echo "🏃 Starting simulation..."

if [ -n "$MAP_FILE" ]; then
    ros2 launch pharma_bot sim_nav_simple.launch.py map_file:="$MAP_FILE"
else
    ros2 launch pharma_bot sim_nav_simple.launch.py
fi
