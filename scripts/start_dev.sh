#!/bin/bash
# Dev Machine SLAM and Control Startup Script
# ===========================================
# Run this on your dev machine to start SLAM, RViz, and teleop

echo "🖥️  Starting Pharma Bot SLAM System on Dev Machine..."
echo "===================================================="

cd /home/ubuntu/dev_ws
source install/setup.bash

echo ""
echo "🗺️  Starting SLAM Toolbox for mapping..."
echo "👁️  Starting RViz for visualization..."
echo "⌨️  Starting keyboard teleop for manual control..."
echo ""
echo "💡 In RViz:"
echo "   - Set Fixed Frame to 'map'"
echo "   - Add Map display (topic: /map)"
echo "   - Add LaserScan display (topic: /ldlidar_node/scan)"
echo ""
echo "🎮 Teleop Controls:"
echo "   - Use WASD or arrow keys to drive"
echo "   - Drive slowly to build a good map"
echo "   - Explore all areas you want mapped"
echo ""

ros2 launch pharma_bot dev_slam_control.launch.py

echo ""
echo "✅ SLAM system startup complete!"
echo ""
echo "💾 To save your map when done exploring:"
echo "   ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \"name: 'my_robot_map'\""
