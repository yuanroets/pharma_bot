# Pharma Bot Documentation
==========================

This folder contains all documentation for the Pharma Bot project.

## 📚 Documentation Files

### Setup Guides
- **`simple_robot_launch_instructions.txt`** - Working real robot launch instructions (serial_motor_demo approach)
- **`real_robot_launch_with_teleop.txt`** - Alternative real robot launch with ROS2 control
- **`sim_launch_with_teleop.txt`** - Simulation launch instructions
- **`rviz_setup_guide.txt`** - RViz configuration and setup

### Configuration Files  
- **`requirements.txt`** - Python dependencies
- **`CMakeLists.txt`** - Build configuration

### Project Status
- **`../SESSION_SUMMARY_AND_TODO.md`** - Current issues and next session goals

## 🚀 Quick Start

### For Real Robot (Recommended):
```bash
# On Pi:
ros2 launch pharma_bot real_robot_pi.launch.py

# On Dev Machine:
ros2 launch pharma_bot real_robot_viz.launch.py
```

### For Simulation:
```bash
ros2 launch pharma_bot launch_sim.launch.py
```

## ⚡ Next Development Phase

1. **Fix Visualization Issues** (robot body, movement)
2. **Implement Proper Odometry** (dynamic transforms)  
3. **SLAM Integration** (slam_toolbox in simulation first)
4. **Real Robot SLAM** (after simulation testing)

See `../SESSION_SUMMARY_AND_TODO.md` for detailed next steps.
