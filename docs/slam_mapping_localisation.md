# SLAM Mapping & Localization Session Summary
## August 25, 2025

### 🎯 **Session Objectives**
- Set up SLAM mapping in Gazebo simulation
- Save maps for later localization
- Test localization with saved maps
- Integrate teleop_twist_keyboard for robot control

### ✅ **What Was Accomplished**

#### 1. **SLAM Mapping Setup (SUCCESS)**
- **Launch Command**: `ros2 launch pharma_bot launch_sim.launch.py`
- **SLAM Command**: `ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./src/pharma_bot/config/mapper_params_online_async.yaml use_sim_time:=true`
- **Teleop Command**: `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/diff_cont/cmd_vel_unstamped`

#### 2. **Key Configuration Changes**
- **Removed twist_mux**: Simplified control by connecting teleop directly to diff_cont
- **Fixed controller config**: Added `enable_odom_tf: true` in `my_controllers.yaml`
- **Updated mapper params**: 
  - `base_frame: base_link` (for simulation)
  - `scan_topic: /scan` (for simulation)
  - `mode: mapping` (for creating maps)

#### 3. **Successful Map Saving**
- **Maps saved to**: `/home/ubuntu/dev_ws/src/pharma_bot/maps/`
- **Files created**:
  - `gazebo_map_serial.data` (SLAM toolbox format)
  - `gazebo_map_serial.posegraph`
  - `gazebo_map_save.yaml` & `.pgm` (standard ROS format)

#### 4. **Localization Setup (IN PROGRESS)**
- **Mode switched**: `mode: localization` in mapper params
- **Key discovery**: `map_start_at_dock: true` seems to help with localization
- **Status**: Showing more promise with dock setting, needs more testing

### 🛠️ **Current Working Configuration**

#### Launch Sequence for Mapping:
```bash
# 1. Start simulation
ros2 launch pharma_bot launch_sim.launch.py

# 2. Start SLAM mapping  
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./src/pharma_bot/config/mapper_params_online_async.yaml use_sim_time:=true

# 3. Control robot (already launches automatically with simulation)
# Teleop window opens automatically via xterm
```

#### Launch Sequence for Localization:
```bash
# 1. Start simulation
ros2 launch pharma_bot launch_sim.launch.py

# 2. Start localization (with map_start_at_dock: true)
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./src/pharma_bot/config/mapper_params_online_async.yaml use_sim_time:=true
```

### 📁 **File Locations**
- **Launch files**: `/home/ubuntu/dev_ws/src/pharma_bot/launch/`
- **Config files**: `/home/ubuntu/dev_ws/src/pharma_bot/config/`
- **Saved maps**: `/home/ubuntu/dev_ws/src/pharma_bot/maps/`
- **World files**: `/home/ubuntu/dev_ws/src/pharma_bot/worlds/`

### 🔧 **Key Parameters**

#### For Simulation (current):
- `base_frame: base_link`
- `scan_topic: /scan`
- `odom_frame: odom`
- `map_frame: map`

#### For Real Robot:
- `base_frame: base_footprint` 
- `scan_topic: /ldlidar_node/scan`

### 🚨 **Issues Encountered & Solutions**

#### 1. **Transform Issues** (SOLVED)
- **Problem**: No transforms between odom and wheels
- **Solution**: Added `enable_odom_tf: true` in diff_drive controller

#### 2. **Teleop Integration** (SOLVED)  
- **Problem**: twist_mux was crashing
- **Solution**: Direct connection teleop → diff_cont via remapping

#### 3. **Gazebo Performance** (NOTED)
- **Problem**: 10% real-time factor
- **Potential fixes**: Reduce physics step size, turn off shadows, reduce verbosity

#### 4. **World File Saving** (ISSUE)
- **Problem**: Saved .sdf world file had XML errors
- **Workaround**: Use empty.world, add objects manually in Gazebo

#### 5. **Localization Startup** (IN PROGRESS)
- **Problem**: "No map received" errors
- **Recent discovery**: `map_start_at_dock: true` shows promise

### 🎯 **Next Session TODO**

#### High Priority:
1. **Complete localization testing** with `map_start_at_dock: true`
2. **Test in RViz**: Verify map loads and robot localizes correctly
3. **Document working localization parameters**

#### Medium Priority:
1. **Improve Gazebo performance** (physics settings)
2. **Create proper custom world** for mapping/testing
3. **Test real robot integration** with saved maps

#### Low Priority:
1. **Organize documentation** in `/docs` folder
2. **Create launch file variants** for different scenarios
3. **Add navigation capabilities** (Nav2 integration)

### 📊 **Performance Notes**
- **SLAM mapping**: Working well
- **Map saving**: Successful
- **Teleop control**: Smooth operation
- **Localization**: Promising with dock setting
- **Gazebo sim**: Functional but slow (10% real-time)

### 🔄 **Quick Reference Commands**

```bash
# Build workspace
colcon build --packages-select pharma_bot

# Check transforms
ros2 run tf2_tools view_frames

# Check controllers
ros2 control list_controllers

# Check topics
ros2 topic list | grep -E "(scan|odom|map|cmd_vel)"

# Save map manually
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name: {data: 'my_map_name'}"
```

---
**Status**: SLAM mapping working perfectly ✅ | Localization in progress 🔄 | Ready for next session 🚀
