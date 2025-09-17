# SLAM Launch Configuration: Mapping vs Localization Modes

## Overview

The pharma_bot package now has **two separate launch files** for different SLAM modes, eliminating the need to comment/uncomment sections. This document explains when to use each launch file and how they're configured.

## Launch Files

| Launch File | Purpose | SLAM Mode | Nav2 Compatible |
|-------------|---------|-----------|------------------|
| `dev_test_launch.py` | Real-time mapping | **MAPPING** | ❌ No |
| `dev_test_launch_localization.py` | Localization with saved maps | **LOCALIZATION** | ✅ Yes |

## Mode Comparison

| Aspect | **LOCALIZATION MODE** | **MAPPING MODE** |
|--------|----------------------|------------------|
| **Launch File** | `dev_test_launch_localization.py` | `dev_test_launch.py` |
| **Purpose** | Navigate using existing maps | Create new maps from scratch |
| **Nav2 Compatible** | ✅ Yes | ❌ No (mapping only) |
| **Requires Saved Map** | ✅ Yes | ❌ No |
| **Use Cases** | Autonomous navigation, path planning | Initial mapping, exploring new areas |
| **SLAM Node** | `localization_slam_toolbox_node` | `async_slam_toolbox_node` |
| **Map Output** | None (uses existing) | Real-time map creation |

## Current Configuration Status

**Two separate launch files available:**
- 🗺️ **MAPPING**: `dev_test_launch.py` 
- 🧭 **LOCALIZATION**: `dev_test_launch_localization.py`

## Mode Details

### 🗺️ MAPPING MODE (Real-time SLAM)

**Launch Command:**
```bash
ros2 launch pharma_bot dev_test_launch.py
```

**When to Use:**
- Creating maps for the first time
- Exploring new areas
- Updating existing maps
- No saved map available

**Active Components:**
```python
# SLAM mapping node for real-time map creation
slam_toolbox_node = LifecycleNode(
    package='slam_toolbox',
    executable='async_slam_toolbox_node',
    namespace='',
    name='slam_toolbox',
    output='screen',
    parameters=[
        '/home/ubuntu/dev_ws/src/pharma_bot/config/mapper_params_online_async.yaml'
    ],
    remappings=[
        ('/scan', '/ldlidar_node/scan')  # Remap from standard /scan to actual LiDAR topic
    ]          
)
```

### 🧭 LOCALIZATION MODE (Nav2 Compatible)

**Launch Command:**
```bash
ros2 launch pharma_bot dev_test_launch_localization.py
```

**When to Use:**
- You have a saved map file
- You want autonomous navigation
- You need Nav2 path planning
- Robot should localize within known environment

**Active Components:**
```python
# SLAM localization using the same simple approach as mapping
slam_toolbox_node = LifecycleNode(
    package='slam_toolbox',
    executable='localization_slam_toolbox_node',
    namespace='',
    name='slam_toolbox',
    output='screen',
    parameters=[
        '/home/ubuntu/dev_ws/src/pharma_bot/config/mapper_params_online_async.yaml'
    ],
    remappings=[
        ('/scan', '/ldlidar_node/scan')  # Remap from standard /scan to actual LiDAR topic
    ]          
)
```

## How to Switch Modes

**No more commenting/uncommenting required!** Simply use the appropriate launch file:

### 🗺️ For MAPPING MODE:
```bash
ros2 launch pharma_bot dev_test_launch.py
```

### 🧭 For LOCALIZATION MODE:
```bash
ros2 launch pharma_bot dev_test_launch_localization.py
```

Both launch files use the **exact same architecture** and **same simple `LifecycleNode` approach** for consistency and reliability.

## Configuration Files

### Required Files for Both Modes:
- `/home/ubuntu/dev_ws/src/pharma_bot/config/mapper_params_online_async.yaml` - SLAM parameters
- `/home/ubuntu/dev_ws/src/pharma_bot/config/pharma_bot.rviz` - RViz configuration

### Localization Mode Additional Requirements:
- `/home/ubuntu/dev_ws/src/pharma_bot/config/lifecycle_mgr_slam.yaml` - Lifecycle manager config
- Saved map files (`.pgm` and `.yaml`) in appropriate directory

## System Architecture

### Common Components (Both Modes):
- **Robot State Publisher**: URDF model loading
- **Simple Odometry**: Encoder-based odometry calculation
- **Static Transforms**: Coordinate frame relationships
- **Teleop Keyboard**: Manual robot control
- **RViz2**: 3D visualization

### Mode-Specific Components:

**LOCALIZATION MODE:**
- `localization_slam_toolbox_node` - SLAM Toolbox localization mode

**MAPPING MODE:**
- `async_slam_toolbox_node` - Real-time SLAM mapping

Both modes use the same simple `LifecycleNode` approach for consistency and reliability.

## Calibrated Parameters

Both modes use the same calibrated odometry parameters:
- **Encoder CPR**: 1859 (average of L=1866, R=1851)
- **Wheel Separation**: 0.173m (corrected from 360° rotation test)
- **Wheel Radius**: 0.02569m (25.69mm, calibrated from 1m test)
- **Motor Scalers**: L=0.991, R=1.0 (bias correction)

## ROS Configuration

- **ROS_DOMAIN_ID**: 30 (must match Pi configuration)
- **Use Sim Time**: false (real robot operation)
- **LiDAR Topic**: `/ldlidar_node/scan`
- **Command Topic**: `/cmd_vel`

## Workflow Recommendations

### Initial Setup (New Environment):
1. Start with **MAPPING MODE**: `ros2 launch pharma_bot dev_test_launch.py`
2. Drive robot around area to create map
3. Save map using: `ros2 run nav2_map_server map_saver_cli -f my_map`
4. Switch to **LOCALIZATION MODE**: `ros2 launch pharma_bot dev_test_launch_localization.py`

### Operational Use (Known Environment):
1. Use **LOCALIZATION MODE** for normal operation: `ros2 launch pharma_bot dev_test_launch_localization.py`
2. Switch to **MAPPING MODE** only when exploring new areas: `ros2 launch pharma_bot dev_test_launch.py`
3. Update saved maps as needed

## Troubleshooting

### Common Issues:
- **Missing map files**: Ensure saved maps exist for localization mode
- **Transform errors**: Check that odometry calibration is correct
- **Node conflicts**: Only one SLAM mode should be active at a time
- **Domain ID mismatch**: Ensure both Pi and dev machine use ROS_DOMAIN_ID=30

### Verification Commands:
```bash
# Check active nodes
ros2 node list

# Monitor transforms
ros2 run tf2_tools view_frames

# Check SLAM status
ros2 topic echo /slam_toolbox/scan_visualization
```

---

**File Location**: `/home/ubuntu/dev_ws/src/pharma_bot/real_launch_mapping_vs_localisation.md`
**Last Updated**: September 17, 2025
**System**: ROS2 Jazzy with Pharma Bot Hardware
