# Hardware Flow Diagram - Key Node Explanations

## Raspberry Pi Nodes (Always Running)

### Core Hardware Interface Nodes
- **`pi_test_launch.py`**: Main hardware launch file that starts all Pi-side nodes
- **`driver.py`**: Motor driver node that communicates with Arduino via USB serial
  - Sends PWM commands to motors
  - Receives encoder feedback data
- **`teleop_bridge.py`**: Converts ROS2 `/cmd_vel` messages to motor commands
  - Subscribes to: `/cmd_vel` (geometry_msgs/Twist)
  - Publishes to: Motor command topics for differential drive
- **`simple_odometry.py`**: Processes encoder data into odometry information
  - Subscribes to: Encoder data from motor driver
  - Publishes to: `/odom` (nav_msgs/Odometry), `odom→base_link` transform
- **`lidar_lifecycle_manager.py`**: Manages LiDAR node startup and health monitoring
- **`ldlidar_bringup_launch.py`**: Launches the LiDAR driver node
  - Publishes to: `/ldlidar_node/scan` (sensor_msgs/LaserScan)
- **`joint_state_bridge.py`**: Converts encoder data to joint states for visualization
  - Publishes to: `/joint_states` (sensor_msgs/JointState)
- **`robot.urdf.xacro`**: Robot model providing transform tree structure

### Key Topic Flows (Pi ↔ Dev Machine)
- **Pi → Dev**: `/ldlidar_node/scan`, `/odom`, `/joint_states`, `/tf`
- **Dev → Pi**: `/cmd_vel` (from teleop or Nav2)

## Dev Machine Modes

### Mapping Mode (Pink)
- **`dev_test_launch.py`**: Sets up visualization and basic robot state on dev machine
- **`online_async_launch.py` (SLAM Toolbox)**: Real-time mapping
  - Subscribes to: `/ldlidar_node/scan`, `/odom`
  - Publishes to: `/map`, `map→odom` transform
- **`teleop_twist_keyboard`**: Manual robot control for exploration
  - Publishes to: `/cmd_vel`

### SLAM Localization Mode (Orange)
- **`dev_test_launch_localization.py`**: Similar to mapping but for localization
- **`localization_launch.py` (SLAM Toolbox)**: Uses saved maps for localization
  - Subscribes to: `/ldlidar_node/scan`, `/odom`
  - Publishes to: `/map`, `map→odom` transform (using saved map)
- **`navigation_launch.py` (Nav2)**: Full navigation stack
  - Subscribes to: `/map`, `/odom`, `/scan`
  - Publishes to: `/cmd_vel`, `/local_plan`, `/global_plan`

### AMCL Mode (Red)
- **`dev_test_amcl_launch.py`**: Basic setup for AMCL-based navigation
- **`amcl_localization_launch.py`**: Probabilistic localization using particle filters
  - Subscribes to: `/scan`, `/odom`
  - Publishes to: `map→odom` transform, `/pose`
- **`amcl_navigation_launch.py`**: Nav2 navigation with AMCL localization
  - More robust than SLAM-based navigation for production use

## UI Integration (Blue)
- **`nav_gui.py`**: ROS2 node that receives navigation goals from web interface
  - Subscribes to: `/nav_goal` (from FastAPI)
  - Publishes to: Nav2 action server for autonomous navigation
- **`nav_api.py`**: FastAPI server exposing HTTP endpoints for web interface
  - Receives HTTP POST requests with coordinates
  - Publishes to: `/nav_goal` topic

## Visualization & Monitoring
- **`rviz2`**: 3D visualization showing:
  - Robot model and transforms
  - LiDAR scan data
  - Map and localization
  - Navigation paths and goals
  - All coordinate frames

## Critical Data Flows

### Control Flow
```
Web App → nav_api.py → /nav_goal → nav_gui.py → Nav2 → /cmd_vel → teleop_bridge.py → Motor Commands → Arduino → Motors
```

### Sensor Data Flow
```
LiDAR → /ldlidar_node/scan → SLAM/AMCL → /map, /tf transforms
Encoders → odometry → /odom → SLAM/AMCL/Nav2
```

### Coordinate Frame Chain
```
map → odom → base_link → chassis → ldlidar_base → ldlidar_link
     (SLAM/AMCL)  (odometry)    (robot_state_publisher)
```

## Network Communication
- **ROS_DOMAIN_ID=30**: Ensures Pi and Dev Machine communicate on same ROS2 network
- **Topic Remapping**: Some nodes use topic relays to ensure compatibility:
  - `/ldlidar_node/scan` → `/scan` (for Nav2/AMCL compatibility)
  - `/cmd_vel_smoothed` → `/cmd_vel` (for smooth navigation)
