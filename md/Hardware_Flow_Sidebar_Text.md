# Hardware Flow Diagram - Key Information

## Raspberry Pi (Green Box)
**Always Running Hardware Interface:**
- **pi_test_launch.py**: Main hardware launcher
- **driver.py**: Arduino USB communication
  - Publishes: `/motor_vel` (motor velocities), encoder data
  - Subscribes: motor commands from teleop_bridge
- **teleop_bridge.py**: Velocity command converter
  - Subscribes: `/cmd_vel` (geometry_msgs/Twist)
  - Publishes: motor commands for differential drive
- **simple_odometry.py**: Odometry processor
  - Subscribes: encoder data from driver
  - Publishes: `/odom` (nav_msgs/Odometry), `odom→base_link` transform
- **lidar_lifecycle_manager.py**: LiDAR health monitoring
- **ldlidar_bringup_launch.py**: LiDAR driver
  - Publishes: `/ldlidar_node/scan` (sensor_msgs/LaserScan)
- **joint_state_bridge.py**: Joint state publisher
  - Subscribes: encoder data
  - Publishes: `/joint_states` (sensor_msgs/JointState)
- **robot.urdf.xacro**: Robot model & all static transforms

## Dev Machine Modes

### Mapping Mode (Pink)
- **Purpose**: Create new maps while exploring
- **dev_test_launch.py**: Basic robot state & visualization setup
- **online_async_launch.py (SLAM)**: Real-time mapping
  - Subscribes: `/ldlidar_node/scan`, `/odom`
  - Publishes: `/map` (nav_msgs/OccupancyGrid), `map→odom` transform
- **teleop_twist_keyboard**: Manual control
  - Publishes: `/cmd_vel` (geometry_msgs/Twist)
- **rviz2**: Live visualization of mapping process

### SLAM Localization Mode (Orange)  
- **Purpose**: Navigate using SLAM with saved maps
- **dev_test_launch_localization.py**: Setup for localization mode
- **localization_launch.py (SLAM)**: Uses saved maps
  - Subscribes: `/ldlidar_node/scan`, `/odom`
  - Publishes: `/map`, `map→odom` transform (from saved map)
- **navigation_launch.py (Nav2)**: Full navigation stack
  - Subscribes: `/map`, `/odom`, `/scan` (remapped), `/nav_goal`
  - Publishes: `/cmd_vel`, `/local_plan`, `/global_plan`, `/cmd_vel_smoothed`
- **Topic Relay**: `/ldlidar_node/scan` → `/scan` for Nav2 compatibility

### AMCL Mode (Red)
- **Purpose**: Production navigation with particle filter localization
- **dev_test_amcl_launch.py**: Basic AMCL setup
- **amcl_localization_launch.py**: Particle filter localization
  - Subscribes: `/scan` (remapped), `/odom`, `/initialpose`
  - Publishes: `map→odom` transform, `/pose`, `/particle_cloud`
- **amcl_navigation_launch.py**: Nav2 with AMCL
  - Subscribes: `/map`, `/odom`, `/scan`, `/nav_goal`
  - Publishes: `/cmd_vel`, `/local_plan`, `/global_plan`
- **Topic Relay**: `/ldlidar_node/scan` → `/scan` for AMCL compatibility

## UI Integration (Blue)
- **nav_gui.py**: ROS2 ↔ Web interface bridge
  - Subscribes: `/nav_goal` (geometry_msgs/PoseStamped)
  - Action Client: `NavigateToPose` (sends goals to Nav2)
- **nav_api.py**: FastAPI HTTP server
  - Endpoint: `/navigate` (receives POST requests with coordinates)
  - Publishes: `/nav_goal` (geometry_msgs/PoseStamped)
- **rviz2**: Real-time 3D visualization
  - Displays: robot model, `/map`, `/scan`, `/local_plan`, `/global_plan`, transforms

## Critical Data Flows & Topics
**Control Flow**: 
- Web App → `HTTP POST /navigate` → nav_api.py → `/nav_goal` → nav_gui.py → Nav2 Action → `/cmd_vel` → teleop_bridge → motor commands → Arduino
- Manual: teleop_twist_keyboard → `/cmd_vel` → teleop_bridge → motors

**Sensor Data Flow**:
- LiDAR: `/ldlidar_node/scan` → SLAM/AMCL → `/map`, transforms
- Encoders: driver → `/motor_vel`, `/odom` → SLAM/AMCL/Nav2
- Joint States: `/joint_states` → robot_state_publisher → visualization

**Key Transform Chain**: 
`map → odom → base_link → chassis → ldlidar_base → ldlidar_link`

**Navigation Topics**:
- `/nav_goal`: Navigation targets from web interface
- `/local_plan`, `/global_plan`: Path planning visualization
- `/cmd_vel_smoothed`: Smoothed velocity commands
- `/initialpose`: Manual robot pose initialization in RViz

**Network**: `ROS_DOMAIN_ID=30` enables Pi ↔ Dev Machine communication

## Topic Flow Summary (For Diagram - Topics as Diamonds)

### `/cmd_vel` (geometry_msgs/Twist)
- **Publishers**: teleop_twist_keyboard, amcl_navigation, nav2_navigation
- **Subscribers**: teleop_bridge.py

### `/motor_vel` (Custom motor velocity messages)
- **Publishers**: driver.py
- **Subscribers**: (Internal motor monitoring/debugging)

### `/odom` (nav_msgs/Odometry)
- **Publishers**: simple_odometry.py
- **Subscribers**: slam_online_async, slam_localization, amcl_localization, amcl_navigation, nav2_navigation

### `/ldlidar_node/scan` (sensor_msgs/LaserScan)
- **Publishers**: ldlidar_bringup_launch.py (LiDAR driver)
- **Subscribers**: slam_online_async, slam_localization, Topic Relay (scan_relay)

### `/scan` (sensor_msgs/LaserScan)
- **Publishers**: Topic Relay (scan_relay)
- **Subscribers**: amcl_localization, amcl_navigation, nav2_navigation

### `/joint_states` (sensor_msgs/JointState)
- **Publishers**: joint_state_bridge.py
- **Subscribers**: robot_state_publisher, RViz

### `/map` (nav_msgs/OccupancyGrid)
- **Publishers**: slam_online_async, slam_localization, amcl_localization (map_server)
- **Subscribers**: amcl_localization, amcl_navigation, nav2_navigation, RViz

### `/nav_goal` (geometry_msgs/PoseStamped)
- **Publishers**: nav_api.py (FastAPI server)
- **Subscribers**: nav_gui.py

### `/local_plan` (nav_msgs/Path)
- **Publishers**: amcl_navigation, nav2_navigation (local planner)
- **Subscribers**: RViz

### `/global_plan` (nav_msgs/Path)
- **Publishers**: amcl_navigation, nav2_navigation (global planner)
- **Subscribers**: RViz

### `/cmd_vel_smoothed` (geometry_msgs/Twist)
- **Publishers**: amcl_navigation, nav2_navigation (velocity smoother)
- **Subscribers**: Topic Relay (relay_cmd_vel_smoothed)

### `/initialpose` (geometry_msgs/PoseWithCovarianceStamped)
- **Publishers**: RViz (manual pose setting)
- **Subscribers**: amcl_localization

### `/pose` (geometry_msgs/PoseWithCovarianceStamped)
- **Publishers**: amcl_localization
- **Subscribers**: amcl_navigation, nav2_navigation, RViz

### `/particle_cloud` (geometry_msgs/PoseArray)
- **Publishers**: amcl_localization
- **Subscribers**: RViz

### Transform Topics (`/tf`, `/tf_static`)
- **Publishers**: robot_state_publisher, simple_odometry.py, slam_online_async, slam_localization, amcl_localization
- **Subscribers**: All nodes requiring coordinate transformations

### Action Topics
#### `NavigateToPose` Action
- **Action Server**: amcl_navigation, nav2_navigation
- **Action Client**: nav_gui.py
