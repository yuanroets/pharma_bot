# System_Architecture

This document provides a comprehensive breakdown of all nodes, files, and relationships for each launch state described in `System_Launches.md`. Use this as a reference to draw detailed software architecture diagrams for your pharma_bot project.

---

## Sim Launch
### Terminal 1: Launch simulation environment
**Command:**
```
ros2 launch pharma_bot launch_sim.launch.py
```
**Nodes/Files/Programs Launched:**
- `robot_state_publisher` (from rsp.launch.py): Publishes transforms from URDF, uses sim time and ros2_control.
- `joystick` (from joystick.launch.py): Launches joystick teleop node (see joystick.launch.py for details).
- `teleop_twist_keyboard` (teleop_twist_keyboard): Publishes `/diff_cont/cmd_vel_unstamped` for manual control.
- `cmd_vel_relay` (topic_tools/relay): Relays `/cmd_vel` to `/diff_cont/cmd_vel_unstamped` (connects Nav2 output to robot controller).
- `gazebo` (ros_gz_sim/gz_sim.launch.py): Simulates the robot and world.
- `spawn_entity` (ros_gz_sim/create): Spawns the robot in Gazebo using the URDF.
- `diff_drive_spawner` (controller_manager/spawner): Spawns the diff drive controller.
- `joint_broad_spawner` (controller_manager/spawner): Spawns the joint broadcaster.
- `ros_gz_bridge` (ros_gz_bridge/parameter_bridge): Bridges Gazebo topics to ROS2, uses `gz_bridge.yaml`.
- `ros_gz_image_bridge` (ros_gz_image/image_bridge): Bridges camera image topics.
- `robot.urdf.xacro` (pharma_bot/description): Robot model used for simulation and state publishing.
- `corridor.world` (pharma_bot/worlds): Gazebo world file.
- `gz_bridge.yaml` (pharma_bot/config): Topic bridge configuration.

**Relationships:**
- `gazebo` simulates the robot and publishes sensor and state topics.
- `spawn_entity` loads the robot model into Gazebo.
- `robot_state_publisher` publishes transforms from the URDF for visualization and control.
- `diff_drive_spawner` and `joint_broad_spawner` enable control and joint state feedback.
- `ros_gz_bridge` bridges topics (e.g., `/scan`, `/odom`, `/cmd_vel`) between Gazebo and ROS2.
- `ros_gz_image_bridge` bridges camera images.
- `teleop_twist_keyboard` allows manual velocity control, publishing to `/diff_cont/cmd_vel_unstamped`.
- `cmd_vel_relay` relays `/cmd_vel` (from Nav2 or other sources) to `/diff_cont/cmd_vel_unstamped` for robot control.
- `joystick` node provides joystick-based teleop, also publishing velocity commands.

**Topic Flow:**
- `/cmd_vel` (from Nav2 or teleop) → `cmd_vel_relay` → `/diff_cont/cmd_vel_unstamped` → diff drive controller in Gazebo.
- Sensor topics (`/scan`, `/odom`, `/tf`, `/camera/image_raw`) are bridged from Gazebo to ROS2 for use by SLAM, navigation, and visualization.
- `robot_state_publisher` and joint broadcaster provide transforms and joint states for RViz and controllers.

**Relevant Files:**
- `launch_sim.launch.py` (main simulation launch)
- `rsp.launch.py` (robot state publisher)
- `joystick.launch.py` (joystick teleop)
- `gz_bridge.yaml` (topic bridge configuration)
- `robot.urdf.xacro` (robot model)
- `corridor.world` (Gazebo world)

---

### Mapping Mode
**Command:**
```
ros2 launch slam_toolbox online_async_launch.py params_file:=/home/ubuntu/dev_ws/src/pharma_bot/config/mapper_params_online_async.yaml
```
**Nodes/Files/Programs Launched:**
- `slam_toolbox` (online_async_launch.py): Performs SLAM, publishes `/map`, `/odom`, `/tf`.
- Uses params file: `mapper_params_online_async.yaml`.

**Relationships:**
- Subscribes to `/scan` (from Gazebo or hardware).
- Publishes `/map`, `/odom`, `/tf` for localization and navigation.

---

### Localization with Saved Map
**Command:**
```
ros2 launch slam_toolbox localization_launch.py slam_params_file:=/home/ubuntu/dev_ws/src/pharma_bot/config/mapper_params_online_async.yaml
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
```
**Nodes/Files/Programs Launched:**
- `slam_toolbox` (localization_launch.py): Localizes robot using saved map.
- `nav2` (nav2_bringup): Navigation stack, publishes `/cmd_vel`.
- Uses params file: `mapper_params_online_async.yaml`.

**Relationships:**
- `nav2` subscribes to `/map`, `/odom`, `/tf` from `slam_toolbox`.
- `nav2` publishes `/cmd_vel` for robot movement.

---

### AMCL Mode
**Command:**
```
ros2 launch pharma_bot localisation_launch.py map:=my_map_save.yaml use_sim_time:=true
ros2 launch pharma_bot navigation_launch.py use_sim_time:=true map_subscribe_transient_local:=true
```
**Nodes/Files/Programs Launched:**
- `amcl` (localisation_launch.py): Probabilistic localization using saved map.
- `nav2` (navigation_launch.py): Navigation stack.
- Uses map file: `my_map_save.yaml`.

**Relationships:**
- `amcl` publishes `/tf`, `/map`, `/odom`.
- `nav2` subscribes to localization topics and publishes `/cmd_vel`.

---

### RViz with SLAM Configuration
**Command:**
```
rviz2 -d /home/ubuntu/dev_ws/src/pharma_bot/config/slam2.rviz
```
**Nodes/Files/Programs Launched:**
- `rviz2`: Visualization tool.
- Uses config file: `slam2.rviz`.

**Relationships:**
- Subscribes to all visualization topics for robot state, map, and sensor data.

---

## Hardware Launch
### On the Pi
**Command:**
```
ros2 launch pharma_bot pi_test_launch.py
```
**Nodes/Files/Programs Launched:**
- `motor_driver` (serial_motor_demo/driver): Communicates with Arduino via USB, sends motor commands, receives encoder feedback.
- `teleop_bridge` (serial_motor_demo/teleop_bridge): Subscribes to `/cmd_vel`, converts velocity commands to motor commands for differential drive.
- `joint_state_bridge` (pharma_bot/pharma_bot/joint_state_bridge.py): Converts encoder data to joint states for visualization.
- `simple_odometry` (serial_motor_demo/simple_odometry): Converts encoder data to odometry (`odom->base_link` transform) for SLAM/localization. (May be disabled if handled elsewhere.)
- `ldlidar_node` (Lidar/ldlidar_node/launch/ldlidar_bringup.launch.py): Publishes `/scan` topic from LiDAR sensor.
- `lidar_lifecycle_manager` (pharma_bot/pharma_bot/lidar_lifecycle_manager.py): Manages LiDAR node lifecycle, ensures robust startup and operation.
- `robot.urdf.xacro` (pharma_bot/description): Robot model for transforms and visualization.

**Relationships:**
- `teleop_bridge` subscribes to `/cmd_vel` (from Nav2 or teleop sources).
- `teleop_bridge` sends motor commands to `motor_driver`.
- `motor_driver` communicates with Arduino for direct motor control.
- `motor_driver` sends encoder data to `joint_state_bridge` (for joint states) and `simple_odometry` (for odometry).
- `joint_state_bridge` publishes joint states for visualization in RViz.
- `simple_odometry` publishes odometry for SLAM/localization.
- `ldlidar_node` publishes `/scan` for SLAM/localization.
- `lidar_lifecycle_manager` ensures LiDAR node starts and operates correctly.
- All transforms and joint states are based on the robot model (`robot.urdf.xacro`).

**Topic Flow:**
- `/cmd_vel` (from Nav2 or teleop) → `teleop_bridge` → motor commands → `motor_driver` → Arduino → Motors.
- Encoder data → `joint_state_bridge` → joint states for RViz.
- Encoder data → `simple_odometry` → `/odom` for SLAM/localization.
- `/scan` (from `ldlidar_node`) → SLAM/localization nodes.
- All relevant transforms and joint states are published for visualization and control.

**Relevant Files:**
- `pi_test_launch.py` (main hardware launch)
- `joint_state_bridge.py` (joint state conversion)
- `lidar_lifecycle_manager.py` (LiDAR lifecycle management)
- `ldlidar_bringup.launch.py` (LiDAR node launch)
- `robot.urdf.xacro` (robot model)

---

### On the Dev Machine
#### Mapping Mode
**Command:**
```
ros2 launch pharma_bot dev_test_launch.py
ros2 launch slam_toolbox online_async_launch.py params_file:=/home/ubuntu/dev_ws/src/pharma_bot/config/mapper_params_online_async.yaml
```
**Nodes/Files/Programs Launched:**
- All Pi nodes (if networked), plus:
- `slam_toolbox` (online_async_launch.py): Mapping.
- Uses params file: `mapper_params_online_async.yaml`.

**Relationships:**
- Same as Pi, but mapping and visualization run on dev machine.

---

#### Localization with Saved Map
**Command:**
```
ros2 launch pharma_bot dev_test_launch_localization.py
ros2 launch slam_toolbox localization_launch.py slam_params_file:=/home/ubuntu/dev_ws/src/pharma_bot/config/mapper_params_online_async.yaml
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
```
**Nodes/Files/Programs Launched:**
- All Pi nodes (if networked), plus:
- `slam_toolbox` (localization_launch.py): Localization.
- `nav2` (nav2_bringup): Navigation.
- Uses params file: `mapper_params_online_async.yaml`.

**Relationships:**
- `nav2` subscribes to `/map`, `/odom`, `/tf` from `slam_toolbox`.
- `nav2` publishes `/cmd_vel` to Pi for motor control.

---

#### AMCL Mode
**Command:**
```
ros2 launch pharma_bot dev_test_amcl_launch.py
ros2 launch pharma_bot amcl_localization_launch.py
ros2 launch pharma_bot amcl_navigation_launch.py
```
**Nodes/Files/Programs Launched:**
- `amcl` (amcl_localization_launch.py): Localization.
- `nav2` (amcl_navigation_launch.py): Navigation.
- All Pi nodes (if networked).

**Relationships:**
- `amcl` publishes localization topics.
- `nav2` subscribes to localization topics and publishes `/cmd_vel`.
- Pi nodes handle motor control and sensor feedback.

---

## UI Launch
### Run the nav_gui Node
**Command:**
```
ros2 run nav_gui nav_gui.py
```
**Nodes/Files/Programs Launched:**
- `nav_gui` (nav_gui/nav_gui.py): ROS2 node that subscribes to `/nav_goal` and sends navigation goals to Nav2.

**Relationships:**
- Subscribes to `/nav_goal` (published by FastAPI server).
- Sends navigation goals to Nav2 stack via `NavigateToPose` action client.

---

### Start the FastAPI GUI API Server
**Command:**
```
python3 nav_api.py
```
**Nodes/Files/Programs Launched:**
- `nav_api.py` (nav_gui/nav_api.py): FastAPI server exposing `/navigate` endpoint.

**Relationships:**
- Receives HTTP POST requests from the web app.
- Publishes `PoseStamped` messages to `/nav_goal` topic for nav_gui node.

---

### Expose the API with ngrok
**Command:**
```
ngrok http 8000
```
**Nodes/Files/Programs Launched:**
- `ngrok`: Tunnels local FastAPI server port to a public URL.

**Relationships:**
- Makes FastAPI `/navigate` endpoint accessible from anywhere via public URL.
- Allows hospital staff to use the web app from any device.

---

### Web App (Webhook)
**Usage:**
- Accessed via public ngrok URL.
- Sends navigation requests to FastAPI `/navigate` endpoint.

**Relationships:**
- User interface for entering coordinates and sending navigation requests.
- Device-agnostic; can be used from any browser.
- Interacts with FastAPI server, which interfaces with ROS2 nav_gui node.

---

## General Relationships (for all diagrams)
- **Nav2** always publishes `/cmd_vel` for movement.
- **teleop_bridge** converts `/cmd_vel` to motor commands.
- **motor_driver** sends commands to Arduino/motors and receives encoder feedback.
- **joint_state_bridge** and **simple_odometry** process encoder data for visualization and odometry.
- **slam_toolbox** or **amcl** provide localization and mapping.
- **ldlidar_node** provides LiDAR data for mapping/localization.
- **rviz2** visualizes all relevant topics.

---

Use this breakdown to draw architecture diagrams for each launch state, showing all nodes, files, and their relationships. Each bullet point can be a box in your diagram, and each relationship an arrow or connector.
