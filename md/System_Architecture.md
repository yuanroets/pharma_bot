# System_Architecture

This document provides a comprehensive breakdown of all nodes, files, and relationships for each launch state described in `System_Launches.md`. Use this as a reference to draw detailed software architecture diagrams for your pharma_bot project.

---

## Sim Launch
### Terminal 1: Launch simulation environment with relay node
**Command:**
```
ros2 launch pharma_bot launch_sim.launch.py
```
**Nodes/Files/Programs Launched:**
- `gazebo` (simulator): Simulates robot and environment.
- `relay_node` (pharma_bot): Relays topics between Gazebo and ROS2 ecosystem.
- `robot_state_publisher` (pharma_bot): Publishes robot transforms from URDF.
- `joint_state_publisher` (pharma_bot): Publishes joint states for visualization.
- `rviz2`: Visualization tool.

**Relationships:**
- `gazebo` publishes `/cmd_vel`, `/scan`, `/odom`, `/tf`.
- `relay_node` relays these topics to ROS2.
- `robot_state_publisher` and `joint_state_publisher` provide transforms and joint states for visualization in `rviz2`.
- `rviz2` subscribes to all visualization topics.

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
- `motor_driver` (serial_motor_demo/driver): Communicates with Arduino, controls motors.
- `teleop_bridge` (serial_motor_demo/teleop_bridge): Converts `/cmd_vel` to motor commands.
- `joint_state_bridge` (pharma_bot/joint_state_bridge.py): Publishes joint states for visualization.
- `simple_odometry` (serial_motor_demo/simple_odometry): Publishes odometry.
- `ldlidar_node` (ldlidar_node/ldlidar_bringup.launch.py): Publishes `/scan`.
- `lidar_lifecycle_manager` (pharma_bot/lidar_lifecycle_manager.py): Manages LiDAR node lifecycle.

**Relationships:**
- `teleop_bridge` subscribes to `/cmd_vel` (from Nav2 or teleop).
- `teleop_bridge` sends motor commands to `motor_driver`.
- `motor_driver` communicates with Arduino for motor control.
- `motor_driver` sends encoder data to `joint_state_bridge` and `simple_odometry`.
- `ldlidar_node` publishes `/scan` for SLAM/localization.
- `lidar_lifecycle_manager` ensures robust LiDAR operation.

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
