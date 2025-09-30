# Intro
This file contains the different launch sequences we have. I want to create a software architecture diagram for all the different ways of launchuing. The arcitecture should show which files interact with eachother in what way and what their relevance are. This means you will have to go through each of the launch files, and look at all the nodes being launched so that you can properly describe this. For example, when doing my normal dev_test_launch.py i also launch serial_motor_demo and with that i think the teleop_bridge node as well. This is so that cmd_vel and motor_vel can work together. You need to describe this for each of the diagrams. This is imperative, because it will essentially show the entire working of my program but for its different launch states.

# Sim Launch
## Terminal 1: Launch simulation environment with relay node
cd ~/dev_ws
source install/setup.bash
ros2 launch pharma_bot launch_sim.launch.py

## Mapping Mode
### Start SLAM for mapping
source install/setup.bash
ros2 launch slam_toolbox online_async_launch.py params_file:=/home/ubuntu/dev_ws/src/pharma_bot/config/mapper_params_online_async.yaml

## Localization with saved map
### Start SLAM localization with saved map
source install/setup.bash
ros2 launch slam_toolbox localization_launch.py slam_params_file:=/home/ubuntu/dev_ws/src/pharma_bot/config/mapper_params_online_async.yaml

### Start Nav2 navigation stack
source install/setup.bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true

## AMCL mode
ros2 launch pharma_bot localisation_launch.py map:=my_map_save.yaml use_sim_time:=true 
<!-- remember we are using the saved map now not serial -->
<!-- Also on rviz set durability to transient local and set initial pose-->
ros2 launch pharma_bot navigation_launch.py use_sim_time:=true map_subscribe_transient_local:=true

## Terminal 4: Launch RViz with SLAM configuration
source install/setup.bash
rviz2 -d /home/ubuntu/dev_ws/src/pharma_bot/config/slam2.rviz


# Hardware Launch
## On the Pi
ros2 launch pharma_bot pi_test_launch.py

## On the dev machine

## Mapping Mode
### Start SLAM for mapping
ros2 laucnh pharma_bot dev_test_launch.py
ros2 launch slam_toolbox online_async_launch.py params_file:=/home/ubuntu/dev_ws/src/pharma_bot/config/mapper_params_online_async.yaml (maybe just check if dev_test_launch does not do this in any way)

## Localization with saved map
### Start SLAM localization with saved map
ros2 laucnh pharma_bot dev_test_launch_localization.py
ros2 launch slam_toolbox localization_launch.py slam_params_file:=/home/ubuntu/dev_ws/src/pharma_bot/config/mapper_params_online_async.yaml

### Start Nav2 navigation stack
source install/setup.bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true

## AMCL mode
ros2 launch pharma_bot dev_test_amcl_launch.py
ros2 launch pharma_bot amcl_localization_launch.py
ros2 launch pharma_bot amcl_navigation_launch.py
<!-- remember we are using the saved map now not serial -->
<!-- Also on rviz set durability to transient local and set initial pose-->

# UI Launch
## Run the nav_gui Node
ros2 run nav_gui nav_gui.py

## Start the FastAPI GUI API Server
python3 nav_api.py

## Expose the API with ngrok
ngrok http 8000
- Copy the public ngrok URL (e.g., `https://xxxx.ngrok-free.dev`).
- In your web app, paste the URL in the settings and add `/navigate` to the end:
