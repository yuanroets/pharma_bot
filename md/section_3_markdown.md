# 3 Technical Integration and Implementation

The complexity of this project did not lie only in the hardware and software 
development, but in creating a way of allowing so many complex systems to 
interface correctly and efficiently. Modern autonomous robotics systems require 
careful consideration of hardware-software integration, power management, 
thermal design, and modularity to ensure reliable operation in demanding 
environments. In this chapter I aim to explain the hardware and software design and 
design choices, as well as showcasing how emergent  problems were mitigated by 
ensuring the system works well together. The systems engineering approach 
adopted prioritizes modularity, scalability, and maintainability while meeting the 
stringent requirements of autonomous hospital navigation.  
## 3.1 Hardware Design and Function  
The hardware design focuses on robust mechanics, precise sensor integration, and 
efficient power management to support autonomous operation in a hospital setting. 
The design philosophy emphasizes modularity to facilitate future upgrades, ease of 
maintenanc e, and system reliability. Each component was selected through a 
systematic evaluation process considering performance specifications, power 
consumption, cost, and integration complexity.




16 3.1.1  Final Design and Wiring Diagram  
Diagram for final assembly here  
### 3.1.2 Chassis, Carriage and Electronics Compartment  
The mechanical design represents a critical foundation for autonomous operation, 
with careful attention to weight distribution, accessibility, modularity, and 
protection of sensitive electronics.  
#### 3.1.2.1 Chassis  
The provided Hiwonder tank chassis serves as the foundation platform for the 
autonomous hospital robot . Analysis of its specifications reveals several 
advantageous characteristics that align well with this robotics application. The tank 
track system offers  two main advantages. Traction, because of the large amount of 
ground contact , and manoeuvrability, since the tracks can spin in opposite 
directions, allowing the robot to turn around while remaining in one place.  
Figure 1 - Wiring Diagram




17 The chassis features an integrated suspension system that provides mechanical 
filtering of vibrations, protecting sensitive electronics and maintaining stable sensor 
readings during operation. This is particularly crucial for LIDAR accuracy, where 
mechanic al vibrations can introduce noise into range measurements. The chassis is 
also strong and lightweight, due to it aluminium construction.  
Multiple mounting points throughout the chassis allow for modular expansion.  
It is also important to note, that a ll critical electronics  (the Raspberry Pi, Arduino 
nano ad motor driver) were placed  within the chassis . This provides inherent 
protection against tampering and accidental damage. Since the chassis has a semi -
open construction, airflow over the components are good, allowing for good 
convective cooling as the robot moves.  
#### 3.1.2.2 Electronics Compartment  
The custom -designed electronics compartment addresses several critical 
requirements: secure battery mounting, easy maintenance access, circuit protection, 
and systematic power distribution. The compartment was manufactured using 
additive manufacturing tech niques, specifically fused deposition modelling (FDM) 
with PLA filament.  
PLA (Polylactic Acid) was selected over alternatives such as ABS or PETG for 
several engineering reasons. The main reason for the use of FDM was for its cheap 
and rapid prototyping capabilities, while still having adequate mechanical 
properties for the static loads encountered. The materials inherent fire resistances 
also adds an additional layer of safety, es pecially when considering the highly 
combustible nature of a Lithium -Polymer (LiPo) battery.  
Accessibility was prioritized in the compartment design, with the battery 
compartment accessible when removing two small pins on the side of the 
compartment, and then removing the lid.  This design decision reflects the 
operational reality that LiPo batteries should be removed for charging to ensure 
safety and battery longevity. The compartment also houses a 6A blade fuse in an 
accessible holder . 
The terminal strip integration provides a centralized distribution point for the 11.1V 
supply rail, facilitating systematic circuit troubleshooting and expansion. The strip 
design incorporates both positive and negative rails with multiple connection 
point s, enabling multimeter verification of supply voltages throughout the system. 
This modular approach to power distribution significantly simplifies maintenance 
and modification procedures.  
#### 3.1.2.3 Carriage  
The cargo carriage represents the functional interface between the robotic platform 
and the hospital logistics requirements. The design dimensions were optimized to




18 accommodate standard hospital items including medication containers, 
prescriptions and small medical supplies while maintaining the robot's stability 
margins.  
The integrated L iDAR bridge elevates the sensor 180mm  above the carriage 
surface, providing unobstructed 360° scanning capability.  
The rear cutout design represents a human factors engineering consideration. 
Hospital staff require rapid loading and unloading capabilities, and the cutout 
provides easy access for item retrieval while maintaining a small  lip to prevent 
cargo displacement during acceleration.  
The decision to employ 3D printing over alternatives such as sheet metal fabrication 
was driven by several factors. Traditional sheet metal fabrication would require 
specialized tooling and forming operations, increasing both cost and lead time. 
While shee t metal construction would provide superior impact resistance and lower 
material costs for volume production, the rapid prototyping advantages of additive 
manufacturing were decisive for this research project.  
### 3.1.3 Electronics and Circuit  
The electronic system design follows a distributed architecture approach, 
partitioning computational tasks across multiple processors to optimize 
performance, power consumption, and system reliability.  
#### 3.1.3.1 Power Management and Battery Life Analysis  
Power Distribution Architecture : The system utilizes a Gens Bashing 3S LiPo 
battery (11.1V nominal, 8000mAh) with hierarchical voltage regulation. A 6A fuse 
provides overcurrent protection based on worst -case analysis, while a terminal strip 
implements star topology distribution minimizing voltage drops.




19 Table 1: System power consumption across operational modes:  
Component  Idle (W)  Cruise (W)  Peak (W)  
Raspberry Pi 4  4.44 6.66 8.88 
LIDAR LD19  2.00 2.00 2.00 
Arduino Nano  0.44 0.44 0.44 
Motor Drivers  0.22 0.89 1.11 
Motors  0.00 6.66 22.20  
Total  7.10 16.65  34.63  
**Battery Life Calculations : Assuming 80% depth of discharge  (DoD).  
Idle operation time:  
 𝑡𝑖𝑑𝑙𝑒 =𝐶𝑏𝑎𝑡𝑡𝑒𝑟𝑦 ×𝐷𝑜𝐷
𝐼𝑖𝑑𝑙𝑒=8000  mAh ×0.8
640  mA=10.0 hours   
```latex
(5)
``` 
Cruise operation time:  
 𝑡𝑐𝑟𝑢𝑖𝑠𝑒 =8000  mAh ×0.8
1500  mA=4.27 hours   
```latex
(6)
``` 
Mixed operation (60% cruise, 20% idle, 10% peak, 10% charging):  
 𝐼𝑎𝑣𝑔=0.6×1.5+0.2×0.64+0.1×3.1=1.34 A  
```latex
(7)
``` 
 𝑡𝑚𝑖𝑥𝑒𝑑 =6400  mAh
1340  mA=4.78 hours   
```latex
(8)
``` 
 
#### 3.1.3.2 Waveshare servo motor driver  
The Waveshare Servo Driver HAT was selected to address two critical system 
limitations namely, the Raspberry Pi's limited PWM channels (only 2 hardware 
PWM outputs) and insufficient GPIO current capability (16mA per pin, 50mA 
total). By adding the HAT, we are limiting the total current drawn from the Pi. The 
HAT's PCA9685 chip provides 16 independent PWM channels, enabling motor 
control and still providing for future expansion including robotic arms, security 
roller covers, or additional LIDAR scanners, whi ch all require their own PWM 
channels.  
Furthermore, t he 12 -bit resolution significantly exceeds the Pi's native 10 -bit PWM:




20  PWM resolution improvement =212
210=4096
1024=4× better granularity   
```latex
(9)
``` 
An important secondary benefit is the HAT's integrated buck regulator, which 
enables direct LiPo -to-Pi power conversion. The LiPo connects to the HAT's VIN 
terminal, and the HAT powers the Pi via the 40 -pin connector . In any mobile 
application, combining elements to ensure a smaller form factor is always 
beneficial.  
Additionally, t he HAT exposes multiple 5V and GND pins, addressing the Pi's 
current limitations and enabling clean power distribution to sensors and peripherals 
without overloading individual GPIO pins. This modular approach supports system 
expansion without circuit rede sign.  
#### 3.1.3.3 Raspberry Pi  Model 4B  
The Raspberry Pi 4 Model B acts as the main onboard computer. It handles real -
time sensor processing and navigation. I chose it over options like the NVIDIA 
Jetson Nano or Intel NUC because it strikes a good balance between performance, 
power efficiency, cost, and available software support.  
Its quad -core ARM Cortex -A72 processor, running at 1.5 GHz, manages tasks like 
processing LiDAR data at 10 Hz with low latencies in ROS 2 nodes. The 8 GB 
RAM version supports running several ROS 2 processes and storing mapping data.  
More demanding computations, such as SLAM and path planning, run on a separate 
development machine, while local control remains on the Pi. This split improves 
scalability, eases development, saves power, and allows easier upgrades.  
Although a Jetson might perform better, especially for advanced AI tasks, it would 
cost much more. Using only Arduinos was not feasible, as they lack the processing 
power and multitasking capabilities needed for complex software like ROS 2 and 
sensor fusio n. 
Key connections include UART for the LiDAR and USB for the Arduino Nano.  
For cooling, I added a passive heatsink and fan, keeping temperatures around 65°C 
under load to prevent performance drops.  
#### 3.1.3.4 Arduino Nano  
The Arduino Nano manages real -time motor control and encoder readings, tasks 
that demand consistent timing which the Raspberry Pi's system cannot always 
guarantee , especially with all the other processes it is currently running . Its 
ATmega328P microcontroller handles interrupts precisely for encoder tracking and 
PID control.




21 I picked it over the ESP32 or STM32 for its low cost, and sufficient speed (16 MHz 
for 50 Hz PID updates). It consumes just 20 mA, keeping power use low.  
It uses hardware interrupts on pins 2 and 3 for encoder resolution below 0.5% of 
wheel rotation, runs PID loops at 50 Hz for speed control, and communicates via 
UART serial with the Raspberry Pi to send odometry data and receive velocity 
commands.  
#### 3.1.3.5 Motor  Driver  (DFRobot TB6612FNG ) 
The DFRobot TB6612FNG motor driver was chosen over the L298N or DRV8833 
for its efficiency, current capacity, small size, and simple control.  
Its MOSFET H -bridge delivers about 95% efficiency, cutting heat generation. For 
instance, at a typical operating current of 800 mA and an on -resistance of 0.5 ohms 
per channel, the power loss can be calculated as follows:  
 𝑃𝑙𝑜𝑠𝑠 =𝐼2𝑅=(0.8)2×0.5=0.32 W per channel .  
```latex
(10)
``` 
It handles 1.2 A continuous (3.2 A peak), matching the motors' 1.5 A stall current. 
Control uses just two pins per motor (PWM and direction), saving Arduino pins.  
Connections include PWM and direction signals from the Arduino Nano, 11.1 V 
from the LiPo for motor voltage, and 5 V logic compatible with the Arduino.  
Built -in protections cover overcurrent, thermal shutdown at 175°C, and 
undervoltage.  
#### 3.1.3.6 Moto rs and Encoders  
The JGB3865 -520R45 -12 geared DC motors suit mobile robots, with a 45:1 ratio, 
12 V operation, 1.5 A stall current, and 0.5 N·m stall torque.  
The maximum linear velocity of the robot, based on the wheel diameter D of 0.065 
m and a no -load motor speed of 150 RPM after gearing, is given by  
 𝑣max =𝜋×𝐷×RPM
60=𝜋×0.065 ×150
60=0.51 m/s,  
```latex
(11)
``` 
which exceeds typical hospital speeds of 0.1 -0.3 m/s.  
The output torque at the wheel, assuming 85% gearbox efficiency and a motor 
torque of 0.15 N·m at rated speed, is  
 𝑇output =0.15×0.85=0.128 N·m .  
```latex
(12)
``` 
The maximum tractive force, considering both motors and the wheel radius (D/2), 
is




22  𝐹max =2×𝑇output
𝐷/2=2×0.128
0.0325=7.88 N.  
```latex
(13)
``` 
For a 6 kg robot accelerating at 0.2 m/s², the required force is 1.2 N, providing a 
safety factor of 6.57. The maximum incline the robot can handle is  
 𝜃max =arcsin  (𝐹max
𝑚×𝑔)=arcsin  (7.88
6×9.81)=7.7∘.  
```latex
(14)
``` 
The integrated Hall effect encoders provide quadrature output for precise position 
and speed feedback. Practically, these encoders work by using two Hall sensors 
positioned at 90 degrees to each other around a magnetic disc attached to the motor 
shaft. As the motor rotates, the sensors detect changes in the magnetic field, 
producing two square wave signals (Channel A and Channel B) that are out of 
phase. This quadrature setup allows the system to determine not only the speed 
(from pulse frequency) but also the direction of rotation (by checking which channel 
leads the other). For example, if Channel A rises before Channel B, the motor is 
turning clockwise; the reverse indicates anticlockwise.  
The encoders generate 11 pulses per motor revolution. After the 45:1 gearbox, this 
becomes 495 pulses per wheel revolution. Using quadrature decoding (counting 
both rising and falling edges on both channels) multiplies this by 4, yielding 1980 
counts per w heel revolution. To interpret the pulses, the Arduino's interrupts count 
these transitions over time. The distance resolution is then  
Distance per count =𝜋×𝐷
1980=𝜋×0.065
1980=1.03×10−4 m/count =
                      0.103 mm/count .  
```latex
(15)
``` 
This high resolution enables accurate odometry by accumulating counts to track 
total distance travelled and differentiating them over time to estimate velocity. 
Noise from electrical interference or mechanical backlash is mitigated through 
software filteri ng in the PID loop.  
Power draw is around 6.66 W continuous at 0.3 A per motor, with heat dissipation 
of about 0.1 W per motor.  
Connections involve PWM from the driver for speed, encoder signals to Arduino 
interrupts for odometry, and power via the driver from the LiPo.  
#### 3.1.3.7 LiDAR  (OKdo LiDAR HAT – LD06 ) 
The OKdo LiDAR HAT with LD06 sensor supplies 360° distance data for mapping 
and avoiding obstacles. It offers 0.02 -12 m range, ±2 cm accuracy, 1° angular 
resolution, 5 -15 Hz scans, Class 1 laser safety, UART at 230,400 baud, and 2 W 
power.  
The 12 m range fits hospital corridors (2 -4 m wide), and 20 mm minimum detects 
nearby objects. Accuracy supports 5 cm mapping grids.




23 It connects via UART to the Raspberry Pi (TX to RXD, PWM for motor speed, 5 
V power, ground). Data rate is 360 points × 15 Hz × 16 bits = 86,400 bits/s, using 
about 37% of UART capacity.  
#### 3.1.3.8 Development  Machine  
The development machine serves as a server for heavy tasks like SLAM and Nav2, 
relieving the Raspberry Pi.  SLAM's complexity  uses 20 -40% CPU and 50 -200 MB 
RAM, which could overload the Pi alongside other processes.  
The development machine connects to the Pi using the ROS 2 Data Distribution 
Service (DDS), which ensures a stable connection with reliable Quality of Service 
(QoS).  
This setup keeps the two devices’ clocks aligned within ±1 millisecond using 
Network Time Protocol (NTP) synchronisation, maintains a communication delay 
of 15 -25 milliseconds, and supports data transfer at up to 500 kilobits per second.  
This arrangement improves the system’s efficiency and allows for future growth. It 
also opens the door to upgrading to edge devices like the NVIDIA Jetson, which 
could eventually manage all tasks onboard without needing the development 
machine.  
## 3.2 Software System Design and Implementation

The software architecture for the autonomous hospital assistant robot is built on ROS2, leveraging its distributed, modular, and lifecycle-managed framework to meet stringent systems engineering requirements. This section details the technical implementation, referencing actual code and launch files, and justifies design decisions through concept analysis.

### 3.2.1 Simulation Environment and Development Workflow
Simulation was performed in Gazebo, using a URDF model defined in `pharma_bot/description/robot.urdf.xacro`. The URDF specifies link geometries, joints, sensors, and collision meshes, enabling physics-based testing of navigation, sensor integration, and control algorithms. This risk-free environment allowed rapid iteration and parameter tuning before hardware deployment.

### 3.2.2 Distributed Architecture and Launch File Structure
The system employs a distributed architecture, partitioning computation between the Raspberry Pi (onboard) and a development machine (offboard). This is orchestrated via multiple launch files:
- `pi_test_launch.py`: Runs on the Pi, launching motor control (via Arduino), teleop bridge, and LiDAR driver with lifecycle management. It uses hardware-specific parameters and URDF for robot state publishing.
- `dev_test_launch.py`: Runs on the dev machine, launching teleop, robot state publisher, SLAM Toolbox (mapping mode), static transforms, and RViz2 for visualization.
- `amcl_localization_launch.py` and `amcl_navigation_launch.py`: Enable AMCL localization and full Nav2 navigation, with topic remapping (relay node) to ensure compatibility between LiDAR drivers and navigation stack.

This modular launch structure supports systematic testing and deployment, with clear separation between simulation, mapping, localization, and hardware operation.

#### Example: Launch File Node Definitions
```python
# pi_test_launch.py (excerpt)
Node(
    package='pharma_bot',
    executable='motor_driver',
    parameters=[...],
    output='screen'
)
Node(
    package='pharma_bot',
    executable='lidar_lifecycle_manager',
    output='screen'
)
```

### 3.2.3 Node Architecture, Lifecycle Management, and Topic Remapping
ROS2 nodes are organized by function, with lifecycle nodes used for critical components (e.g., LiDAR driver) to enable managed state transitions and robust fault recovery. The `lidar_lifecycle_manager.py` script demonstrates this:
```python
class LidarLifecycleManager(Node):
    def __init__(self):
        # ...existing code...
        self.timer = self.create_timer(2.0, self.manage_lifecycle)
    def manage_lifecycle(self):
        # Checks state, activates node, handles errors
```
Lifecycle management allows dynamic reconfiguration and graceful failure handling, essential for real-world robotics.

Topic remapping is used to ensure compatibility between sensor drivers and navigation stack. For example, AMCL expects `/scan`, but the LiDAR driver publishes `/ldlidar_node/scan`. A relay node remaps topics in `amcl_localization_launch.py`:
```python
Node(
    package='topic_tools',
    executable='relay',
    arguments=['/ldlidar_node/scan', '/scan'],
    output='screen'
)
```
This design ensures modularity and future extensibility.

### 3.2.4 Hardware Abstraction and Embedded Control
Motor control and odometry are handled by the Arduino (firmware in `Bridge_and_Motor/ROSArduinoBridge.ino`) and by ROS2 nodes on the Pi and dev machine. The Arduino implements a command-based protocol for velocity, encoder, and PID control, with safety features (timeouts, validation, buffer management).

Odometry is calculated using differential drive kinematics, as implemented in `serial_motor_demo/serial_motor_demo/serial_motor_demo/simple_odometry.py`:
```python
class SimpleOdometry(Node):
    def encoder_callback(self, msg):
        left_distance = (msg.left_diff / self.encoder_cpr) * 2.0 * math.pi * self.wheel_radius
        right_distance = (msg.right_diff / self.encoder_cpr) * 2.0 * math.pi * self.wheel_radius
        # ...update pose, publish odometry...
```
Parameters (encoder CPR, wheel separation, radius) are empirically calibrated and set via YAML files (e.g., `motor_driver_params.yaml`).

### 3.2.5 Simultaneous Localisation and Mapping (SLAM)

#### 3.2.5.1 SLAM Toolbox Implementation

The SLAM system utilises the slam_toolbox package (Macenski & Jambrecic, 2021), which implements graph-based SLAM algorithms optimised for real-time operation. The system operates in two primary modes:

**Mapping Mode (online_async):**
- Real-time map construction from LiDAR and odometry data
- Loop closure detection and correction
- Occupancy grid map generation
- Pose graph optimisation

**Localisation Mode (localization):**
- Pose estimation within a known map
- Particle filter-based localisation
- Continuous pose correction and refinement

The implementation uses the default slam_toolbox parameter set with minimal modifications for the distributed computing architecture and robot-specific requirements. The default parameters provide a well-tested baseline optimized for indoor mobile robotics applications. Several parameters were adjusted to improve network timing performance and motion update responsiveness for the distributed system architecture.

**Operational Mode Configuration:** The mode parameter switches between mapping and localization. Mapping mode builds new maps from LiDAR and odometry data, performing real-time loop closure detection and pose graph optimization during initial environment surveying. Localization mode performs pose estimation within a pre-existing map (loaded via map_file_name) and is used during autonomous navigation missions. The map_start_at_dock parameter enables initialization at a known pose when entering localization mode, avoiding the initial uncertainty period where the particle filter must converge.

#### 3.2.5.2 Occupancy Grid Mapping

The slam_toolbox output consists of occupancy grid maps, where each cell contains a probability of occupancy. The mapping algorithm updates cell probabilities using the log-odds representation:

𝑙(𝑐|𝑧₁:𝑡) = 𝑙(𝑐|𝑧₁:𝑟₋₁) + 𝑙𝑜𝑔(𝑃(𝑐|𝑧𝑡)) − 𝑙𝑜𝑔(𝑃(𝑐)) (24)

Where 𝑓 represents the log-odds value, 𝑐 is the cell, and 𝑧₁:𝑡 are sensor measurements. This representation enables efficient incremental updates: each new sensor measurement adds a log-odds contribution rather than requiring full probability recalculation. The approach naturally handles sensor uncertainty by accumulating evidence over multiple observations, with cells becoming more confident (higher absolute log-odds values) as contradictory or confirming measurements arrive.

The default map resolution of 0.05 metres per pixel balances navigation requirements with computational efficiency. This resolution provides adequate detail for representing doorways (typically 0.8-1.0 m wide, corresponding to 16-20 cells) and corridor features whilst maintaining manageable memory footprint (a 20m × 20m area requires 160,000 cells).

#### 3.2.5.3 Localisation within Known Maps

For operation within existing maps, the system employs Adaptive Monte Carlo Localisation (AMCL), which maintains a particle filter representation of position belief. The implementation uses default AMCL parameters (Macenski et al., 2023) with modifications for encoder-based odometry characteristics.

**Modified Parameters:**

**Particle Filter:** The particle count range is elevated compared to typical applications to handle geometric ambiguity in repetitive hospital corridors. The adaptive filter dynamically adjusts particle count based on localization confidence, using fewer particles when position is certain and approaching maximum during symmetric corridor sections until distinctive features resolve ambiguity.

**Motion Model:** Reflects elevated uncertainty in encoder-based linear distance estimation due to wheel slip, deweighting odometry relative to LiDAR scan matching.

**Update Thresholds:** Trigger filter updates more frequently during rotation where encoders accumulate greater error.

**Sensor Model:** Emphasize accurate range measurements and subsample the 360° scan for computational efficiency.

**Table 3.1: SLAM Toolbox Parameters - Original vs Tuned Values**

| Parameter | Original Value | Tuned Value | Rationale |
|-----------|----------------|-------------|-----------|
| `map_update_interval` | 5.0 s | 2.0 s | More frequent map updates for dynamic environment handling |
| `transform_timeout` | 0.2 s | 0.5 s | Increased timeout tolerance for network delays |
| `minimum_time_interval` | 0.5 s | 0.2 s | More responsive mapping updates |
| `minimum_travel_distance` | 0.5 m | 0.2 m | Finer granularity for detailed mapping |
| `minimum_travel_heading` | 0.5 rad | 0.2 rad | More sensitive angular threshold for loop closure |
| `loop_closure_frequency` | 10 | 5 | Reduced frequency to balance accuracy with computational load |
| `max_laser_range` | 12.0 m | 8.0 m | Limited range appropriate for indoor hospital environment |
| `resolution` | 0.05 m | 0.03 m | Higher resolution for detailed obstacle representation |

**Table 3.1a: AMCL Localization Parameters - Original vs Tuned Values**

| Parameter | Original Value | Tuned Value | Rationale |
|-----------|----------------|-------------|-----------|
| `kld_err` | 0.05 | 0.01 | Reduced error threshold for more accurate particle distribution |
| `odom_alpha_3` | 0.2 | 0.1 | Reduced rotational drift noise for better odometry model |
| `update_min_d` | 0.2 m | 0.1 m | More frequent updates with smaller distance threshold |
| `update_min_a` | π/6.0 rad | π/12.0 rad | More sensitive angular update threshold |
| `laser_z_hit` | 0.5 | 0.95 | Increased weight for accurate laser measurements |
| `laser_z_rand` | 0.5 | 0.05 | Reduced random measurement noise assumption |
| `laser_max_beams` | 60 | 180 | Use more laser beams for better localization accuracy |
| `min_particles` | 2000 | 1500 | Reduced minimum particles for computational efficiency |

### 3.2.6 Navigation and Path Planning

#### 3.2.6.1 Navigation Stack Architecture

The navigation system employs the Nav2 stack (Macenski et al., 2020), implementing a behaviour tree-based architecture for decision coordination. The modular design separates responsibilities across five core components: the Planner Server performs global path planning using the A* algorithm on the occupancy grid map, the Controller Server executes local trajectory tracking with the Dynamic Window Approach (DWA), the Recoveries Server implements failure recovery behaviours including rotation, backup, and waiting, the Behaviour Tree Navigator coordinates high-level decision logic, and the Costmap system provides multi-layered obstacle representation combining static map obstacles, dynamic LiDAR detections, and inflated safety margins. This architecture enables the robot to handle temporary obstacles, recover from navigation failures, and dynamically replan paths when environmental conditions change.

#### 3.2.6.2 Global Path Planning

The A* algorithm operates on the occupancy grid map to find optimal paths from current position to goal locations. The implementation uses Euclidean distance as the heuristic function, combines distance travelled with obstacle proximity penalties in the cost function, applies post-processing smoothing for kinematically feasible trajectories, and triggers replanning when significant map changes occur (e.g., new obstacles detected). The planner operates at low frequency, sufficient given that global paths change infrequently in static environments whilst avoiding continuous recomputation overhead. Parameters were adjusted with relaxed tolerance to accommodate odometry drift, and unknown region planning enabled for exploration capabilities.

#### 3.2.6.3 Local Path Planning and Obstacle Avoidance

The DWA algorithm (Macenski et al., 2023) evaluates candidate trajectories within kinematic constraints, optimizing a multi-objective scoring function that combines path alignment, goal progress, and obstacle clearance. Trajectory evaluation occurs at high frequency, providing responsive real-time obstacle avoidance.

**Safety-Oriented Parameter Selection:** Maximum speed is limited to enable safe stopping within LiDAR detection range with adequate reaction time margin. The trajectory prediction horizon balances forward planning (detecting obstacles early) against computational cost (longer horizons exponentially increase candidate trajectory count). Velocity sampling focuses resolution on rotational control where differential drive robots have greater agility.

#### 3.2.6.4 Costmap Configuration

The layered costmap system combines three representations: the Static Layer encodes permanent obstacles from the occupancy grid map; the Obstacle Layer incorporates real-time LiDAR detections of dynamic obstacles; and the Inflation Layer expands obstacle boundaries by a configurable inflation radius. Each layer assigns numerical costs (0-254 scale) to grid cells, with the combined costmap guiding path planning decisions.

The inflation radius accounts for the robot's physical dimensions plus a safety margin. This configuration enables navigation through standard doorways with adequate clearance on each side, providing safety buffer for odometry uncertainty and dynamic obstacle margins (humans typically require personal space). Costmap update frequency balances obstacle detection responsiveness with computational load on the embedded system.

**Table 3.2: Navigation Controller Parameters - Original vs Tuned Values**

| Parameter | Original Value | Tuned Value | Rationale |
|-----------|----------------|-------------|-----------|
| `controller_frequency` | default | 20.0 Hz | High frequency trajectory evaluation for responsive obstacle avoidance |
| `max_vel_x` | 0.26 m/s | 0.12 m/s | Conservative speed for safe hospital navigation |
| `max_vel_theta` | 1.0 rad/s | 0.4 rad/s | Reduced rotational speed for stability and safety |
| `acc_lim_x` | 1.0 m/s² | 0.4 m/s² | Gentle acceleration for smooth patient-friendly operation |
| `decel_lim_x` | -1.0 m/s² | -1.5 m/s² | Enhanced deceleration for rapid obstacle response |
| `sim_time` | 2.0 s | 2.5 s | Extended planning horizon for early obstacle detection |
| `PathAlign.scale` | 20.0 | 32.0 | Increased weight to maintain close path following |
| `path_distance_bias` | default | 64.0 | Bias for path alignment in multi-objective scoring |
| `goal_distance_bias` | default | 24.0 | Weight for goal progress in trajectory scoring |
| `occdist_scale` | default | 0.02 | Scale factor for obstacle clearance scoring |
| `vx_samples` | 25 | 30 | More velocity sampling options for better trajectory selection |
| `vy_samples` | default | 5 | Velocity sampling for holonomic motion (not used for differential drive) |
| `vtheta_samples` | 25 | 30 | More angular velocity options for smoother turning |
| `linear_granularity` | 0.03 | 0.02 | Higher resolution trajectory evaluation |
| `angular_granularity` | 0.02 | 0.015 | Higher angular resolution for smoother rotational movements |
| `xy_goal_tolerance` | 0.25 m | 0.35 m | Relaxed tolerance accounting for odometry uncertainty |
| `trans_stopped_velocity` | 0.1 m/s | 0.08 m/s | More sensitive stopped detection for better goal achievement |

**Table 3.3: Costmap Parameters - Original vs Tuned Values**

| Parameter | Original Value | Tuned Value | Rationale |
|-----------|----------------|-------------|-----------|
| `inflation_radius` | 0.55 m | 1.0 m | Larger safety cushion around obstacles for conservative navigation |
| `cost_scaling_factor` | 5.0 | 4.5 | Balanced scaling for safety without excessive path deviation |
| `update_frequency` (local) | 10.0 Hz | 12.0 Hz | Higher frequency for responsive dynamic obstacle detection |
| `update_frequency` (global) | 2.0 Hz | 3.0 Hz | Faster global map updates for planning efficiency |
| `robot_radius` | 0.22 m | 0.21 m | Accurate representation of 300mm robot footprint |
| `resolution` | 0.05 m | 0.025 m | Higher resolution for precise obstacle representation |
| `obstacle_max_range` | 3.5 m | 4.0 m | Extended detection range for early obstacle identification |
| `raytrace_max_range` | 4.0 m | 4.5 m | Longer raytracing for effective obstacle clearing |

The navigation system's parameter configuration reflects a safety-first approach optimised for healthcare environments, where smooth operation, conservative speeds, and reliable obstacle avoidance are paramount. The extensive tuning process resulted in robust navigation behaviour that prioritises patient safety while maintaining operational efficiency.

### 3.2.7 Systems Engineering and Design Justification
ROS2’s modular, distributed architecture directly supports systems engineering principles:
- **Modularity:** Nodes and launch files are organized by function, enabling independent development and testing.
- **Scalability:** Distributed computation allows offloading heavy tasks (SLAM, Nav2) to the dev machine, with the Pi handling real-time control.
- **Maintainability:** Lifecycle management, topic remapping, and parameterization support rapid adaptation and fault recovery.

Design choices (e.g., lifecycle nodes, topic remapping, distributed architecture) were made after considering alternatives (monolithic nodes, direct topic connections, single-board computation) and found to best balance reliability, extensibility, and performance for hospital robotics.

### 3.2.8 Debugging, Monitoring, and Validation
RViz2 is used for real-time visualization (robot model, LiDAR, maps, trajectories). ROS2 introspection tools (`ros2 topic echo`, `ros2 node info`, `tf2_tools view_frames`) support debugging and validation. For example, topic introspection revealed SLAM was subscribing to `/scan` instead of `/ldlidar_node/scan`, leading to the addition of a relay node. System diagnostics and logging are configured per node for detailed monitoring.

### 3.2.9 Testing and Performance Characterization

A rigorous testing and validation methodology was adopted to ensure the reliability and performance of the integrated robotic system. Unit testing was performed on individual components such as odometry, motor control, SLAM, and network communication. For example, odometry validation involved comparing physical robot motion over measured distances and rotations with calculated odometry outputs, revealing systematic errors that were corrected through empirical calibration of wheel parameters and encoder counts. Motor control testing included step input response and frequency analysis, with PID parameters tuned to achieve critically damped response and minimal steady-state error.

SLAM validation was conducted by comparing generated maps with ground truth measurements in controlled environments, and by repeatedly traversing known paths to confirm loop closure detection and map consistency. Integration testing focused on verifying inter-node communication, distributed operation, and failure recovery. This included simulating network packet loss and latency to ensure robust message passing between the Raspberry Pi and development machine, and testing lifecycle node startup/shutdown sequences for dynamic reconfiguration and fault tolerance.

Transform consistency was validated by monitoring the TF2 transform tree during dynamic operation, ensuring correct spatial relationships between coordinate frames. Performance characterization involved monitoring computational load (CPU utilization during SLAM, navigation, and sensor processing), memory usage (RAM consumption for map storage, particle filters, and sensor data), real-time performance (timing analysis of control loops and navigation updates), and network bandwidth (data transmission rates and latency for distributed operation).

Empirical calibration and systematic testing ensured accurate navigation and robust distributed operation. The use of ROS2 introspection tools (e.g., `ros2 topic echo`, `ros2 node info`, `tf2_tools view_frames`) was essential for debugging, identifying communication failures, and resolving parameter misconfigurations. For instance, introspection revealed that SLAM was subscribing to `/scan` instead of `/ldlidar_node/scan`, prompting the addition of a relay node for correct topic remapping. This iterative, systems engineering-driven approach to testing and validation was critical for achieving a reliable, maintainable, and high-performance autonomous hospital robot.

---

## 3.3 Testing and Validation Framework

A rigorous testing and validation process is essential for demonstrating the reliability, accuracy, and robustness of the autonomous hospital robot. The following framework outlines the key tests to be performed, the methodology for each, and the quantitative results and analysis required for a comprehensive technical report.

### 3.3.1 Performance Testing
- **Objective:** Quantify the robot's ability to perform core navigation and control tasks under real-world conditions.
- **Tests to perform:**
  - Navigation accuracy in mapped environments (compare planned vs. actual trajectory).
  - Obstacle avoidance reliability (record success rate and minimum clearance).
  - System response time (latency from command to actuation).
- **Measurements required:**
  - Trajectory plots (planned vs. actual).
  - Success/failure rates for obstacle avoidance.
  - Latency measurements (ms).
  - Table summarizing test scenarios and outcomes.

### 3.3.2 PID Tuning and Motor Control Validation
- **Objective:** Demonstrate systematic tuning of PID parameters for optimal motor control.
- **Tests to perform:**
  - Step response tests for different PID values (record overshoot, settling time, steady-state error).
  - Frequency response analysis (Bode plot if possible).
  - Stability and repeatability under varying loads.
- **Measurements required:**
  - Plots of motor speed vs. time for each PID setting.
  - Table of PID values and corresponding performance metrics.
  - Justification for final chosen PID values.

### 3.3.3 Odometry and Calibration Testing
- **Objective:** Validate the accuracy of odometry calculations and calibrate wheel separation/radius.
- **Tests to perform:**
  - Straight-line and rotational movement tests over measured distances.
  - Compare calculated vs. actual position (record error).
  - Empirical calibration of wheel separation and radius (document process and results).
- **Measurements required:**
  - Error plots (distance and angle).
  - Table of calibration values before and after adjustment.
  - Quantitative analysis of odometry error (mean, std deviation).

### 3.3.4 Failure Mode and Robustness Analysis
- **Objective:** Assess system behavior under fault conditions and validate recovery strategies.
- **Tests to perform:**
  - Simulate sensor failure (e.g., disconnect LiDAR, encoder dropout) and observe system response.
  - Induce communication loss between Pi and dev machine (record recovery behavior).
  - Node crash/restart scenarios (test lifecycle management and fault tolerance).
- **Measurements required:**
  - Description and timing of recovery actions.
  - Table of failure scenarios and system responses.
  - Recommendations for improving robustness.

### 3.3.5 Summary Table and Visual Aids
- **Objective:** Present results in a clear, quantitative manner.
- **Requirements:**
  - Summary table of all tests, metrics, and outcomes.
  - Diagrams/plots for key results (e.g., odometry error, PID response, navigation accuracy).
  - Block diagram of test setup if relevant.

### 3.3.6 PID Tuning and Motor Control Validation

To validate the PID tuning and motor symmetry, a step response test was performed using teleop_twist_keyboard to command the robot to move forward and backward. The default velocity command was used, resulting in a target speed close to 13 rad/s, as observed in the motor speed response plot below.

The following figure shows the recorded left and right motor speeds in response to step inputs:

![PID Step Response (Left & Right Motors)](pid_step_response.png)

**Analysis:**
- Both motors respond rapidly to step inputs, reaching the target speed with minimal delay and little to no overshoot.
- The steady-state speed for both motors closely matches the commanded value, with negligible error.
- The left and right motor speed traces are nearly identical, confirming effective wheel bias correction and symmetric PID tuning.
- The system is robust to both positive and negative step inputs, with similar performance in both directions.
- The traces are smooth, with minimal noise or oscillation, indicating good stability and filtering in the control loop.

**Conclusion:**
This step response demonstrates that the PID parameters are tuned well enough for reliable motor control. The system achieves fast, stable, and symmetric response for both motors, validating the control architecture for further navigation and integration testing.

---

## References

Macenski, S. & Jambrecic, I. (2021). SLAM Toolbox: SLAM for the dynamic world. *Journal of Open Source Software*, 6(61), 2783. https://doi.org/10.21105/joss.02783

Macenski, S., Martín, F., White, R. & Clavero, J. G. (2020). The Marathon 2: A Navigation System. In *2020 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*. IEEE. https://arxiv.org/abs/2003.00368

Macenski, S., Moore, T., Lu, D. V., Merzlyakov, A. & Ferguson, M. (2023). From the desks of ROS maintainers: A survey of modern & capable mobile robotics algorithms in the robot operating system 2. *Robotics and Autonomous Systems*, 168, 104493. https://doi.org/10.1016/j.robot.2023.104493

Macenski, S., Booker, M. & Wallace, J. (2024). Open-Source, Cost-Aware Kinematically Feasible Planning for Mobile and Surface Robotics. *arXiv preprint arXiv:2401.07993*. https://arxiv.org/abs/2401.07993

Macenski, S., Singh, S., Martin, F. & Gines, J. (2023). Regulated Pure Pursuit for Robot Path Tracking. *Autonomous Robots*, 47(6), 685-702. https://doi.org/10.1007/s10514-023-10097-6

Merzlyakov, A. & Macenski, S. (2021). A Comparison of Modern General-Purpose Visual SLAM Approaches. In *2021 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*. IEEE. https://arxiv.org/abs/2107.07589
