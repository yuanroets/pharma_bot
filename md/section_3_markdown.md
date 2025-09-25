# 3 Technical Integration and Implementation

# 3 Technical Integration and Implementation
Implementation  
The complexity of this project did not lie only in the hardware and software 
development, but in creating a way of allowing so many complex systems to 
interface correctly and efficiently. Modern autonomous robotics systems require 
careful consideration of hardware -software integration, power management, 
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
## 3.2 Software Design  and Implementation  
The software architecture for the autonomous hospital assistant robot represents a 
complex integration of distributed computing, real -time control systems, and 
advanced robotics algorithms. Built upon the Robot Operating System 2 (ROS2) 
framework, the system demonstrates sophisticated systems engineering principles 
through its modular design, fault -tolerant operation, and seamless integration 
between high -level navigation algorith ms and low -level hardware control.  
The software stack addresses several fundamental challenges in autonomous 
robotics: real -time sensor processing, simultaneous localisation and mapping 
(SLAM), path planning, distributed computation across multiple processing units, 
and robust hardware abst raction. The architecture employs a distributed approach, 
with computationally intensive tasks executed on a development machine whilst 
time-critical hardware interfacing operates on the embedded Raspberry Pi platform.  
This design philosophy reflects modern industry practices in autonomous systems, 
where computational loads are balanced across different  hardware platforms to 
optimise performance, power consumption, and system reliability. The modular 
architecture ensures maintainability, upgradability , and adherence to established 
robotics software engineering principles.




24 3.2.1  System Architecture Overview  
#### 3.2.1.1 Simulation Environment and Development Workflow  
The project employed a Gazebo simulation environment for initial algorithm 
development, parameter tuning, and risk -free testing of navigation behaviours.  
Gazebo Integration and URDF Modelling:   
Gazebo provides a physics -based simulation environment that accurately models 
robot dynamics, sensor behaviour, and environmental interactions. The robot model 
was defined using URDF (Unified Robot Description Format), an XML -based 
specification that descr ibes the robot's physical properties, joint relationships, and 
sensor placements.  
The URDF model includes detailed specifications for:  
• Link geometries and inertial properties (mass, centre of mass, moments of 
inertia)  
• Joint definitions and kinematic constraints  
• Sensor mounting positions and orientations  
• Material properties and collision meshes  
• Visual representations for simulation rendering  
Simulation Benefits:  The simulation environment enabled rapid iteration of control 
algorithms without hardware wear, testing of edge cases and failure scenarios 
safely, validation of navigation algorithms before hardware deployment, and 
parameter optimisation in controlled co nditions.  
Transition to Hardware Implementation:  With simulation validation complete, the 
focus shifted to implementing a robust software architecture for the physical robot. 
The following sections detail the actual robot software implementation, which 
forms the core contribution of this project.  
#### 3.2.1.2 Distributed Computing Architecture  
The software system employs a distributed architecture that partitions 
computational responsibilities between two primary computing platforms  (section 
### 3.1.3 ): 
Raspberry Pi 4:  
• Real-time motor control and encoder processing  
• LiDAR sensor driver and data acquisition  
• Hardware abstraction and low -level device management




25 • Serial communication bridge with Arduino microcontroller  
• Basic teleoperation interface  
Development Machine (Off -board Processing):  
• SLAM processing and map generation  
• Advanced localisation algorithms (AMCL)  
• Global and local path planning (Nav2 stack)  
• Visualisation and debugging tools (RViz2)  
• High -level navigation coordination  
#### 3.2.1.3 Node Architecture and Lifecycle Management  
The software employs ROS2's node -based architecture, with clear separation of 
concerns across functional modules. Critical nodes utilise ROS2's lifecycle 
management system, enabling managed state transitions and robust fault recovery 
capabilities.  
Insert image here of rqt graph  
Core Lifecycle Nodes:  
• SLAM Toolbox nodes (mapping and localisation modes)  
• Navigation stack components (planners, controllers, costmaps)  
• LiDAR driver node  
• Motor control interface  
Stateless Nodes:  
• Odometry calculation and publishing  
• Teleoperation bridges  
• Transform publishers  
• Visualisation interfaces  
Lifecycle nodes provide managed state transitions through four primary states: 
unconfigured, inactive, active, and finalized. This enables dynamic reconfiguration, 
graceful failure handling, and systematic startup/shutdown procedures without 
requiring complete system restarts.  
The lifecycle management approach represents advanced systems engineering 
practice, acknowledging that complex robotic systems require sophisticated state 
management to handle the dynamic nature of real -world operation.  An example of 
this would be, to temporarily disable the LiDAR by changing its state to inactive 
for debugging reasons.




26 3.2.1.4  Communication Patterns and Data Flow  
The system employs multiple ROS2 communication patterns to handle different 
types of data exchange:  
Topics  for streaming sensor data (LiDAR scans, odometry, transforms) Actions  for 
goal-oriented tasks (navigation commands, mapping operations) Services  for 
request -response operations (parameter updates, system queries) Parameters  for 
configuration management and dynamic reconfiguration  
Data flow follows a hierarchical pattern from hardware abstraction through sensor 
processing to high -level navigation algorithms. The transform tree (TF2) system 
maintains spatial relationships between coordinate frames, ensuring consistent 
spatial reasoni ng across all system components.  
### 3.2.2 Hardware Abstraction and Embedded Control  
#### 3.2.2.1 Arduino -ROS2 Serial Bridge  
The interface between ROS2 and the embedded Arduino microcontroller represents 
a critical component requiring robust communication protocols and real -time 
performance. The system implements a custom serial protocol that bridges high -
level ROS2 velocity commands to low -level motor control.  
The Arduino firmware implements a command -based protocol supporting:  
• Motor velocity commands (rad/s and PWM modes)  
• Encoder value queries and streaming  
• PID controller parameter updates  
• System diagnostics and status reporting  
 
1. // Command parsing structure  
2. typedef struct { 
3.   char command; 
4.   float arg1, arg2, arg3; 
5.   bool valid; 
6. } CommandFrame ; 
7.   
The protocol employs single -character commands for efficiency, with numerical 
arguments transmitted as ASCII strings. This approach balances simplicity with 
extensibility, enabling future hardware additions without protocol redesign.  
Safety Mechanisms:  
• Automatic motor timeout (500ms) prevents runaway conditions  
• Command validation prevents invalid motor states




27 • Serial buffer management avoids data overflow  
• Graceful degradation during communication failures  
#### 3.2.2.2 Motor Control and PID Implementation  
The embedded control system implements PID (Proportional -Integral -Derivative) 
velocity control for each motor, converting high -level velocity commands to PWM 
signals. The PID implementation follows established control theory best practices:  
 1. void doPID(SetPointInfo  * p) { 
 2.   long Perror; 
 3.   long output; 
 4.   int input; 
 5.   
 6.   input = p->Encoder - p->PrevEnc; 
 7.   Perror = p->TargetTicksPerFrame  - input; 
 8.   
 9.   output = (Kp * Perror - Kd * (input - p->PrevInput ) + p->ITerm) / Ko; 
10.   p->PrevEnc = p->Encoder; 
11.   
12.   output += p->output; 
13.   if (output >= MAX_PWM) 
14.     output = MAX_PWM; 
15.   else if (output <= -MAX_PWM) 
16.     output = -MAX_PWM; 
17.   else 
18.     p ->ITerm += Ki * Perror; 
19.   
20.   p->output = output; 
21.   p->PrevInput  = input; 
22. } 
Key Engineering Features:  
• Derivative -on-measurement approach prevents derivative kick  
• Integral windup prevention through output saturation limits  
• Calibrated motor scaling factors compensate for mechanical variations  
• 50 Hz control loop frequency ensures responsive performance  
The PID parameters were empirically tuned through systematic testing, with Kp = 
20, Ki = 0, Kd = 12, and Ko = 50. The derivative term proved most effective for 
damping oscillations, whilst the integral term was disabled to prevent windup in the 
presence of  mechanical backlash.  
#### 3.2.2.3 Encoder Integration and Odometry  
Encoder feedback utilises interrupt -driven processing to ensure accurate velocity 
measurement and position tracking. The quadrature encoder interface employs 
hardware interrupts on Arduino pins 2 and 3  for motor 1 and pins 4 and 5 for motor 
2, with a lookup table approach minimising computational overhead:  
 1. ISR (PCINT2_vect ) { 
 2.   static uint8_t enc_last_left = 0; 
 3.   static uint8_t enc_last_right = 0;




28  4.    
 5.   enc_last_left <<= 2; 
 6.   enc_last_left |= (PIND & (3 << 4)) >> 4; 
 7.   left_enc_pos += ENC_STATES [(enc_last_left & 0x0f)]; 
 8.    
 9.   enc_last_right <<= 2; 
10.   enc_last_right |= (PIND & (3 << 2)) >> 2; 
11.   right_enc_pos += ENC_STATES [(enc_last_right & 0x0f)]; 
12. } 
The encoder resolution was determined to be  1859 counts per wheel revolution. 
This differs from the theoretical value of 1980 counts (11 pulses per motor 
revolution × 45:1 gearbox × 4 for quadrature decoding), with the discrepancy 
attributed to mechanical tolerances and gearbox variations. The test ing 
methodology and validation of this empirically -derived value is detailed in the 
subsequent testing section  (refer to section ). This resolution yields a distance 
resolution of 0.087 mm per count, providing ade quate precision for odometry 
calculations essential for navigation algorithms.  
### 3.2.3 Sensor Integration and Data Processing  
#### 3.2.3.1 LiDAR Integration and Driver Architecture  
The LDLiDAR LD 06 integration demonstrates sophisticated sensor driver 
architecture utilising ROS2's lifecycle node capabilities. The driver implements a 
composable node design, enabling containerised deployment and efficient resource 
utilisation.  
The sensor provides 360° ranging data with 2 cm accuracy over a 12 -meter range, 
scanning at 10 Hz with 1° angular resolution. Communication occurs via UART at 
230,400 baud, with the driver publishing sensor data on the /ldlidar_node/ scan  
topic using sensor_msgs/LaserScan  messages.  
Configuration Management:  
 1. comm: 
 2.   serial_port : '/dev/ttyAMA0'  
 3.   baudrate : 230400 
 4. lidar : 
 5.   model : 'LDLiDAR_LD19'  
 6.   frame_id : 'ldlidar_link'  
 7.   bins : 455 
 8.   range_min : 0.03 
 9.   range_max : 15.0 
The lifecycle node design enables dynamic reconfiguration and fault recovery. If 
the sensor encounters communication errors, the lifecycle manager can restart the 
node without affecting other system components.




29 3.2.3.2  Coordinate Frame Management  
Accurate spatial representation requires careful management of coordinate frames 
using ROS2's TF2 (Transform) system. The robot maintains a hierarchical frame 
tree: 
Insert view frames image here  
map 
└── odom (published by odometry or SLAM)  
    └── base_link (published by robot_state_publisher)  
        ├── base_footprint  
        ├── wheel_left_link  
        ├── wheel_right_link  
        └── ldlidar_link  
Frame Relationships:  
• map: Global reference frame for navigation and localisation  
• odom : Odometry frame, subject to drift but locally accurate  
• base_link : Robot body frame, origin for sensors and actuators  
• ldlidar_link : LiDAR sensor frame, positioned 180mm above base  
The transform tree enables sensor fusion by providing spatial relationships between 
all coordinate frames. Static transforms define fixed relationships (sensor mounting 
positions), whilst dynamic transforms track the robot's motion through space.  
#### 3.2.3.3 Odometry Calculation and Publishing  
The odometry system converts encoder readings to positi on estimates using 
differential drive kinematics. The simple_odometry  node subscribes to encoder 
values and applies kinematic equations:  
 1. # Convert encoder counts to wheel distances  
 2. left_distance = (left_diff / self.encoder_cpr ) * 2.0 * math.pi * 
self.wheel_radius  
 3. right_distance = (right_diff / self.encoder_cpr ) * 2.0 * math.pi * 
self.wheel_radius  
 4.   
 5. # Calculate robot motion  
 6. distance = (left_distance + right_distance ) / 2.0 
 7. delta_theta = (right_distance - left_distance ) / self.wheel_separation  
 8.   
 9. # Update pose  
10. self.x += distance * math.cos(self.theta + delta_theta /2.0) 
11. self.y += distance * math.sin(self.theta + delta_theta /2.0) 
12. self.theta += delta_theta  
Calibration Parameters:  
• Encoder CPR: 1859 counts per wheel revolution (empirically determined)  
• Wheel radius: 0.02569 m ( measured to get a baseline and then empirically 
calibrated)




30 • Wheel separation: 0.173 m ( measured to get a baseline and then empirically 
calibrated ) 
• Motor scaling factors: Applied to compensate for velocity differences between 
left and right motors  due to mechanical interference. ( I don’t like mechanical 
interference use some thing better ) 
The odometry node publishes both the /odom  topic and the odom → base_link  
transform, ensuring compatibility with SLAM and navigation algorithms. 
Velocities are calculated through numerical differentiation and included in the 
odometry message.  
### 3.2.4 Simultaneous Localisation and Mapping (SLAM)  
#### 3.2.4.1 SLAM Toolbox Implementation  
The SLAM system utilises the slam_toolbox package, which implements graph -
based SLAM algorithms optimised for real -time operation. The system operates in 
two primary modes:  
Mapping Mode ( online_async ): 
• Real-time map construction from LiDAR and odometry data  
• Loop closure detection and correction  
• Occupancy grid map generation  
• Pose graph optimisation  
Localisation Mode ( localization ): 
• Pose estimation within a known map  
• Particle filter -based localisation  
• Continuous pose correction and refinement  
The SLAM configuration was tuned for hospital environments, balancing map 
accuracy with computational performance:  
 1. # Key SLAM parameters  
 2. minimum_travel_distance : 0.5 
 3. minimum_travel_heading : 0.5 
 4. scan_buffer_size : 10 
 5. scan_buffer_maximum_scan_distance : 10.0 
 6. link_match_minimum_response_fine : 0.1 
 7. link_scan_maximum_distance : 1.5 
 8. loop_search_maximum_distance : 3.0 
 9. do_loop_closing : true 
10. loop_match_minimum_chain_size : 10 
11. loop_match_maximum_variance_coarse : 3.0 
12. loop_match_minimum_response_coarse : 0.35 
13. loop_match_minimum_response_fine : 0.45




31 These parameters were empirically determined through extensive testing in 
simulated hospital environments. The loop closure parameters are particularly 
critical, as incorrect loop closures can introduce significant map distortions.  
#### 3.2.4.2 Occupancy Grid Mapping  
SLAM output consists of occupancy grid maps, where each cell contains a 
probability of occupancy. The mapping algorithm updates cell probabilities using 
the log -odds representation:  
 𝑙(𝑐|𝑧₁:𝑡) = 𝑙(𝑐|𝑧₁:𝑡₋₁) + 𝑙𝑜𝑔(𝑃(𝑐|𝑧𝑡)) − 𝑙𝑜𝑔(𝑃(𝑐))  
```latex
(16)
``` 
Where  𝑙 represents the log -odds value,  𝑐 is the cell, and 𝑧₁:𝑡 are sensor 
measurements. This approach efficiently handles sensor uncertainty whilst 
providing maps suitable for path planning algorithms.  
Map resolution was set to 0.05 metres per pixel, providing sufficient detail for 
navigation whilst maintaining manageable computational loads. The mapping range 
was limited to 10 metres to focus on relevant hospital corridor distances.  
#### 3.2.4.3 Localisation within Known Maps  
For operation within existing maps, the system employs Adaptive Monte Carlo 
Localisation (AMCL), which maintains a particle filter representation of pos ition 
belief. AMCL parameters were tuned to emphasise LiDAR accuracy over 
odometry:  
 1. # AMCL configuration  
 2. min_particles : 500 
 3. max_particles : 2000 
 4. kld_err : 0.05 
 5. kld_z : 0.99 
 6. update_min_d : 0.2 
 7. update_min_a : 0.5 
 8. laser_max_beams : 60 
 9. laser_z_hit : 0.5 
10. laser_z_short : 0.05 
11. laser_z_max : 0.05 
12. laser_z_rand : 0.5 
13. laser_sigma_hit : 0.2 
14. odom_alpha_1 : 0.2 
15. odom_alpha_2 : 0.2 
16. odom_alpha_3 : 0.8 
17. odom_alpha_4 : 0.2 
The high particle count (500 -2000) ensures robust localisation in corridor 
environments where geometric features may be sparse. The laser model parameters 
emphasise accurate range measurements whilst the odometry model parameters 
reflect the uncertainty in troduced by odometry .




32 3.2.5  Navigation and Path Planning  
#### 3.2.5.1 Navigation Stack Architecture  
The navigation system employs the Nav2 stack, which implements a behaviour 
tree-based architecture for complex decision -making. The modular design separates 
global planning, local control, and recovery behaviours:  
Core Components:  
• Planner Server : Global path planning using A* algorithm  
• Controller Server : Local trajectory tracking with DWA (Dynamic Window 
Approach)  
• Recoveries Server : Failure recovery behaviours (spin, backup, wait)  
• Behaviour Tree Navigator : High -level decision coordination  
• Costmap Layers : Multi -layered obstacle representation  
The behaviour tree approach enables sophisticated navigation logic, with the ability 
to handle complex scenarios such as temporary obstacles, navigation failures, and 
dynamic replanning.  
#### 3.2.5.2 Global Path Planning  
Global planning employs the A* algorithm operating on the occupancy grid map. 
The planner finds optimal paths from the robot's current position to goal locations, 
accounting for static obstacles represented in the map.  
Insert image of A* arrows  
A* Implementation Details:  
• Heuristic function: Euclidean distance to goal  
• Cost function: Combined distance and obstacle penalties  
• Path smoothing: Post -processing for realistic trajectories  
• Dynamic replanning: Triggered by significant map changes  
The global planner operates at 1 Hz, providing efficient path updates without 
excessive computational overhead. Path tolerance parameters were relaxed to 
accommodate odometry uncertainty:  
1. # Global planner parameters  
2. tolerance : 0.5 
3. use_astar : true 
4. allow_unknown : true 
5. use_final_approach_orientation : false




33 3.2.5.3  Local Path Planning and Obstacle Avoidance  
Local planning employs the DWA algorithm, which evaluates candidate trajectories 
within the robot's kinematic constraints. The algorithm optimises a multi -objective 
function considering path progress, obstacle clearance, and velocity constraints.   
Insert image of the arrows with DWA  
DWA Trajectory Evaluation:  
1. score = path_distance_bias × path_distance +  
2.         goal_distance_bias × goal_distance +  
3.         occdist_scale × obstacle_distance  
Parameters were tuned for hospital environments, emphasising safety over speed:  
 1. # DWA parameters  
 2. max_vel_x : 0.26 
 3. min_vel_x : 0.0 
 4. max_vel_y : 0.0 
 5. max_vel_theta : 1.0 
 6. min_vel_theta : -1.0 
 7. acc_lim_x : 2.5 
 8. acc_lim_theta : 3.2 
 9. sim_time : 1.7 
10. discretize_by_time : false 
11. vx_samples : 6 
12. vtheta_samples : 20 
13. path_distance_bias : 64.0 
14. goal_distance_bias : 24.0 
15. occdist_scale : 0.56 
16. forward_point_distance : 0.325 
17. stop_time_buffer : 0.2 
18. scaling_speed : 0.25 
19. max_scaling_factor : 0.2 
The trajectory evaluation occurs at 20 Hz, ensuring responsive obstacle avoidance 
whilst maintaining computational efficiency.  
#### 3.2.5.4 Costmap Configuration  
The navigation system employs layered costmaps to represent different obstacle 
types and navigation constraints:  
Static Layer : Permanent obstacles from the occupancy grid map .  
Obstacle Layer : Dynamic obstacles detected by LiDAR .  
Inflation Layer : Safety margins around obstacles . 
Each layer assigns numerical costs to grid cells, with the final costmap representing 
the combination of all layers. Path planning algorithms use these costs to generate 
safe, efficient trajectories.  
 1. # Costmap configuration  
 2. global_frame : map 
 3. robot_base_frame : base_link  
 4. update_frequency : 5.0




34  5. publish_frequency : 2.0 
 6. resolution : 0.05 
 7. robot_radius : 0.20 
 8. inflation_radius : 0.55 
 9. cost_scaling_factor : 5.0 
The inflation radius (0.55m) provides adequate clearance for the robot's dimensions 
whilst enabling navigation through standard doorways (typically 0.8 -1.0m wide).  
3.2.6  System Integration and Launch Management  
3.2.6.1  Launch File Architecture  
The system employs multiple launch files to handle different operational scenarios:  
Development Testing ( dev_test_launch.py ): 
• Robot state publisher and URDF loading  
• SLAM Toolbox (mapping mode)  
• RViz2 visualisation  
• Static transform publishers  
Localisation Testing ( dev_test_launch_localization.py ): 
• SLAM Toolbox (localisation mode with saved map)  
• Map server for map loading  
• Visualisation tools  
Full Navigation ( dev_test_amcl_launch.py ): 
• AMCL localisation  
• Nav2 navigation stack  
• Complete autonomous navigation capability  
Hardware Deployment ( pi_test_launch.py ): 
• Motor driver and serial communication  
• LiDAR driver  
• Hardware -specific interfaces  
This modular launch structure enables systematic testing and deployment, with 
clear separation between simulation, development, and hardware operation modes.  
3.2.6.2  Node Lifecycle Orchestration  
Launch files employ delayed actions to ensure proper startup sequencing, 
particularly critical for lifecycle nodes that require managed initialisation:




35  1. # Example lifecycle node startup  
 2. slam_node = LifecycleNode ( 
 3.     package='slam_toolbox' , 
 4.     executable ='async_slam_toolbox_node' , 
 5.     name ='slam_toolbox' , 
 6.     parameters =[slam_params_file ], 
 7.     namespace ='', 
 8.     output ='screen'  
 9. ) 
10.   
11. lifecycle_manager = Node( 
12.     package='nav2_lifecycle_manager' , 
13.     executable ='lifecycle_manager' , 
14.     name ='lifecycle_manager_slam' , 
15.     output ='screen' , 
16.     parameters =[{'autostart' : True}, 
17.                 {'node_names' : ['slam_toolbox' ]}] 
18. ) 
The lifecycle manager coordinates node state transitions, ensuring all dependencies 
are satisfied before activating navigation capabilities.  
3.2.6.3  Parameter Management and Configuration  
Configuration parameters are managed through YAML files, enabling systematic 
tuning and documentation of system behaviour:  
Motor Control Parameters ( motor_driver_params.yaml ): 
 1. motor_driver : 
 2.   ros__parameters : 
 3.     encoder_cpr : 1859 
 4.     wheel_separation : 0.173 
 5.     wheel_radius : 0.02569 
 6.     max_linear_speed : 1.0 
 7.     max_angular_speed : 2.0 
 8.     motor_1_scaler : 1.0 
 9.     motor_2_scaler : 0.95 
SLAM Parameters ( mapper_params_online_async.yaml ): Replace this with the 
actual parametrs that we frequently changed but with the comments to show what 
we need to change for localisation and mapping and so on  
1. slam_toolbox : 
2.   ros__parameters : 
3.     solver_plugin : solver_plugins ::CeresSolver  
4.     ceres_linear_solver : SPARSE_NORMAL_CHOLESKY  
5.     ceres_preconditioner : SCHUR_JACOBI  
6.     ceres_trust_strategy : LEVENBERG_MARQUARDT  
This parameterisation approach supports reproducible testing and enables rapid 
adaptation to different operational environments.




36 3.2.7  Time Synchronisation and Distributed Operation  
3.2.7.1  Network Time Synchronisation  
Distributed operation across multiple computing platforms requires precise time 
synchronisation to ensure consistent sensor fusion and mapping. The system 
employs Chrony (Network Time Protocol implementation) to maintain sub -
millisecond clock synchronisati on between the Raspberry Pi and development 
machine.  
Chrony Configuration:  
1. # On development machine (server)  
2. allow 192.168.0.0/24 
3. local stratum 10 
4.   
5. # On Raspberry Pi (client)  
6. server 192.168.0.100 iburst 
7. maxsources 1 
Time synchronisation is critical for SLAM algorithms, which rely on accurately 
timestamped sensor data for proper operation. Clock drift can result in rejected 
sensor measurements and mapping failures.  
3.2.7.2  Quality of Service and Network Reliability  
ROS2's DDS middleware provides configurable Quality of Service (QoS) policies 
to handle network reliability and latency requirements:  
Sensor Data : Best Effort delivery for high -frequency streams Navigation  
Commands : Reliable delivery for critical control messages  
Map Data : Reliable delivery with large message queues Transform  Data : Latest 
message semantics for real -time updates  
The QoS configuration balances network efficiency with data reliability, ensuring 
critical navigation data is transmitted reliably whilst allowing non -critical sensor 
streams to operate with best -effort delivery.  
3.2.8  Debugging and System Monitoring  
3.2.8.1  Visualisation and Development Tools  
The system incorporates comprehensive debugging and monitoring capabilities 
through RViz2 integration. Custom RViz configurations provide real -time 
visualisation of:  
• Robot model and coordinate frames  
• LiDAR scan data and point clouds  
• Occupancy grid maps and costmaps




37 • Planned and executed trajectories  
• Particle filter pose estimates  
• Transform tree relationships  
RViz Configuration Management:  Multiple RViz configurations support different 
operational phases:  
• pharma_bot.rviz : General development and testing  
• slam.rviz : SLAM -specific visualisation  
• view_bot.rviz : Robot state and sensor monitoring  
3.2.8.2  System Diagnostics and Logging  
ROS2's logging system provides hierarchical message filtering and distributed log 
collection. Log levels are configured per node, enabling detailed debugging without 
overwhelming system output:  
1. # Example diagnostic logging  
2. self.get_logger ().info('Odometry updated: x={:.3f}, y={:.3f}, 
theta={:.3f}' .format( 
3.     self.x, self.y, self.theta)) 
4. self.get_logger ().warn('Encoder timeout detected, stopping motors' ) 
5. self.get_logger ().error('Serial communication failure: {}' .format(str(e))) 
The logging system supports real -time monitoring during operation and post -
analysis of system behaviour during development and testing phases.  
3.2.8.3  Topic and Service Introspection  
ROS2 provides extensive introspection capabilities for debugging distributed 
systems:  
 1. # Monitor topic data rates and message content  
 2. ros2 topic echo /scan 
 3. ros2 topic hz /odom 
 4.   
 5. # Inspect node connectivity and interfaces   
 6. ros2 node info /slam_toolbox  
 7. ros2 node list  
 8.   
 9. # Monitor transform tree relationships  
10. ros2 run tf2_tools view_frames  
11. ros2 run tf2_ros tf2_echo map base_link  
These tools proved essential during development for identifying communication 
failures, parameter misconfigurations, and timing issues in the distributed system.  
Give so random examples here of what we determined using the introspection and 
how we fixed the problem, for example slam was subscribing to the /scan topic 
instead of the ldlidar_node/scan topic, this showed us that we needed to add a relay 
node.




38 3.2.9  Testing and Validation Methodology  
3.2.9.1  Unit Testing Approach  
Individual system components were validated through systematic unit testing:  
Odometry Validation : Physical measurement of robot motion compared with 
calculated odometry over known distances and rotations. Testing revealed 
systematic errors requiring empirical calibration of wheel parameters.  
Motor Control Testing : PID response characterisation through step input testing 
and frequency response analysis. Parameters were tuned to achieve critically 
damped response whilst avoiding steady -state error.  
SLAM Validation : Map accuracy assessment through comparison with ground 
truth measurements in controlled environments. Loop closure detection was 
validated through repeated traversal of known paths.  
3.2.9.2  Integration Testing  
System integration testing focused on inter -node communication, distributed 
operation, and failure recovery:  
Network Communication : Validation of message passing between distributed 
nodes under various network conditions, including simulated packet loss and 
latency.  
Lifecycle Management : Testing of node startup/shutdown sequences, failure 
recovery, and dynamic reconfiguration capabilities.  
Transform Consistency : Validation of coordinate frame relationships and 
transform tree integrity during dynamic operation.  
3.2.9.3  Performance Characterisation  
System performance was characterised across multiple metrics:  
Computational Load : CPU utilisation monitoring during SLAM, navigation, and 
sensor processing operations.  
Memory Usage : RAM consumption analysis for map storage, particle filters, and 
sensor data buffering.  
Real -time Performance : Timing analysis of control loops, sensor processing, and 
navigation updates.  
Network Bandwidth : Data transmission rates and latency measurements for 
distributed operation.
