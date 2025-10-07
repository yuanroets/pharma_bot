# Nav2 and SLAM Parameter Tuning Summary

## Overview
This document provides a comprehensive summary of all parameter tuning performed for the pharma_bot navigation system, specifically focusing on Nav2 navigation stack optimization and SLAM localization improvements. The tuning was performed to achieve safer, more controlled robot operation with better dynamic obstacle handling and improved path following.

## Robot Specifications
- **Physical Dimensions**: 300mm longest dimension
- **Navigation Radius**: 210mm (0.21m) - optimized for precise navigation
- **Operating Environment**: Indoor pharmacy/healthcare facility
- **Safety Requirements**: Conservative operation with obstacle over-correction preference

## Nav2 Parameter Tuning (`nav2_params.yaml`)

### 1. Velocity and Motion Control Parameters

#### Primary Velocity Limits
- **`max_vel_x`**: `0.25` → `0.18` → `0.12` m/s
  - **Reasoning**: Progressive speed reduction for enhanced safety and control. Final speed of 0.12 m/s provides walking-pace operation suitable for healthcare environments.

- **`max_vel_theta`**: `1.0` → `0.6` → `0.4` rad/s
  - **Reasoning**: Reduced rotational speed for more stable turning and better path following precision.

- **`max_speed_xy`**: `0.25` → `0.18` → `0.12` m/s
  - **Reasoning**: Matches linear velocity limits for consistent behavior.

#### Acceleration and Deceleration
- **`acc_lim_x`**: `1.0` → `0.6` → `0.4` m/s²
  - **Reasoning**: Gentler acceleration reduces wheel slip and provides smoother motion for delicate cargo transport.

- **`decel_lim_x`**: `-1.0` → `-1.2` → `-1.5` m/s²
  - **Reasoning**: Faster deceleration for enhanced safety when obstacles are detected.

- **`acc_lim_theta`**: `1.5` → `1.0` → `0.8` rad/s²
  - **Reasoning**: Reduced rotational acceleration for more stable turning behavior.

- **`decel_lim_theta`**: `-1.5` → `-1.8` rad/s²
  - **Reasoning**: Faster rotational stopping for better obstacle response.

### 2. DWB Controller Planning Parameters

#### Trajectory Sampling
- **`vx_samples`**: `25` → `30`
  - **Reasoning**: More velocity sampling options for better trajectory selection in complex environments.

- **`vtheta_samples`**: `25` → `30`
  - **Reasoning**: More angular velocity options for smoother turning trajectories.

- **`sim_time`**: `2.0` → `2.5` seconds
  - **Reasoning**: Look further ahead for obstacles, providing more reaction time for safety.

#### Trajectory Resolution
- **`linear_granularity`**: `0.03` → `0.02` m
  - **Reasoning**: Higher resolution trajectory evaluation for more precise path following.

- **`angular_granularity`**: `0.02` → `0.015` rad
  - **Reasoning**: Higher angular resolution for smoother rotational movements.

### 3. Path Following and Goal Behavior

#### Critic Weights (Path Following Priority)
- **`PathAlign.scale`**: `20.0` → `28.0` → `32.0`
  - **Reasoning**: Increased weight forces robot to stay closer to planned path, reducing wandering behavior.

- **`PathDist.scale`**: `24.0` → `32.0`
  - **Reasoning**: Stronger preference for staying near the global path.

- **`GoalAlign.scale`**: `20.0` → `24.0`
  - **Reasoning**: Better goal approach behavior while maintaining path following priority.

- **`GoalDist.scale`**: `20.0` → `24.0`
  - **Reasoning**: Balanced goal seeking with path adherence.

- **`RotateToGoal.scale`**: `32.0`
  - **Reasoning**: Strong final rotation to goal orientation for precise positioning.

#### Goal and Movement Tolerances
- **`xy_goal_tolerance`**: `0.25` → `0.35` m
  - **Reasoning**: Slightly relaxed tolerance to account for noisy odometry while maintaining precision.

- **`yaw_goal_tolerance`**: `0.25` → `0.35` rad
  - **Reasoning**: More forgiving angular tolerance for easier goal achievement.

- **`trans_stopped_velocity`**: `0.1` → `0.08` m/s
  - **Reasoning**: More sensitive stopped detection for better goal achievement.

### 4. Progress and Movement Monitoring

#### Progress Checker
- **`required_movement_radius`**: `0.5` → `0.3` m
  - **Reasoning**: More sensitive progress detection to identify stuck situations earlier.

- **`movement_time_allowance`**: `10.0` → `8.0` seconds
  - **Reasoning**: Less patience for stuck situations, triggering recovery behaviors sooner.

### 5. Costmap Configuration

#### Local Costmap Updates
- **`update_frequency`**: `10.0` → `12.0` Hz
  - **Reasoning**: Faster obstacle detection for better dynamic obstacle response.

- **`publish_frequency`**: `5.0` → `6.0` Hz
  - **Reasoning**: More frequent publishing for reactive behavior in dynamic environments.

#### Global Costmap Updates
- **`update_frequency`**: `2.0` → `3.0` Hz
  - **Reasoning**: Faster global map updates for better long-term planning.

- **`publish_frequency`**: `2.0` → `2.5` Hz
  - **Reasoning**: Improved global costmap availability for planning.

### 6. Robot Safety Parameters

#### Robot Footprint
- **`robot_radius`**: `0.22` → `0.21` m
  - **Reasoning**: Optimized for 300mm robot (210mm radius = 420mm diameter) providing accurate collision detection.

#### Inflation Layer
- **`inflation_radius`**: `0.9` → `1.0` m
  - **Reasoning**: Larger safety cushion around obstacles for conservative navigation.

- **`cost_scaling_factor`**: `5.0` → `4.5`
  - **Reasoning**: Slightly reduced scaling to balance safety with path following capability.

### 7. Sensor Configuration

#### LiDAR Parameters
- **`raytrace_max_range`**: `4.0` → `4.5` m
  - **Reasoning**: Longer raytracing for better obstacle clearing and map maintenance.

- **`obstacle_max_range`**: `3.5` → `4.0` m
  - **Reasoning**: Detect obstacles at greater distances for improved reaction time.

## SLAM/AMCL Parameter Tuning

### Localization Robustness
- **`alpha1`**: `0.2` → `0.5`
- **`alpha2`**: `0.2` → `0.5`
- **`alpha3`**: `0.2` → `0.5`
- **`alpha4`**: `0.2` → `0.5`
  - **Reasoning**: Increased odometry noise parameters to rely more heavily on LiDAR for localization in environments with potential wheel slip.

### Particle Filter Enhancement
- **`min_particles`**: `500` → `1000`
  - **Reasoning**: More particles for more robust localization, especially during recovery situations.

- **`max_beams`**: `60` → `90`
  - **Reasoning**: Use more laser scan points for better scan matching accuracy.

### Laser Model Tuning
- **`z_hit`**: `0.5` → `0.8`
  - **Reasoning**: Higher trust in correct laser readings for better localization accuracy.

- **`z_rand`**: `0.5` → `0.2`
  - **Reasoning**: Lower trust in random readings to reduce noise impact.

## Key Design Decisions

### 1. Speed vs Safety Trade-off
**Decision**: Prioritize safety over speed with 0.12 m/s maximum velocity
**Rationale**: Healthcare environment requires conservative operation to protect patients and equipment

### 2. Path Following vs Obstacle Avoidance
**Decision**: Strong path following with large safety margins
**Rationale**: Prefer slight over-correction for obstacles while maintaining planned routes

### 3. Localization Method
**Decision**: Maintain SLAM localization instead of switching to EKF
**Rationale**: Current SLAM setup works reliably; avoid introducing complexity

### 4. Update Frequencies
**Decision**: Higher costmap update frequencies (12Hz local, 3Hz global)
**Rationale**: Dynamic healthcare environment requires rapid obstacle detection

## Runtime Configuration Notes

**Important**: Most Nav2 parameters require a navigation stack restart to take effect. Parameters are not dynamically reconfigurable during operation.

**Recommended Workflow**:
1. Make batch parameter changes rather than incremental adjustments
2. Restart navigation stack after parameter modifications
3. Test in controlled environment before deployment
4. Monitor system performance and adjust if needed

## Performance Impact Summary

- **Safety**: Significantly improved with larger inflation radius and conservative speeds
- **Path Following**: Enhanced with stronger PathAlign weights and reduced wandering
- **Obstacle Response**: Improved with faster costmap updates and longer planning horizon
- **Goal Achievement**: Balanced precision with achievability through relaxed tolerances
- **System Stability**: More robust with gentler accelerations and better progress monitoring

## Future Tuning Considerations

1. **Environment-Specific Tuning**: Consider different parameter sets for different facility areas
2. **Load-Dependent Parameters**: Adjust speeds and accelerations based on cargo weight
3. **Time-of-Day Variations**: Different safety margins for busy vs quiet periods
4. **Seasonal Adjustments**: Parameter updates based on facility usage patterns

---
*Document Generated*: October 2025  
*Robot Configuration*: Pharma Bot Navigation Stack  
*Tuning Session*: Conservative Safety-First Optimization
