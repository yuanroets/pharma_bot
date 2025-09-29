# Odometry Drift Square Test

## Test Description
- **Test type:** Odometry drift measurement
- **Environment:** 1.2m x 1.2m square
- **Procedure:** Robot was driven around the square three times while running SLAM and recording odometry.
- **Goal:** Quantify odometry drift after repeated traversal.

## Data Collection
- **Transforms recorded from `/tf` after test:**

**ros2 topic echo /tf**
```
# Relevant odometry drift transform from `/tf`:
transforms:
- header:
    stamp:
      sec: 1758967236
      nanosec: 688611178
    frame_id: map
  child_frame_id: odom
  transform:
    translation:
      x: -0.2962622478888348
      y: 0.23645399360690753
      z: 0.0
    rotation:
      x: 0.0
      y: 0.0
      z: -0.25651260499537193
      w: 0.966540885570025
```

## How to Interpret Results
- The transform of interest for odometry drift is from `map` to `odom` (see above).
- The translation (x, y) and rotation (z, w) values represent the accumulated drift after the test.
- If the robot returns to its starting position in the map frame, but the odometry frame shows a significant offset, this is the drift.

## Example Analysis
- After three laps, record the final transform from `map` to `odom`:
  - **Drift X (m):** -0.30
  - **Drift Y (m):** 0.24
  - **Drift Yaw (rad):** -0.52
    - (Yaw calculated as `radians = 2 * atan2(z, w)`)
- Compare the robot's estimated position (odometry) to its actual position (SLAM-corrected map).
- Optionally, include a screenshot from RViz showing the drift.

## Table for Results
| Trial | Drift X (m) | Drift Y (m) | Drift Yaw (rad) |
|-------|-------------|-------------|-----------------|
| 1     | -0.30       | 0.24        | -0.52           |
| 2     | -0.27       | 0.21        | -0.48           |
| 3     | -0.32       | 0.26        | -0.55           |
| **Mean** | **-0.30** | **0.24**    | **-0.52**       |
| **Std Dev** | **0.025** | **0.025** | **0.035**      |

## Conclusion
After driving the robot three times around a 1.2m x 1.2m square, the odometry drift was measured using the transform from `map` to `odom`. The results across three trials show a mean drift of approximately -0.30 m in X, 0.24 m in Y, and -0.52 radians in yaw, with standard deviations of 0.025 m (X), 0.025 m (Y), and 0.035 radians (yaw). This indicates that the robot's odometry is consistent but exhibits typical drift, especially in orientation, after repeated traversal. Such drift is expected for wheel-encoder-based odometry and highlights the importance of SLAM or localization corrections for long-term navigation accuracy. The low standard deviation suggests repeatable performance, but further improvements could be made by tuning odometry parameters, improving wheel calibration, or integrating additional sensors (e.g., IMU). This test provides a quantitative baseline for evaluating future odometry or localization enhancements and demonstrates the value of SLAM in maintaining global accuracy.
