# Estimation Module Assumptions

Based on the hardware stack, the following assumptions are made for the state estimation architecture:

## Hardware Capabilities & Data Sources

1. **Nvidia Jetson Orin Nano Super Developer Kit**
   - Provides sufficient compute for high-frequency EKF at ~100Hz.
   - Runs the ZED ROS2 wrapper (`zed-ros2-wrapper`) and VESC driver alongside the estimation stack.

2. **ZED 2i Stereo Camera**
   - Built-in IMU published automatically by `zed-ros2-wrapper` at ~400Hz on `/zed/zed_node/imu/data`.
   - Visual odometry (pose + twist) published on `/zed/zed_node/odom` by the same wrapper.
   - No custom IMU driver required; the wrapper handles it.

3. **Hokuyo UST-10LX LiDAR**
   - Provides 2D scans at 40Hz on `/scan`.
   - Used for **two independent purposes simultaneously**:
     - Cone detection (existing `lidar_processor.py` node).
     - Scan-matching localization via a separate SLAM node (e.g., Cartographer or Hector SLAM) to produce map-frame pose estimates.
   - These two uses share the same `/scan` topic; they do not interfere. The SLAM node configuration (map tuning, loop closure) is treated as a **separate deliverable** outside this estimation module.

4. **Trampa VESC 6 MKVI**
   - The `vesc_driver` ROS2 package publishes wheel odometry on `/vesc/odom` natively.
   - Provides high-quality linear velocity in `x` and angular velocity from encoder counts.
   - The existing codebase already sends drive commands to the VESC; receiving odometry simply requires the driver to be running (standard deployment step).

## Estimation Requirements

- **State Vector:** $[x, y, \theta, v_x, v_y, \omega]$ — 2D pose and 2D twist.
- **Target Frequency:** 50–100Hz to keep MPC/pure-pursuit controllers stable at race speeds.
- **Adaptive Covariance:** Covariances are inflated dynamically based on vehicle state (e.g., wheel-slip at high angular velocity, visual blur at speed).

## Simulation Constraints

- The f1tenth_gym_ros simulator does **not** publish an IMU topic. Sim mode therefore runs the EKF on odometry alone (`/ego_racecar/odom`), which is sufficient for testing given the gym's ground-truth quality. A synthetic IMU node can be added later if IMU-path testing is needed.
- The VESC and ZED SDK are not present in simulation; their topics are absent. All sim inputs route from the gym's single odom topic.
