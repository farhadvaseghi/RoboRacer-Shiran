# RoboRacer Estimation Module (`roboracer_estimation`)

## Overview & System Design
The `roboracer_estimation` package is a standalone ROS2 estimation module designed to provide high-frequency, low-latency state estimation for the RoboRacer platform. It employs an adaptive Extended Kalman Filter (EKF) capable of running on Jetson Orin Nano hardware, with native support for both real-world deployment and WSL-based hardware-in-the-loop simulation.

The architecture comprises two main nodes working in tandem:
1. **Adaptive Covariance Pre-Processor (`adaptive_covariance_node.py`)**: Subscribes to raw sensor streams and actively manipulates and inflates covariance matrices dynamically based on real-time environmental factors (e.g., wheel slip from angular velocities, motion blur from lateral acceleration).
2. **Core EKF Node (`robot_localization::ekf_node`)**: A standard ROS2 package that subscribes to the pre-processed "adaptive" outputs and continuously fuses them at 50-100Hz to produce a single reliable estimated state (`/odometry/filtered`) and reliable TFs.

## Dependencies
- **Core ROS 2 Libraries**: `rclpy`, `nav_msgs`, `sensor_msgs`, `geometry_msgs`
- **Execution Utilities**: `robot_localization` (C++ executable, managed via `exec_depend` in `package.xml`)
- **Build & Packaging**: `ament_python`, `setuptools`

---

## Running the Stack

Open **4 terminals** and run the following in each:

### Terminal 1 — Simulator
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch roboracer_estimation sim.launch.py
```

### Terminal 2 — Navigation Stack
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch roboracer_estimation navigation.launch.py
```

### Terminal 3 — Pure Pursuit Controller
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch roboracer_control controller.launch.py
```

### Terminal 4 — RViz
```bash
source ~/ros2_ws/install/setup.bash
ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```

Once all three are running, use the **Nav2 Goal** tool in RViz to click a target position on the track. The robot will plan a path and autonomously follow it to the goal.

---

## File Breakdown, Inputs, Outputs & Functionality

### 1. `roboracer_estimation/roboracer_estimation/adaptive_covariance_node.py`
**Functionality:** Pre-processes raw sensor messages and republishes them with adjusted covariance matrices before they reach the EKF. When angular velocity exceeds `omega_threshold`, wheel slip is assumed and the VESC odometry covariance is inflated. When lateral acceleration exceeds `ay_threshold`, visual degradation is assumed and the ZED odometry covariance is inflated. In sim mode the node passes gym odometry through with nominal non-zero covariances (the gym publishes zero covariances, which would cause the EKF to over-trust the input).

**Parameters:**
- `is_sim` (bool): Selects sim mode (single gym odom input) or real mode (ZED + VESC inputs).
- `omega_threshold` (double): Angular velocity above which VESC twist covariance is inflated (rad/s).
- `ay_threshold` (double): Lateral acceleration above which ZED pose and twist covariances are inflated (m/s²).
- `slip_inflation_factor` (double): Multiplier applied to VESC twist covariance diagonal when slip is detected.
- `visual_inflation_factor` (double): Multiplier applied to ZED pose and twist covariance diagonals when lateral acceleration is high.

**Inputs (Subscriptions):**
- *Sim mode:* `/ego_racecar/odom` (`nav_msgs/Odometry`) — gym ground-truth odometry.
- *Real mode:*
  - `/zed/zed_node/imu/data` (`sensor_msgs/Imu`) — ZED 2i IMU at ~400 Hz.
  - `/zed/zed_node/odom` (`nav_msgs/Odometry`) — ZED visual odometry.
  - `/vesc/odom` (`nav_msgs/Odometry`) — VESC wheel odometry.

**Outputs (Publishers):**
- *Sim mode:* `/estimation/sim_odom_adaptive` (`nav_msgs/Odometry`)
- *Real mode:*
  - `/estimation/imu_adaptive` (`sensor_msgs/Imu`) — republished unchanged; ZED already fills covariance fields.
  - `/estimation/zed_odom_adaptive` (`nav_msgs/Odometry`)
  - `/estimation/vesc_odom_adaptive` (`nav_msgs/Odometry`)

### 2. `roboracer_estimation/roboracer_estimation/ekf_validator_node.py`
**Functionality:** Sim-only validation tool (Phase 4). Subscribes to the EKF output and the gym ground truth simultaneously, computing position error (Euclidean), yaw error (wrapped angular difference), and forward velocity error on every matched message pair. Immediately logs `WARN` when any metric exceeds its threshold. Every `report_interval` seconds it prints a rolling mean/max summary and resets accumulators.

**Parameters:**
- `report_interval` (double): Seconds between summary log prints (default 5.0).
- `max_position_error` (double): Position error threshold before `WARN` is emitted (metres).
- `max_yaw_error` (double): Yaw error threshold before `WARN` is emitted (radians).
- `max_velocity_error` (double): Forward velocity error threshold before `WARN` is emitted (m/s).

**Inputs (Subscriptions):**
- `/odometry/filtered` (`nav_msgs/Odometry`) — EKF output.
- `/ego_racecar/odom` (`nav_msgs/Odometry`) — gym ground truth.

**Outputs:**
- Console logs only (`INFO` for periodic reports, `WARN` for threshold breaches). No ROS topic publishers.

### 3. `launch/estimation.launch.py`
**Functionality:** Single entry point for the full estimation stack. An `OpaqueFunction` resolves all argument-dependent logic in Python at launch time (config file path, frequency default) before any node is started.

**Launch Arguments:**
- `is_sim` (default `true`): Passes `is_sim` to `adaptive_covariance_node` and selects `ekf_sim.yaml` or `ekf_real.yaml`.
- `use_sim_time` (default `true`): Passed to both nodes; set `false` on real hardware.
- `ekf_frequency` (default `0.0`): EKF update rate in Hz. `0` = auto-select: 50 Hz when `is_sim:=true`, 100 Hz when `is_sim:=false`.

**Nodes launched:**
- `adaptive_covariance_node` from this package.
- `ekf_filter_node` from `robot_localization`, with `ekf_frequency` and `use_sim_time` layered on top of the YAML config.

### 4. `config/ekf_sim.yaml` & `config/ekf_real.yaml`
**Functionality:** Parameter files for the `robot_localization` EKF node. Each file specifies which sensor topics to fuse, which state dimensions to include, and TF broadcast behaviour.

**Sim mode (`ekf_sim.yaml`):**
- Single odometry input: `/estimation/sim_odom_adaptive`.
- `publish_tf: false` — f1tenth_gym_ros hard-publishes `odom → base_link` itself; a second broadcaster would cause TF conflicts.
- Fuses x, y, yaw, vx, vy, and yaw-rate from the gym odometry.

**Real hardware (`ekf_real.yaml`):**
- Three sensor inputs:
  - `imu0` (`/estimation/imu_adaptive`): fuses yaw, yaw-rate, and forward acceleration. `imu0_remove_gravitational_acceleration: true` subtracts the static gravity component from the accelerometer reading before fusion.
  - `odom0` (`/estimation/zed_odom_adaptive`): fuses full 2D pose (x, y, yaw) and twist.
  - `odom1` (`/estimation/vesc_odom_adaptive`): fuses forward velocity (vx) and yaw-rate only — position is excluded to prevent encoder drift accumulating in the position estimate.
- `publish_tf: true` — the EKF is the sole `odom → base_link` TF broadcaster on real hardware.

### 5. Standard Python Setup (`setup.py`, `setup.cfg`, `package.xml`)
ROS2 ament_python packaging files. `setup.py` registers two console scripts (`adaptive_covariance_node`, `ekf_validator_node`) so both are accessible via `ros2 run roboracer_estimation <node_name>` after `colcon build`. `robot_localization` is listed as `exec_depend` in `package.xml` only — it is a C++ package and must not appear in `setup.py`'s `install_requires`.

### 6. Supporting Design and Technical Documents
- `estimation_design.md`: Design specification — describes the architecture, data flow diagrams, and sim vs real hardware decisions.
- `implementation_plan.md`: Phase-by-phase implementation roadmap with concrete step-by-step tasks.
- `assumptions.md`: Hardware capability assumptions and sensor topic contracts the design depends on.
- `progress.md`: Phase completion records, key decisions made during implementation, and test instructions for each phase.
- `report_phase<N>test.md`: Per-phase test reports confirming each phase's acceptance criteria were met.
