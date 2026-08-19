# Estimation Module — Implementation Progress

## Phase 1: Package Scaffolding & EKF Configuration — DONE

### What was created

```
roboracer_estimation/
├── package.xml                          # ROS2 package manifest (ament_python)
├── setup.py                             # Python package build config
├── setup.cfg                            # ament_python install path config
├── resource/
│   └── roboracer_estimation             # ament index marker (empty file)
├── roboracer_estimation/
│   └── __init__.py                      # Python package root
└── config/
    ├── ekf_sim.yaml                     # EKF config for WSL/gym simulation
    └── ekf_real.yaml                    # EKF config for Jetson real hardware
```

### Key decisions

**`package.xml` build type: `ament_python`**
The package contains only Python nodes and YAML/launch files.
`robot_localization` is listed as `<exec_depend>` only — it is a C++ ROS2
package and must not appear in `setup.py`'s `install_requires`.

**`ekf_sim.yaml`**
- Frequency: 50 Hz (WSL resource-friendly).
- Single odometry input: `/estimation/sim_odom_adaptive`.
- `publish_tf: false` — f1tenth_gym_ros hard-publishes `odom → base_link`
  itself; a second broadcaster would cause TF conflicts that break
  `cone_tracker.py` and other nodes consuming that transform.

**`ekf_real.yaml`**
- Frequency: 100 Hz.
- Three fused inputs:
  - `imu0` — ZED 2i IMU via `/estimation/imu_adaptive` (yaw + yaw-rate + ax).
  - `odom0` — ZED visual odometry via `/estimation/zed_odom_adaptive`
    (full 2D pose + twist).
  - `odom1` — VESC wheel odometry via `/estimation/vesc_odom_adaptive`
    (vx + yaw-rate only; position not fused from VESC to avoid encoder drift
    accumulating in the position estimate).
- `publish_tf: true` — EKF is the sole `odom → base_link` TF source on
  real hardware.

---

## How to test Phase 1

### 1 — Verify the package is discovered by ROS2

```bash
# From your ROS2 workspace root (where roboracer_estimation/ lives as a package)
colcon build --packages-select roboracer_estimation
source install/setup.bash
ros2 pkg list | grep roboracer_estimation   # must appear
```

### 2 — Verify config files are installed

```bash
ros2 pkg prefix roboracer_estimation
# then check:
ls $(ros2 pkg prefix roboracer_estimation)/share/roboracer_estimation/config/
# expected: ekf_sim.yaml  ekf_real.yaml
```

### 3 — Validate YAML syntax

```bash
python3 -c "import yaml; yaml.safe_load(open('config/ekf_sim.yaml'))"
python3 -c "import yaml; yaml.safe_load(open('config/ekf_real.yaml'))"
# both should return with no errors
```

### 4 — Smoke-test robot_localization reads the sim config

Requires `robot_localization` installed (`sudo apt install ros-<distro>-robot-localization`).

```bash
# In WSL with ROS2 sourced:
ros2 run robot_localization ekf_node --ros-args \
  --params-file install/roboracer_estimation/share/roboracer_estimation/config/ekf_sim.yaml
# Expected: node starts, prints "Preparing to set 1 odometry inputs",
#           warns about no data (normal — no publishers yet).
# Not expected: YAML parse errors or missing parameter crashes.
```

### 5 — Full integration (after Phase 2 & 3 are done)

```bash
# Inside WSL with the sim running:
bash run_sim.sh &
ros2 launch roboracer_estimation estimation.launch.py is_sim:=true
ros2 topic echo /odometry/filtered   # must publish at ~50 Hz
ros2 topic hz /odometry/filtered     # confirm rate
```

---

---

## Phase 2: Adaptive Covariance Pre-Processor Node — DONE

### What was created

```
roboracer_estimation/
└── roboracer_estimation/
    └── adaptive_covariance_node.py     # Phase 2 deliverable
```

### What the node does

`adaptive_covariance_node.py` is a single ROS2 node that handles **both sim and
real modes** via the `is_sim` parameter (default `true`).

**Sim mode** (`is_sim:=true`)

| Subscription | Publisher |
|---|---|
| `/ego_racecar/odom` (`nav_msgs/Odometry`) | `/estimation/sim_odom_adaptive` |

The gym ground-truth odom is passed through with **nominal covariances** written
in (gym messages often have zero covariances which would confuse the EKF).
The adaptive code-path is still exercised, ensuring end-to-end integration
coverage.

**Real mode** (`is_sim:=false`)

| Subscription | Publisher | Adaptive rule |
|---|---|---|
| `/zed/zed_node/imu/data` (`sensor_msgs/Imu`) | `/estimation/imu_adaptive` | Republish as-is (ZED populates covariances) |
| `/zed/zed_node/odom` (`nav_msgs/Odometry`) | `/estimation/zed_odom_adaptive` | Inflate pose + twist covariance ×`visual_inflation_factor` when `\|ay\| > ay_threshold` |
| `/vesc/odom` (`nav_msgs/Odometry`) | `/estimation/vesc_odom_adaptive` | Inflate twist covariance ×`slip_inflation_factor` when `\|ω\| > omega_threshold` |

The node caches the latest IMU angular velocity (`ω`) and lateral acceleration
(`ay`) so odom callbacks can query them without extra bookkeeping.

### Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `is_sim` | `true` | Switch between sim and real wiring |
| `omega_threshold` | `1.5` rad/s | Above this, VESC odom covariance is inflated |
| `ay_threshold` | `3.0` m/s² | Above this, ZED visual odom covariance is inflated |
| `slip_inflation_factor` | `10.0` | Multiplier applied to VESC twist covariance diagonal |
| `visual_inflation_factor` | `5.0` | Multiplier applied to ZED pose + twist covariance diagonal |

### Key decisions

**Nominal covariance injection in sim mode**
The gym publishes zero covariances in its odometry messages. `robot_localization`
treats a zero covariance as "this sensor is infinitely trusted", which causes the
EKF to over-trust the input and makes the filter numerically unstable if the
message is even slightly inconsistent with the filter state. Injecting nominal
non-zero covariances fixes this.

**Inflation guard (`_LARGE_COV_THRESHOLD = 1e5`)**
Masked dimensions (those set to 1e6 to indicate "don't fuse this") are skipped
during inflation so they remain effectively infinite and are not accidentally
reduced by repeated multiplication.

**IMU pass-through in real mode**
The ZED SDK already computes and fills IMU covariance fields. Re-estimating them
would be redundant and potentially less accurate. The IMU subscriber updates the
cached `_omega` and `_ay` values used by the odom callbacks, then publishes the
original message unchanged.

---

## How to test Phase 2

### 1 — Static unit test: nominal covariance is applied in sim mode

```bash
# In WSL with ROS2 sourced, open two terminals.

# Terminal 1 — run the node in sim mode
ros2 run roboracer_estimation adaptive_covariance_node \
  --ros-args -p is_sim:=true

# Terminal 2 — publish a zero-covariance gym odom and inspect the output
ros2 topic pub /ego_racecar/odom nav_msgs/msg/Odometry \
  '{header: {frame_id: "odom"}, child_frame_id: "base_link"}' &
ros2 topic echo /estimation/sim_odom_adaptive --once
# Expected: pose.covariance[0] ≈ 0.05, twist.covariance[0] ≈ 0.01
#           (not 0.0 as published by the gym stub)
```

### 2 — Static unit test: VESC covariance inflated above omega threshold

```bash
# Terminal 1 — run in real mode
ros2 run roboracer_estimation adaptive_covariance_node \
  --ros-args -p is_sim:=false -p omega_threshold:=1.0

# Terminal 2 — publish a fake IMU with high angular velocity (ω = 2.0 rad/s)
ros2 topic pub /zed/zed_node/imu/data sensor_msgs/msg/Imu \
  '{angular_velocity: {z: 2.0}}' &

# Terminal 2 — publish a fake VESC odom with small nominal covariances
ros2 topic pub /vesc/odom nav_msgs/msg/Odometry \
  '{twist: {covariance: [0.01,0,0,0,0,0, 0,1e6,0,0,0,0, 0,0,1e6,0,0,0, 0,0,0,1e6,0,0, 0,0,0,0,1e6,0, 0,0,0,0,0,0.01]}}' &

ros2 topic echo /estimation/vesc_odom_adaptive --once
# Expected: twist.covariance[0] ≈ 0.10  (0.01 × 10.0 slip_inflation_factor)
#           twist.covariance[35] ≈ 0.10
```

### 3 — Static unit test: ZED covariance inflated above ay threshold

```bash
# Similar setup — publish IMU with high lateral acceleration (ay = 5.0 m/s²)
ros2 topic pub /zed/zed_node/imu/data sensor_msgs/msg/Imu \
  '{linear_acceleration: {y: 5.0}}' &

ros2 topic pub /zed/zed_node/odom nav_msgs/msg/Odometry \
  '{pose: {covariance: [0.05,0,0,0,0,0, 0,0.05,0,0,0,0, 0,0,1e6,0,0,0, 0,0,0,1e6,0,0, 0,0,0,0,1e6,0, 0,0,0,0,0,0.05]}}' &

ros2 topic echo /estimation/zed_odom_adaptive --once
# Expected: pose.covariance[0] ≈ 0.25  (0.05 × 5.0 visual_inflation_factor)
#           pose.covariance[35] ≈ 0.25
```

### 4 — Integration smoke test with sim (after Phase 3 launch file is done)

```bash
bash run_sim.sh &
ros2 launch roboracer_estimation estimation.launch.py is_sim:=true
ros2 topic hz /estimation/sim_odom_adaptive   # must match gym odom rate (~50 Hz)
ros2 topic echo /estimation/sim_odom_adaptive --once
# Confirm covariance fields are non-zero
```

### 5 — Confirm node shuts down cleanly on Ctrl-C

```bash
ros2 run roboracer_estimation adaptive_covariance_node \
  --ros-args -p is_sim:=true
# Press Ctrl-C
# Expected: clean shutdown, no stack trace
```

---

---

## Phase 3: Launch Infrastructure — DONE

### What was created

```
roboracer_estimation/
└── launch/
    └── estimation.launch.py     # Phase 3 deliverable
```

### What the launch file does

`estimation.launch.py` is the single entry point for the entire estimation
stack. It spins up both nodes:

1. `adaptive_covariance_node` (Python, this package)
2. `ekf_node` from `robot_localization` (C++)

An `OpaqueFunction` executes at launch time to resolve all argument-dependent
logic in plain Python before any node is started.

### Launch arguments

| Argument | Default | Meaning |
|---|---|---|
| `is_sim` | `true` | Selects sim vs real wiring and config file |
| `use_sim_time` | `true` | Passed to both nodes; set `false` on real hardware |
| `ekf_frequency` | `0.0` | EKF Hz; `0` = auto-select (50 sim / 100 real) |

### Config file selection

| `is_sim` | EKF config used |
|---|---|
| `true` | `config/ekf_sim.yaml` (50 Hz, `publish_tf: false`) |
| `false` | `config/ekf_real.yaml` (100 Hz, `publish_tf: true`) |

The `ekf_frequency` and `use_sim_time` parameters are **layered on top** of the
YAML via a second `parameters` dict so they can be overridden at launch time
without duplicating all YAML keys in the launch file.

### Key decisions

**No topic remapping at the launch level**
Topic wiring is handled entirely inside the two nodes:
- `adaptive_covariance_node` reads `is_sim` and selects its subscriptions/publishers internally.
- `ekf_node` reads topic names from the YAML config files (`odom0:`, `imu0:`, etc.).

This keeps the launch file minimal and avoids duplicating topic names in three places.

**`ekf_frequency: 0.0` as auto-sentinel**
`DeclareLaunchArgument` cannot express a conditional default, so `0.0` is used
as the sentinel meaning "pick the right default for this mode." The
`OpaqueFunction` resolves it to 50 Hz (sim) or 100 Hz (real) before the node
is launched.

---

## How to test Phase 3

### 1 — Verify the launch file is installed

```bash
colcon build --packages-select roboracer_estimation
source install/setup.bash
ros2 launch roboracer_estimation estimation.launch.py --show-args
# Expected output lists: is_sim, use_sim_time, ekf_frequency
```

### 2 — Dry-run: inspect which nodes would be launched

```bash
# This starts the nodes but you can Ctrl-C immediately after verifying the output.
ros2 launch roboracer_estimation estimation.launch.py is_sim:=true
# Expected log lines (before any sensor data):
#   [adaptive_covariance_node]: AdaptiveCovarianceNode started in SIM mode
#   [ekf_filter_node]: Preparing to set 1 odometry inputs
```

### 3 — Full sim integration (requires the gym running)

```bash
bash run_sim.sh &
sleep 5   # wait for gym to start publishing

ros2 launch roboracer_estimation estimation.launch.py is_sim:=true

# In a third terminal:
ros2 topic hz /estimation/sim_odom_adaptive  # should match gym odom rate
ros2 topic hz /odometry/filtered             # should be ~50 Hz
ros2 topic echo /odometry/filtered --once    # confirm pose fields are non-zero
```

### 4 — Confirm ekf_frequency override works

```bash
ros2 launch roboracer_estimation estimation.launch.py ekf_frequency:=30.0
# In another terminal:
ros2 topic hz /odometry/filtered   # should be ~30 Hz, not 50 Hz
```

### 5 — Confirm auto-frequency selection

```bash
# Sim mode (auto → 50 Hz)
ros2 launch roboracer_estimation estimation.launch.py is_sim:=true ekf_frequency:=0.0
ros2 topic hz /odometry/filtered   # expect ~50 Hz

# Real mode dry-run (auto → 100 Hz) — nodes will warn about missing sensors, which is expected
ros2 launch roboracer_estimation estimation.launch.py is_sim:=false use_sim_time:=false ekf_frequency:=0.0
# [ekf_filter_node] frequency parameter should be 100.0 in the node output
```

---

---

## Phase 4: Integration and WSL Testing — DONE

### What was created

```
roboracer_estimation/
├── roboracer_estimation/
│   └── ekf_validator_node.py    # Phase 4 deliverable — sim validation node
└── setup.py                     # updated: ekf_validator_node entry point added
```

### Note on package structure

The nested `roboracer_estimation/roboracer_estimation/` layout is **correct and
intentional**. ROS2 ament_python packages always have this shape: the outer
directory is the ROS2 package (owns `package.xml`); the inner directory is the
Python module (owns `__init__.py` and all source files). `setup.py` uses
`find_packages()` which discovers the inner module correctly. The entry point
`roboracer_estimation.adaptive_covariance_node:main` resolves as expected after
`colcon build`.

### `ekf_validator_node.py`

Subscribes to both `/odometry/filtered` (EKF output) and `/ego_racecar/odom`
(gym ground truth). On every matched pair of messages it computes:

| Metric | Formula |
|---|---|
| Position error | Euclidean distance between EKF and GT (x, y) |
| Yaw error | Signed angular difference, wrapped to (−π, π] |
| Velocity error | `|filtered_vx − gt_vx|` |

Immediately emits `WARN` log lines if any metric exceeds its threshold.
Every `report_interval` seconds (default 5 s) prints a rolling report with
mean and max for each metric, then resets accumulators.

#### Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `report_interval` | `5.0` s | How often to print the summary |
| `max_position_error` | `0.20` m | Threshold before WARN |
| `max_yaw_error` | `0.10` rad | Threshold before WARN |
| `max_velocity_error` | `0.30` m/s | Threshold before WARN |

---

## How to test Phase 4

### 1 — Rebuild and verify both nodes are registered

```bash
colcon build --packages-select roboracer_estimation
source install/setup.bash
ros2 pkg executables roboracer_estimation
# Expected:
#   roboracer_estimation adaptive_covariance_node
#   roboracer_estimation ekf_validator_node
```

### 2 — Full sim integration: all three nodes running together

```bash
# Terminal 1 — start the sim
bash run_sim.sh

# Terminal 2 — start estimation stack
ros2 launch roboracer_estimation estimation.launch.py is_sim:=true

# Terminal 3 — start the validator
ros2 run roboracer_estimation ekf_validator_node

# Expected after ~5 s:
# --- EKF validation report (N samples) ---
#   position : mean=X.XXXX m   max=X.XXXX m   limit=0.200 m
#   yaw      : mean=X.XX°      max=X.XX°       limit=5.73°
#   velocity : mean=X.XXXX m/s max=X.XXXX m/s  limit=0.300 m/s
# No WARN lines should appear during normal straight or low-speed driving.
```

### 3 — Verify /odometry/filtered is publishing at the correct rate

```bash
ros2 topic hz /odometry/filtered
# Expected: ~50 Hz (sim default)

ros2 topic hz /estimation/sim_odom_adaptive
# Expected: matches gym odom rate (~50 Hz)
```

### 4 — TF conflict check (publish_tf: false in sim)

```bash
ros2 run tf2_tools view_frames
# Open frames.pdf — must show odom → base_link published by the gym,
# NOT by ekf_filter_node.  There must be exactly one broadcaster for
# this edge; a duplicate would show as a TF warning in the logs.

# Also confirm cone_tracker (if running) still receives valid transforms:
ros2 run tf2_ros tf2_echo odom base_link
# Must print valid transforms without "extrapolation into the future" errors.
```

### 5 — Confirm /odometry/filtered contains sane values

```bash
ros2 topic echo /odometry/filtered --once
# Check:
#   pose.pose.position.x/y  — non-zero after the car has moved
#   pose.covariance[0]      — small positive number (not 0, not NaN)
#   twist.twist.linear.x    — positive when car is moving forward
#   twist.twist.angular.z   — non-zero when turning
```

### 6 — Real hardware pre-deployment checklist

Before switching to `is_sim:=false` on the Jetson:

- [ ] `zed-ros2-wrapper` is running:
  ```bash
  ros2 topic hz /zed/zed_node/imu/data    # expect ~400 Hz
  ros2 topic hz /zed/zed_node/odom        # expect ~15–30 Hz
  ```
- [ ] `vesc_driver` is running:
  ```bash
  ros2 topic hz /vesc/odom                # expect ~50–100 Hz
  ```
- [ ] Launch estimation in real mode:
  ```bash
  ros2 launch roboracer_estimation estimation.launch.py \
      is_sim:=false use_sim_time:=false
  ros2 topic hz /odometry/filtered        # expect ~100 Hz
  ```
- [ ] Confirm `publish_tf: true` is active — `odom → base_link` must come
  from `ekf_filter_node`, not from any other source:
  ```bash
  ros2 run tf2_ros tf2_echo odom base_link
  ```
- [ ] SLAM node (Cartographer / Hector SLAM) is publishing its pose on a
  `map → odom` TF if map-frame localization is required. This is a
  **separate deliverable** — `lidar_processor.py` and the SLAM node both
  subscribe to `/scan` independently and do not interfere with each other
  or with the estimation stack.

---

## Implementation complete

All four phases are done. The full estimation stack is:

```
ros2 launch roboracer_estimation estimation.launch.py   # sim (default)
ros2 launch roboracer_estimation estimation.launch.py is_sim:=false use_sim_time:=false  # real
```

Output: `/odometry/filtered` (`nav_msgs/Odometry`) at 50 Hz (sim) or 100 Hz (real).
