# RoboRacer — Testing Guide

The repo now contains one ROS2 package: `roboracer_estimation`.
The f1tenth gym simulator (solid oval track, two-car mode) lives in `deps/`.

---

## Prerequisites

```bash
source /opt/ros/humble/setup.bash
cd ~/roboracer_ws   # or wherever your colcon workspace root is
```

---

## 1. Build

```bash
colcon build --packages-select roboracer_estimation
source install/setup.bash
```

Verify the package is registered:

```bash
ros2 pkg list | grep roboracer_estimation
# roboracer_estimation
```

Verify all three executables are available:

```bash
ros2 run roboracer_estimation adaptive_covariance_node --help
ros2 run roboracer_estimation ekf_validator_node --help
ros2 run roboracer_estimation moving_obstacle_controller --help
```

---

## 2. Run the Full Sim Stack (single command)

This starts the gym bridge (solid oval track, two cars), the moving-obstacle
controller for the opponent car, and the EKF estimation stack together:

```bash
ros2 launch roboracer_estimation sim.launch.py
```

Optional: override the EKF update rate:

```bash
ros2 launch roboracer_estimation sim.launch.py ekf_frequency:=30.0
```

**What to expect:**
- RViz opens showing the solid oval track
- Ego car spawns at one side of the track, opponent car on the other side
- The opponent car immediately starts driving laps autonomously
- The EKF begins publishing on `/odometry/filtered` after ~1 second

---

## 3. Verify Topics

In a second terminal (after sourcing the workspace):

```bash
# EKF output — the main product of this stack
ros2 topic hz /odometry/filtered           # expect ~50 Hz

# Gym odometry feeding the EKF (via adaptive covariance node)
ros2 topic hz /ego_racecar/odom            # expect ~50 Hz
ros2 topic hz /estimation/sim_odom_adaptive  # expect ~50 Hz

# Opponent car is driving
ros2 topic hz /opp_racecar/odom            # expect ~50 Hz
ros2 topic hz /opp_drive                   # expect ~20 Hz (controller output)

# Inspect EKF output values
ros2 topic echo /odometry/filtered --once
# Check:
#   pose.pose.position.x/y      — non-zero after the car has moved
#   pose.covariance[0]          — small positive number (not 0, not NaN)
#   twist.twist.linear.x        — positive when ego car is moving forward
```

---

## 4. Drive the Ego Car

With the sim running, open a third terminal and use keyboard teleop:

```bash
# Install if needed: ros2 run teleop_twist_keyboard teleop_twist_keyboard
# Or drive via topic directly:
ros2 topic pub /ego_racecar/drive ackermann_msgs/msg/AckermannDriveStamped \
  "{drive: {speed: 1.0, steering_angle: 0.0}}" --once
```

---

## 5. EKF Accuracy Validation

With the sim running, start the validator in a separate terminal:

```bash
ros2 run roboracer_estimation ekf_validator_node
```

Drive the ego car (or let it sit while the opponent laps). Every 5 seconds it prints:

```
--- EKF validation report (N samples) ---
  position : mean=X.XXXX m   max=X.XXXX m   limit=0.200 m
  yaw      : mean=X.XX°      max=X.XX°       limit=5.73°
  velocity : mean=X.XXXX m/s max=X.XXXX m/s  limit=0.300 m/s
```

**Pass criteria:** all mean values below their limits, no `WARN` lines during normal operation.

---

## 6. TF Sanity Check

With the sim + estimation running:

```bash
# Confirm gym is the sole odom→base_link broadcaster (publish_tf: false in sim mode)
ros2 run tf2_tools view_frames
# Open frames.pdf — must show exactly one broadcaster for odom→base_link

# Confirm transforms are valid (no extrapolation warnings)
ros2 run tf2_ros tf2_echo odom base_link
```

---

## 7. Reverting to Before This Claude Code Session

Two commits were made in this session. The commit before the session started is `be7b9c4`.

### Step 1 — back up files that will be deleted by the hard reset

`git reset --hard` removes files that were added to git tracking in our commits.
Back them up first:

```bash
cp -r roboracer_estimation /tmp/roboracer_estimation_backup
```

### Step 2 — reset to before the session

```bash
git reset --hard be7b9c4
```

### Step 3 — restore estimation as untracked (its state before the session)

```bash
cp -r /tmp/roboracer_estimation_backup ./roboracer_estimation
```

After this, the repo root is the `roboracer_perception` package (flat layout),
`roboracer_estimation/` is an untracked nested folder, and no files from this
session remain.

### Verify

```bash
git log --oneline -3
# be7b9c4 Add cylinder wall scenarios and combined obstacle support  ← HEAD
# c177547 Adjust moving obstacle start positions

ls
# CMakeLists.txt  package.xml  setup.py  ...  roboracer_estimation/
```
