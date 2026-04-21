# How to Run the RoboRacer Simulation

Step-by-step guide for every team member to get the simulation running from scratch.

---

## Prerequisites

- Ubuntu 22.04
- ROS 2 Humble installed — [installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- Python 3.10+
- Git

Install required ROS/Python packages if not already present:

```bash
sudo apt install python3-colcon-common-extensions python3-rosdep \
    ros-humble-nav2-map-server ros-humble-nav2-lifecycle-manager \
    ros-humble-rviz2 ros-humble-robot-state-publisher \
    ros-humble-xacro ros-humble-rqt-image-view
pip install transforms3d numpy opencv-python-headless scikit-learn scipy cv-bridge
```

---

## First-Time Setup

Run this once after cloning. It handles everything: cloning dependencies, patching the simulator, and building.

```bash
# 1. Create the workspace
mkdir -p ~/roboracer_ws/src

# 2. Clone the team repo into src/
git clone -b perception https://github.com/farhadvaseghi/RoboRacer-Shiran.git \
    ~/roboracer_ws/src/roboracer_perception

# 3. Run the setup script
cd ~/roboracer_ws/src/roboracer_perception
bash setup_workspace.sh
```

The setup script:
- Clones `f1tenth_gym_ros` and `f1tenth_gym` automatically
- Applies the required patches (portable map path, collision handling)
- Installs `f110_gym` via pip
- Builds the full workspace

When it finishes you will see:
```
=== Setup complete ===
```

---

## Running the Simulation

Open **4 terminals**. In every terminal, source the workspace first:

```bash
source ~/roboracer_ws/install/setup.bash
```

---

### Terminal 1 — Simulator + RViz

```bash
cd ~/roboracer_ws && source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

This starts the physics simulator and opens RViz automatically.
The car spawns at position (0, 0) on the oval track.

---

### Terminal 2 — Keyboard Teleop

```bash
cd ~/roboracer_ws && source install/setup.bash
ros2 run roboracer_perception teleop_key
```

**Controls:**

| Key | Action |
|-----|--------|
| `W` | Forward (3.0 m/s) |
| `S` | Reverse (1.5 m/s) — also resets steering to straight |
| `A` | Steer left |
| `D` | Steer right |
| `Space` | Full stop |
| `Q` | Quit |

**Hold keys** — the terminal sends a key-repeat signal while held. Steering automatically returns to centre ~0.5 s after you release the key.

---

### Terminal 3 — Fake ZED Camera

```bash
cd ~/roboracer_ws && source install/setup.bash
ros2 run roboracer_perception sim_camera
```

Publishes synthetic cone images on the same topics as the real ZED 2i:
- `/zed/zed_node/rgb/image_rect_color`
- `/zed/zed_node/depth/depth_registered`
- `/zed/zed_node/rgb/camera_info`

---

### Terminal 4 — View Camera Feed (optional)

```bash
cd ~/roboracer_ws && source install/setup.bash
ros2 run rqt_image_view rqt_image_view /zed/zed_node/rgb/image_rect_color
```

Opens a window showing what the simulated camera sees.

---

## Topics Published by Perception

These are the outputs your module subscribes to:

| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | `sensor_msgs/LaserScan` | LiDAR scan (270°, 1081 rays, 40 Hz) |
| `/ego_racecar/odom` | `nav_msgs/Odometry` | Car pose and velocity |
| `/perception/detected_cones` | `visualization_msgs/MarkerArray` | Detected cones (x, y, color, confidence, radius) in `base_link` frame |

Cone colors: `1` = blue (left boundary), `2` = yellow (right boundary), `3` = orange (start gate).

---

## Coordinate Frame

- Origin (`base_link`) = **rear axle**, ground level
- x = forward, y = left, z = up (ROS REP-103)
- LiDAR is at x=+0.270 m, z=+0.110 m relative to `base_link`

---

## Rebuilding After Code Changes

Python scripts are **not** symlinked by colcon. After editing any `.py` file you must rebuild and re-source in every open terminal:

```bash
cd ~/roboracer_ws
colcon build --symlink-install --packages-ignore f110_gym
source install/setup.bash
```

> Run `source install/setup.bash` in **each open terminal** after rebuilding — once is not enough.

---

## Troubleshooting

**Simulator fails to load the map**
Run `setup_workspace.sh` again — the patches may not have been applied correctly the first time.

**`ModuleNotFoundError: f110_gym`**
```bash
pip install -e ~/roboracer_ws/src/f1tenth_gym/
```

**Teleop has no effect / car does not move**
Make sure Terminal 1 (simulator) is running first before starting teleop. Check that Terminal 2 shows `speed=` output when you press keys.

**RViz view is tilted**
Click **Reset** in the bottom-left of RViz to restore the default top-down view.

**Changes to `.py` files have no effect**
Rebuild and re-source (see above).
