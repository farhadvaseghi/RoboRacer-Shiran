# RoboRacer Perception — HOWTO

Everything you need to clone, build, and run the perception stack on your
machine, plus the interface contract for downstream sub-modules (estimation,
planning, control).

## What this package does

One scenario, end-to-end:

- **Track**: solid-wall oval, 60 m straights, 2.85 m lane width, drawn as
  light/dark pixels in `maps/solid_oval_track.pgm`.
- **Sensors**: HOKUYO UST-10LX 2D LIDAR + ZED 2i stereo camera. In simulation,
  the LIDAR comes from `f1tenth_gym_ros` and the camera is synthesised by
  `sim_camera_walls` (renders the wall geometry + opponent into a fake ZED
  feed at ~5–6 Hz on WSL2; real ZED on Jetson is unrelated).
- **LIDAR processing**: DBSCAN clustering + PCA in `wall_opponent_detector`.
  Classifies each cluster into WALL or OPPONENT and publishes one
  `DetectionArray` with full shape (x, y, length, width, yaw, color,
  confidence, radius).
- **Camera processing**: HSV color blob detection in `camera_processor`.
  Detects yellow outer walls, blue inner walls, and the red opponent body.
- **Opponent**: a second simulator agent driven by `moving_obstacle_controller`
  around the same oval at ~0.9 m/s, with a smooth lateral bypass when static
  obstacles are present.

There are no cones, no other tracks, no other scenarios. The package was
trimmed to this one configuration in May 2026.

## Prerequisites

- Ubuntu 22.04 (native or under WSL2 + WSLg)
- ROS 2 Humble Hawksbill, sourced as usual:
  `source /opt/ros/humble/setup.bash`
- Python 3.10
- `colcon`, `git`, `pip`
- For WSL2 users: the WSLg display server is on by default — no extra X
  install required.

## First-time setup

### 1. Clone the workspace

```bash
mkdir -p ~/roboracer_ws/src
cd ~/roboracer_ws/src
git clone -b perception https://github.com/farhadvaseghi/RoboRacer-Shiran.git roboracer_perception
```

### 2. Pull the simulator and apply patches

The package vendors `f1tenth_gym_ros` and `f1tenth_gym` under `deps/` and
keeps the patches it needs under `patches/`. Run the setup script once:

```bash
cd ~/roboracer_ws/src/roboracer_perception
bash setup_workspace.sh
```

This will:
- clone `f1tenth_gym_ros` and `f1tenth_gym` into `deps/` if missing
- copy our patched files from `patches/` over the vendored copies
- `pip install -e` the `f110_gym` python package
- run `colcon build --symlink-install`

### 3. Critical Python environment fix — numpy 1.x

ROS 2 Humble ships `cv_bridge` built against numpy 1.x. If your system has
numpy 2.x installed (common on fresh WSL2), `cv_bridge` will crash at import
with `AttributeError: _ARRAY_API not found`, which kills `sim_camera_walls`,
`camera_processor`, and anything else that uses cv_bridge.

```bash
pip install --user 'numpy<2'
```

You can keep `numpy<2` (1.26.x recommended) for the lifetime of this project.
Verify with:

```bash
python3 -c "import numpy, cv2; from cv_bridge import CvBridge; print('OK', numpy.__version__)"
```

### 4. Source and run

```bash
cd ~/roboracer_ws
source install/setup.bash
```

## Running the simulation

You need two terminals.

**Terminal 1 — simulator + perception + rviz:**

```bash
cd ~/roboracer_ws && source install/setup.bash
ros2 launch roboracer_perception solid_wall_perception.launch.py
```

**Terminal 2 — keyboard teleop:**

```bash
cd ~/roboracer_ws && source install/setup.bash
ros2 run roboracer_perception teleop_key
```

Controls in Terminal 2:

| Key       | Action |
|-----------|--------|
| `W` / `↑` | accelerate forward (4 m/s) |
| `S` / `↓` | brake / reverse (3 m/s)    |
| `A` / `←` | steer left                 |
| `D` / `→` | steer right                |
| `Space`   | full stop                  |
| `Q`       | quit                       |

Teleop uses Xlib's global keymap polling, so you don't strictly need to focus
the T2 terminal, but if your keys aren't being read make sure `DISPLAY` is
set in T2.

## What you should see in RViz

- Light-grey background
- White drivable area inside black non-drivable area (the .pgm map)
- Grey cylinder walls along both boundaries
- Blue ego car in the centre (chase camera follows it)
- Blue opponent car nearby
- Red line segments along detected walls (from `/perception/walls`)
- Green box on the detected opponent (from `/perception/opponent`)
- Red dots where the laser hits walls
- ZED Camera panel showing yellow outer tubes, blue inner tubes, and a red
  box on the opponent when it's in view

## Interface contract — what teammates subscribe to

All perception output is in the `base_link` frame (rear axle, ground level,
x forward, y left, z up — ROS REP-103/105).

| Topic                            | Type                                 | Rate    | Contents |
|----------------------------------|--------------------------------------|---------|----------|
| `/perception/detections`         | `roboracer_perception/DetectionArray`| ~100 Hz | LIDAR PCA detections — every detected wall and the opponent. Full shape (x, y, length, width, yaw, color, confidence, radius). `color` is always `COLOR_UNKNOWN`; classify by aspect ratio if you need WALL vs OPPONENT. |
| `/perception/camera_detections`  | `roboracer_perception/DetectionArray`| ~5 Hz   | HSV detections from the ZED feed — colored by `COLOR_BLUE` (inner walls), `COLOR_YELLOW` (outer walls), `COLOR_RED` (opponent). `yaw=0.0` (a single 2D bounding box has no orientation). |
| `/perception/walls`              | `visualization_msgs/MarkerArray`     | ~100 Hz | RViz only — red LINE_STRIP per detected wall.                       |
| `/perception/opponent`           | `visualization_msgs/MarkerArray`     | ~100 Hz | RViz only — green CUBE on the opponent.                             |
| `/ego_racecar/odom`              | `nav_msgs/Odometry`                  | ~180 Hz | From gym_bridge (or VESC on hardware). Required by estimation/planning. |
| `/opp_racecar/odom`              | `nav_msgs/Odometry`                  | ~180 Hz | Sim only — no real opponent on hardware.                            |
| `/scan`                          | `sensor_msgs/LaserScan`              | ~200 Hz in sim, 40 Hz on HOKUYO | Raw LIDAR.       |
| `/zed/zed_node/rgb/image_rect_color` | `sensor_msgs/Image`              | ~5 Hz sim, 30 Hz hardware | Camera frame.                              |
| `/zed/zed_node/depth/depth_registered` | `sensor_msgs/Image`            | same    | Aligned depth.                                                      |
| `/zed/zed_node/rgb/camera_info`  | `sensor_msgs/CameraInfo`             | same    | Intrinsics.                                                         |

### Detection message schema

```
roboracer_perception/msg/Detection.msg:

uint8 COLOR_UNKNOWN = 0
uint8 COLOR_BLUE    = 1   # left / inner boundary
uint8 COLOR_YELLOW  = 2   # right / outer boundary
uint8 COLOR_ORANGE  = 3   # start/finish gate (unused in this scenario)
uint8 COLOR_RED     = 4   # opponent / dynamic obstacle

float64 x          # base_link frame
float64 y
uint8 color
float64 confidence # 0.0 – 1.0
float64 length     # PCA major axis (m), or bbox width for camera
float64 width      # PCA minor axis (m), or bbox height for camera
float64 yaw        # principal-axis orientation (rad). 0.0 if not computed.
float64 radius     # legacy size summary = max(length, width) / 2
```

`DetectionArray.msg` is just `std_msgs/Header header` + `Detection[] detections`.

### Drive interface (for control teammate)

Publish `ackermann_msgs/msg/AckermannDriveStamped` on `/drive`. Speed is in
m/s (forward positive), steering in radians (left positive). The simulator
expects this — no extra conversion needed.

## Edits made to vendored / pulled code

For full transparency, here is every change made under `deps/` or to
externally maintained files:

### `deps/f1tenth_gym_ros/f1tenth_gym_ros/gym_bridge.py`

The patched gym_bridge has a LIDAR-based "wall slowdown" safety system that
intercepts the driver's commanded speed and reduces it when the forward
clearance is small. For perception/demo work this was unhelpful (it made the
car crawl near walls). At line 527 the block is gated with `if False:`:

```python
# Original:
if recovery_cmd is None and self.ego_requested_speed != 0:
    ... clearance check ... applied_speed *= factor ...

# Patched:
if False:  # Wall slowdown disabled for demo — was hijacking commanded speed
    ... (same body, never executed) ...
```

The post-collision `_recovery_phase` reset (lines 388–420) still works — if
you actually hit a wall the car will reverse out and recover. Only the
preemptive slowdown is disabled.

### `deps/f1tenth_gym_ros/config/sim_moving_obstacle.yaml`

Spawn positions moved closer to lane centre so the LIDAR's forward cone has
clear space at startup:

```yaml
sx:  0.0
sy:  -0.3      # was -0.6 in Farhad's original
stheta: 0.0

sx1: 0.0
sy1: 0.5       # was +0.6 in Farhad's original
stheta1: 0.0
```

### Everything else in `patches/`

`setup_workspace.sh` copies a number of patched files into the vendored sim:

- `patches/gym_bridge.py` — wall-slowdown disable + safe-pose buffer + recovery
- `patches/gym_bridge_launch.py` — portable map/config selection
- `patches/sim_moving_obstacle.yaml` — spawn-position override
- `patches/gym_bridge_solid*.launch.py` — Farhad's cylinder-wall scenario
  launchers (kept for reference; the perception package now uses its own flat
  launch instead)

## Customising the track

Track geometry lives in two files that must stay in sync:

- `tools/generate_solid_oval_map.py` — generates the `.pgm` from constants
- `roboracer_perception/solid_wall_geometry.py` — same constants, used by
  `solid_wall_visualizer` to draw the 3-D cylinder walls

To change track length, lane width, or curve radius, edit BOTH files and
regenerate the maps:

```bash
cd ~/roboracer_ws/src/roboracer_perception
python3 tools/generate_solid_oval_map.py
python3 tools/generate_solid_oval_obstacles_map.py
```

The new `.pgm` is symlinked into `install/share/.../maps/` automatically —
no rebuild needed, just relaunch.

Current values (matching Farhad's original lane width, on a 3× extended
straight):

| Constant            | Value      |
|---------------------|-----------:|
| `STRAIGHT_X_MAX`    | 60.0 m     |
| `BASE_INNER_R`      | 1.5 m      |
| `BASE_OUTER_R`      | 3.5 m      |
| Lane width (straight) | 2.85 m   |
| Lane width (curve)  | 2.90 m     |
| Map `WIDTH`         | 3040 px    |
| Map `HEIGHT`        | 1040 px    |
| Map `RESOLUTION`    | 0.025 m/px |

## Troubleshooting

### `cv_bridge` crashes on import

`AttributeError: _ARRAY_API not found` → see "Critical Python environment fix"
above. Run `pip install --user 'numpy<2'` and re-source.

### `sim_camera_walls` shows "No Image" in the ZED panel

Check `ros2 topic hz /zed/zed_node/rgb/image_rect_color`. If it's zero, the
node crashed — check Terminal 1 for tracebacks (usually the numpy/cv_bridge
issue above).

### Camera rate is 1–2 Hz with huge variance

WSL2 system load. Close other heavy processes and restart the launch. The
sustained rate on a fresh WSL2 session is ~5–6 Hz. Real ZED on Jetson runs
at 30+ Hz, so this is purely a sim-side limitation that does not affect
deployment.

### Teleop key presses do nothing

- Check `ros2 topic echo /drive --field drive.speed` while pressing W. If
  the value goes to 4.0, teleop is publishing fine — the issue is rviz or
  the chase cam.
- If the value stays 0, your `DISPLAY` env var probably isn't set in T2.
  Run `echo $DISPLAY` — should print something like `:0`. If empty, open a
  fresh terminal from your WSL distro (not from a tmux session).

### Car doesn't move even though `/drive` shows 4.0

Look in Terminal 1 for `Wall slowdown:` log lines. If you see them, the
gym_bridge patch wasn't applied — re-run `bash setup_workspace.sh`.

### Map looks misaligned with the rendered wall cylinders

The `.pgm` and `solid_wall_geometry.py` constants are out of sync. Re-run
the generator scripts above to regenerate the `.pgm` from the current
constants.

### RViz camera shows car from the front after going around the bend

Verify the View Type in RViz is `ThirdPersonFollower`, NOT `Orbit`. Orbit
only follows position — when the car rotates 180° around a bend, Orbit
keeps the camera on the same world side, so you end up looking at the
front of the car. ThirdPersonFollower follows orientation too.

## Active nodes in `solid_wall_perception.launch.py`

For reference, these are the nodes the launch starts:

| Node                          | Purpose                                                 |
|-------------------------------|---------------------------------------------------------|
| `gym_bridge`                  | sim physics, publishes /scan + odom + map               |
| `tf_base_to_laser`            | static TF (0.270, 0, 0.110)                             |
| `tf_base_to_zed`              | static TF (0.270, -0.005, 0.155)                        |
| `wall_opponent_detector`      | LIDAR PCA → /perception/detections                      |
| `moving_obstacle_controller`  | drives the opponent around the oval                     |
| `solid_wall_visualizer`       | publishes cylinder wall markers                         |
| `solid_wall_scan_highlighter` | colors laser hits on walls red                          |
| `sim_camera_walls`            | synthetic ZED feed for sim                              |
| `camera_processor_node`       | HSV detection → /perception/camera_detections           |
| `rviz2`                       | visualization                                           |

## Moving to real hardware (Jetson)

When deploying on the actual car, swap the simulator for real drivers but
keep the perception nodes unchanged — they were written against the same
topic names the real drivers publish.

**Remove from the launch:**
- `gym_bridge`
- `sim_camera_walls`
- `moving_obstacle_controller`

**Add to the launch:**
- `urg_node` (or `urg_node2`) for the HOKUYO LIDAR → publishes `/scan`
- `zed_wrapper` `zed_camera.launch.py` for the ZED 2i → publishes
  `/zed/zed_node/...` on identical topic names

`wall_opponent_detector`, `camera_processor`, `tf_base_to_laser`,
`tf_base_to_zed`, `solid_wall_visualizer`, `solid_wall_scan_highlighter`
all keep running unchanged.

Also: re-tune the HSV ranges in `config/perception_params.yaml` against real
lighting. The synthetic colors used in sim are pure BGR primaries; real-world
yellow/blue/red span a wider hue range and you'll want to widen the
thresholds.

Hardware coordinate frame matches sim — see the static transforms in the
launch file.
