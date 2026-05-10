# How to Run the RoboRacer Simulation

This document explains how to run the simulator and explicitly records the modifications added in this project.

## What Was Modified

The base `f1tenth_gym_ros` simulator was extended with:

- portable map/config selection in [patches/gym_bridge_launch.py](/home/farhad/roboracer_ws/src/roboracer_perception/patches/gym_bridge_launch.py:1)
- patched simulator bridge logic in the vendored [deps/f1tenth_gym_ros/f1tenth_gym_ros/gym_bridge.py](/home/farhad/roboracer_ws/src/roboracer_perception/deps/f1tenth_gym_ros/f1tenth_gym_ros/gym_bridge.py:1)
  - collision recovery from older safe poses
  - wall/obstacle slowdown behavior
  - safer reverse behavior
  - two-agent stepping support for the moving-obstacle scenario
- new map generators:
  - [tools/generate_solid_oval_map.py](/home/farhad/roboracer_ws/src/roboracer_perception/tools/generate_solid_oval_map.py:1)
  - [tools/generate_solid_oval_obstacles_map.py](/home/farhad/roboracer_ws/src/roboracer_perception/tools/generate_solid_oval_obstacles_map.py:1)
- new scenario launch files:
  - [patches/gym_bridge_solid.launch.py](/home/farhad/roboracer_ws/src/roboracer_perception/patches/gym_bridge_solid.launch.py:1)
  - [patches/gym_bridge_solid_cylinder.launch.py](/home/farhad/roboracer_ws/src/roboracer_perception/patches/gym_bridge_solid_cylinder.launch.py:1)
  - [patches/gym_bridge_solid_obstacles.launch.py](/home/farhad/roboracer_ws/src/roboracer_perception/patches/gym_bridge_solid_obstacles.launch.py:1)
  - [patches/gym_bridge_solid_obstacles_cylinder.launch.py](/home/farhad/roboracer_ws/src/roboracer_perception/patches/gym_bridge_solid_obstacles_cylinder.launch.py:1)
  - [patches/gym_bridge_solid_moving_obstacle.launch.py](/home/farhad/roboracer_ws/src/roboracer_perception/patches/gym_bridge_solid_moving_obstacle.launch.py:1)
  - [patches/gym_bridge_solid_moving_obstacle_cylinder.launch.py](/home/farhad/roboracer_ws/src/roboracer_perception/patches/gym_bridge_solid_moving_obstacle_cylinder.launch.py:1)
  - [patches/gym_bridge_solid_static_moving_obstacles.launch.py](/home/farhad/roboracer_ws/src/roboracer_perception/patches/gym_bridge_solid_static_moving_obstacles.launch.py:1)
  - [patches/gym_bridge_solid_static_moving_obstacles_cylinder.launch.py](/home/farhad/roboracer_ws/src/roboracer_perception/patches/gym_bridge_solid_static_moving_obstacles_cylinder.launch.py:1)
- a dedicated two-agent simulator config:
  - [patches/sim_moving_obstacle.yaml](/home/farhad/roboracer_ws/src/roboracer_perception/patches/sim_moving_obstacle.yaml:1)
- an autonomous controller for the second vehicle:
  - [roboracer_perception/moving_obstacle_controller.py](/home/farhad/roboracer_ws/src/roboracer_perception/roboracer_perception/moving_obstacle_controller.py:1)
- a patched RViz config that shows both ego and opponent in the moving-obstacle scenario:
  - [patches/gym_bridge.rviz](/home/farhad/roboracer_ws/src/roboracer_perception/patches/gym_bridge.rviz:1)
- cylinder wall visualization support:
  - [roboracer_perception/solid_wall_visualizer.py](/home/farhad/roboracer_ws/src/roboracer_perception/roboracer_perception/solid_wall_visualizer.py:1)
  - [roboracer_perception/solid_wall_geometry.py](/home/farhad/roboracer_ws/src/roboracer_perception/roboracer_perception/solid_wall_geometry.py:1)
- wall-hit LiDAR highlighting for cylinder scenarios:
  - [roboracer_perception/solid_wall_scan_highlighter.py](/home/farhad/roboracer_ws/src/roboracer_perception/roboracer_perception/solid_wall_scan_highlighter.py:1)
- updated automation in [setup_workspace.sh](/home/farhad/roboracer_ws/src/roboracer_perception/setup_workspace.sh:1)
  - regenerates the map variants
  - copies patched launch/config/rviz files
  - rebuilds the workspace

## Vendored Dependencies

This repository intentionally tracks the modified simulator dependencies as
regular folders:

- [deps/f1tenth_gym](/home/farhad/roboracer_ws/src/roboracer_perception/deps/f1tenth_gym:1)
- [deps/f1tenth_gym_ros](/home/farhad/roboracer_ws/src/roboracer_perception/deps/f1tenth_gym_ros:1)

Do not clone separate upstream copies over these directories. The local
versions contain compatibility and behavior changes needed by this project,
including the patched bridge, scenario launches, RViz config, map handling,
collision recovery, and teleop/reverse-driving behavior.

The dependency directories must not contain their own internal `.git`
directories. They are vendored so the main RoboRacer repository tracks their
source files directly and teammates get the same working simulator after a
normal clone.

## Current Track Geometry

The current solid-wall map is wider than the original 2.0 m lane and now gives
extra room on the inner side of the straights and turns.

- straight width: `2.85 m`
- curve width: `2.90 m`

This geometry is shared by:

- `solid_oval_track`
- `solid_oval_track_obstacles`
- the cylinder-wall marker geometry used in RViz

## Prerequisites

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10+
- Git

Install required packages if needed:

```bash
sudo apt install python3-colcon-common-extensions python3-rosdep \
    ros-humble-nav2-map-server ros-humble-nav2-lifecycle-manager \
    ros-humble-rviz2 ros-humble-robot-state-publisher \
    ros-humble-xacro ros-humble-rqt-image-view
python3 -m pip install transforms3d numpy opencv-python-headless scikit-learn scipy cv-bridge 'coverage>=7,<8'
```

## First-Time Setup

```bash
mkdir -p ~/roboracer_ws/src
git clone -b perception https://github.com/farhadvaseghi/RoboRacer-Shiran.git \
    ~/roboracer_ws/src/roboracer_perception
cd ~/roboracer_ws/src/roboracer_perception
bash setup_workspace.sh
```

The setup script:
- regenerates the custom map assets
- symlinks the vendored `f1tenth_gym_ros` and `f1tenth_gym` from `deps/` into `src/`
- copies the extra scenario launch/config/rviz files into `f1tenth_gym_ros`
- installs `f110_gym` via pip
- builds the full workspace

When it finishes you will see:
```bash
=== Setup complete ===
```

## Running the Simulation

Open the terminals you need for the scenario. In every terminal, source the workspace first:

```bash
source ~/roboracer_ws/install/setup.bash
```

## Available Launch Files

The project now has four scenario families, each with a flat-wall version and
an optional cylinder-wall visualization version.

### Default Oval

```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

Uses the original `oval_track`.

### Solid Walls Only

```bash
ros2 launch f1tenth_gym_ros gym_bridge_solid.launch.py
```

Uses `solid_oval_track`.

### Solid Walls Only With Cylinder Wall Visualization

```bash
ros2 launch f1tenth_gym_ros gym_bridge_solid_cylinder.launch.py
```

Uses the same `solid_oval_track` collision map, but adds raised horizontal
cylinder wall markers in RViz.

Important note:

- collision is still 2-D and still comes from the occupancy-grid map
- the cylinders are a visualization overlay, not true 3-D collision objects

### Solid Walls + Static Obstacles

```bash
ros2 launch f1tenth_gym_ros gym_bridge_solid_obstacles.launch.py
```

Uses `solid_oval_track_obstacles`.

Static obstacles currently added:

- lower half: center `(6.5, 0.55)`, radius `0.40 m`
- upper half: center `(13.5, 4.45)`, radius `0.40 m`

### Solid Walls + Static Obstacles With Cylinder Wall Visualization

```bash
ros2 launch f1tenth_gym_ros gym_bridge_solid_obstacles_cylinder.launch.py
```

Uses the same `solid_oval_track_obstacles` collision map, plus raised
horizontal cylinder wall markers in RViz.

### Solid Walls + Moving Obstacle Vehicle

```bash
ros2 launch f1tenth_gym_ros gym_bridge_solid_moving_obstacle.launch.py
```

This launches a separate two-agent scenario.

- You control only the ego vehicle
- The second vehicle is controlled automatically
- RViz shows both cars by default

### Solid Walls + Moving Obstacle Vehicle With Cylinder Wall Visualization

```bash
ros2 launch f1tenth_gym_ros gym_bridge_solid_moving_obstacle_cylinder.launch.py
```

Uses the same moving-obstacle scenario, plus raised horizontal cylinder wall
markers in RViz.

Current start poses in [sim_moving_obstacle.yaml](/home/farhad/roboracer_ws/src/roboracer_perception/patches/sim_moving_obstacle.yaml:1):

- ego: `(0.0, -0.6, 0.0)`
- opponent: `(0.0, 0.6, 0.0)`

Opponent behavior in the current controller:

- follows the oval autonomously
- uses a smoother bypass trajectory around the static obstacles when they exist
- slows slightly while crossing the bypass zone to avoid abrupt steering

### Solid Walls + Static And Dynamic Obstacles

```bash
ros2 launch f1tenth_gym_ros gym_bridge_solid_static_moving_obstacles.launch.py
```

Uses `solid_oval_track_obstacles` and also starts the moving obstacle
controller, so the scenario contains both the static map obstacles and the
dynamic opponent vehicle.

### Solid Walls + Static And Dynamic Obstacles With Cylinder Wall Visualization

```bash
ros2 launch f1tenth_gym_ros gym_bridge_solid_static_moving_obstacles_cylinder.launch.py
```

Uses the same combined static-plus-dynamic obstacle scenario, plus raised
horizontal cylinder wall markers and wall-hit LiDAR highlighting.

For cylinder scenarios, RViz also shows:

- `/visualization/solid_wall_markers`
- `/visualization/solid_wall_hit_markers`

The wall-hit overlay highlights scan returns that land on the cylinder-wall
geometry using elevated red points and red beam segments.

## Teleop

Run teleop in another terminal:

```bash
ros2 run roboracer_perception teleop_key
```

Controls:

- `W`: forward
- `S`: reverse
- `A`: steer left
- `D`: steer right
- `Space`: full stop
- `Q`: quit

Teleop publishes on `/drive`. In the moving-obstacle scenario, you do not control the opponent.

Important note:

- combined forward inputs like `W`+`A` and `W`+`D` are supported
- combined reverse inputs like `S`+`A` and `S`+`D` are also supported
- forward steering is slightly softer than before: `+/-0.18 rad`
- reverse steering is deliberately capped in the bridge to `+/-0.030 rad` to
  allow backing turns without returning to the earlier spin/stuck behavior

## Optional Camera Tools

Fake camera:

```bash
ros2 run roboracer_perception sim_camera
```

Image viewer:

```bash
ros2 run rqt_image_view rqt_image_view /zed/zed_node/rgb/image_rect_color
```

## Added Test Scripts

Automatic lap test:

```bash
python3 ~/roboracer_ws/src/roboracer_perception/tools/run_autolap_test.py
```

Near-wall and reverse regression:

```bash
python3 ~/roboracer_ws/src/roboracer_perception/tools/run_wall_reverse_regression.py
```

These assume the simulator is already running.

## Important Topics

One-agent scenarios:

- `/drive`
- `/scan`
- `/ego_racecar/odom`

Moving-obstacle scenario adds:

- `/opp_drive`
- `/opp_racecar/odom`
- `/opp_scan`

Cylinder-wall scenarios also add:

- `/visualization/solid_wall_markers`
- `/visualization/solid_wall_hit_markers`

## Bridge Behavior Notes

The patched bridge includes:

- reset-to-safe-pose collision recovery
- recovery phases after collision
- forward slowdown near walls and obstacles
- steer-aware forward slowdown path
- conservative reverse handling

All of that is implemented in the vendored [deps/f1tenth_gym_ros/f1tenth_gym_ros/gym_bridge.py](/home/farhad/roboracer_ws/src/roboracer_perception/deps/f1tenth_gym_ros/f1tenth_gym_ros/gym_bridge.py:1).

## Rebuilding After Changes

For normal code edits:

```bash
cd ~/roboracer_ws
colcon build --symlink-install --packages-ignore f110_gym
source install/setup.bash
```

For patch/map/launch changes, rerun the full setup:

```bash
cd ~/roboracer_ws/src/roboracer_perception
bash setup_workspace.sh
```

## Troubleshooting

**Second robot is not visible in the moving-obstacle scenario**

The RViz display comes from the patched [gym_bridge.rviz](/home/farhad/roboracer_ws/src/roboracer_perception/patches/gym_bridge.rviz:1). If needed:

```bash
cd ~/roboracer_ws/src/roboracer_perception
bash setup_workspace.sh
```

**Simulator fails to load the map**

Run `setup_workspace.sh` again so the vendored simulator symlinks and copied patch files are refreshed.

**Cylinder walls are visible but do not collide like 3-D objects**

That is expected. The simulator still uses a 2-D occupancy-grid map for
collision. The cylinders are an RViz overlay only.

**Wall-hit highlighting only appears in the cylinder scenarios**

That is expected. The highlighting node is only launched by the
`*_cylinder.launch.py` files.

**Teleop does nothing in the moving-obstacle scenario**

Use the dedicated moving-obstacle launch, not the one-agent launches:

```bash
ros2 launch f1tenth_gym_ros gym_bridge_solid_moving_obstacle.launch.py
```

**Map/launch edits have no effect**

Run `bash setup_workspace.sh` again. It recopies the patched files into `f1tenth_gym_ros`.

**Combined static + dynamic scenario feels different from the moving-only scenario**

That is expected. The opponent controller uses a different path shape in the
static-obstacle regions so it can go around the two fixed obstacles smoothly.

**Python changes have no effect**

Rebuild and re-source.

**`ModuleNotFoundError: f110_gym`**

```bash
python3 -m pip install -e ~/roboracer_ws/src/f1tenth_gym/
```

**RViz opens blank or shows map/TF errors**

```bash
python3 -m pip install --user --upgrade 'coverage>=7,<8'
cd ~/roboracer_ws
colcon build --symlink-install --packages-ignore f110_gym
source install/setup.bash
```
