# F1TENTH Racecar Simulator

This workspace contains a lightweight 2D F1TENTH simulator package for ROS2.
The project is set up for ROS2 Humble with `ament_cmake` and `colcon`.

## Project Context

This repository is used by an autonomy team project focused on autonomous racing workflows.
It supports end-to-end development across:

- perception
- estimation
- planning
- control

Primary goals:

- develop and integrate a complete autonomy stack
- test and validate behavior in simulation
- support transfer of methods to a real racecar platform

## Team

| Name                             | Responsibility                                |
| -------------------------------- | --------------------------------------------- |
| Mohammadsadegh Shoushtaridehshal | Team Lead / Perception / Autonomy Integration |
| Farhad Vaseghi                   | Perception / Autonomy Integration             |
| Milad Bahari Qaragoz             | Estimation                                    |
| Kazhal Shirvani                  | Planning                                      |
| Mohammad Barabadi                | Control                                       |

## Technologies

- ROS2 (Humble)
- C++
- Python
- simulation and autonomy algorithms

## Current Status

- ROS2 launch entrypoint: `launch/simulator.launch.py`
- Build system: `ament_cmake`
- Package name: `f1tenth_simulator`
- Main executables: `simulator`, `mux`, `behavior_controller`, `random_walk`, `keyboard`

## Workspace Layout

- `node/` ROS2 node source files
- `src/` simulator and kinematics library source files
- `include/f1tenth_simulator/` headers
- `launch/` launch files and RViz configuration
- `maps/` occupancy grid maps
- `params.yaml` runtime parameters

## Requirements

Target platform:

- Ubuntu 22.04
- ROS2 Humble

Runtime dependencies used by this package:

- `ackermann_msgs`
- `nav2_map_server`
- `joy`
- `tf2_geometry_msgs`
- `visualization_msgs`
- `robot_state_publisher`
- `rviz2`
- `xacro`

## Build

From your ROS2 workspace root:

```bash
colcon build --symlink-install
source install/setup.bash
```

Optional dependency resolution before build:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

## Run

Launch the full simulator stack:

```bash
ros2 launch f1tenth_simulator simulator.launch.py
```

The launch includes:

- `joy_node`
- `nav2_map_server`
- `robot_state_publisher`
- `simulator`
- `mux` (`mux_controller`)
- `behavior_controller`
- `random_walk`
- `keyboard`
- `rviz2`

## Basic Control

Default mode toggle keys (from `params.yaml`):

- `k` keyboard driving mode
- `j` joystick driving mode
- `r` random walker mode
- `b` brake mode
- `n` navigation channel mode

Keyboard driving keys:

- `w` accelerate
- `s` decelerate/reverse
- `a` steer left
- `d` steer right
- `space` stop

## Important Topics

- Drive command input: `/drive`
- LiDAR: `/scan`
- Odometry: `/odom`
- IMU: `/imu`
- Ground-truth pose: `/gt_pose`
- Manual pose set: `/pose`
- RViz initial pose: `/initialpose`
- Map: `/map`
- Mux channel select: `/mux`

## Parameters

All runtime configuration is in `params.yaml`, including:

- vehicle dynamics limits and geometry
- LiDAR model settings
- joystick/keyboard mappings
- mux channel indices
- topic and frame names

## Maps and Visualization

- Default map in launch: `maps/levine.yaml`
- RViz config: `launch/simulator.rviz`
- Robot model source: `racecar.xacro`

To switch maps, edit `map_file` in `launch/simulator.launch.py`.

## Documentation in This Workspace

- `ROS2_QUICK_START.md`
- `ROS2_MIGRATION.md`
- `migration.md`
- `NODE_DATAFLOW.md`
