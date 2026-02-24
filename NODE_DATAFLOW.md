# F1TENTH Simulator Node and Data Flow

This document describes the ROS node graph, topic flow, and the expected input/output structure for the `f1tenth_simulator` package.

## 1) End-to-End Flow Diagram

```mermaid
graph TD
  %% External / user inputs
  JoyNode[joy_node\n(sensor_msgs/Joy)]
  KeyboardNode[keyboard\n(std_msgs/String)]
  RandomWalk[random_walk planner\n(ackermann_msgs/AckermannDriveStamped)]
  NavPlanner[nav planner (optional)\n(ackermann_msgs/AckermannDriveStamped)]
  NewPlanner[new planner (optional)\n(ackermann_msgs/AckermannDriveStamped)]
  PoseTool[RViz 2D Pose Estimate / external pose publisher]
  ClickedPoint[RViz Publish Point\n(geometry_msgs/PointStamped)]
  MapServer[map_server\n(nav_msgs/OccupancyGrid)]

  %% Core nodes
  Behavior[behavior_controller]
  Mux[mux_controller]
  Sim[simulator]

  %% Visualization / consumers
  RViz[RViz]
  PlanningStack[Planner/Control stack]
  Logger[Collision log file]

  %% Topic flow
  JoyNode -->|/joy| Behavior
  KeyboardNode -->|/key| Behavior
  Sim -->|/scan (LaserScan)| Behavior
  Sim -->|/odom (Odometry)| Behavior
  Sim -->|/imu (Imu)| Behavior
  Sim -->|/brake_bool (Bool)| Behavior

  Behavior -->|/mux (Int32MultiArray)| Mux
  Behavior -->|collision events| Logger

  JoyNode -->|/joy| Mux
  KeyboardNode -->|/key| Mux
  RandomWalk -->|/rand_drive| Mux
  NavPlanner -->|/nav| Mux
  NewPlanner -->|/new_drive| Mux

  Mux -->|/drive (AckermannDriveStamped)| Sim

  MapServer -->|/map| Sim
  PoseTool -->|/initialpose or /pose| Sim
  ClickedPoint -->|/clicked_point| Sim

  Sim -->|/scan| RViz
  Sim -->|/odom| RViz
  Sim -->|/imu| RViz
  Sim -->|/gt_pose| RViz
  Sim -->|TF: map->base_link, base_link->laser| RViz
  Sim -->|/map (with added obstacles)| RViz

  Sim -->|/scan, /odom, /imu, /gt_pose| PlanningStack
```

## 2) Node I/O Contracts

| Node | Subscribes | Publishes | Purpose |
|---|---|---|---|
| `simulator` | `/drive`, `/map`, `/pose`, `/initialpose`, `/clicked_point` | `/scan`, `/odom`, `/imu`, `/gt_pose`, `/map` (updated with obstacles), TF transforms | Main vehicle + sensor simulator |
| `behavior_controller` | `/joy`, `/key`, `/scan`, `/odom`, `/imu`, `/brake_bool` | `/mux` | Selects active driver channel and handles safety/collision behavior |
| `mux_controller` | `/mux`, `/joy`, `/key`, planner channels (`/rand_drive`, `/nav`, `/new_drive`) | `/drive` | Routes one active control stream to simulator |
| `random_walk` | `/odom` | `/rand_drive` | Example autonomous planner |
| `keyboard` | (terminal keypresses) | `/key` | Keyboard teleop key publisher |
| `joy_node` | (joystick device) | `/joy` | Joystick driver |
| `map_server` | (map file) | `/map` | Static map provider |

## 3) Functional Data Flow (Control Loop)

1. `simulator` publishes sensor/state (`/scan`, `/odom`, `/imu`, TF).
2. `behavior_controller` listens to sensors + input devices and publishes a mux selection vector (`/mux`).
3. `mux_controller` forwards the currently enabled command source to `/drive`.
4. `simulator` consumes `/drive`, updates dynamics, and repeats at `update_pose_rate`.
5. If TTC collision is detected, simulator can force-stop; behavior controller can also clear active mux channels.

## 4) Input/Output Message Structure Summary

### 4.1 Drive command path

- **Planner/teleop output**: `ackermann_msgs/AckermannDriveStamped`
  - `drive.speed` (m/s)
  - `drive.steering_angle` (rad)
- **Mux output to simulator**: `/drive` same message type

### 4.2 LiDAR output

- **Topic**: `/scan`
- **Type**: `sensor_msgs/LaserScan`
- **Fields used**:
  - `header.stamp`, `header.frame_id`
  - `angle_min`, `angle_max`, `angle_increment`
  - `range_max`
  - `ranges[]`, `intensities[]`

### 4.3 Vehicle state output

- **Odometry** (`/odom`, `nav_msgs/Odometry`):
  - Pose in `map` frame
  - Twist (`linear.x`, `angular.z`)
- **Ground truth pose** (`/gt_pose`, `geometry_msgs/PoseStamped`)
- **TF**:
  - `map -> base_link`
  - `base_link -> laser`
  - steering wheel hinge transforms for visualization

### 4.4 Mux control vector

- **Topic**: `/mux`
- **Type**: `std_msgs/Int32MultiArray`
- **Semantics**:
  - One-hot style array matching configured mux indices (`joy`, `keyboard`, `random_walk`, `brake`, `nav`, etc.)
  - If all zero, mux commands zero speed/steer stop

### 4.5 Map and obstacle interaction

- **Map in**: `/map` (`nav_msgs/OccupancyGrid`) from `map_server`
- **Dynamic obstacle add**: `/clicked_point` (`geometry_msgs/PointStamped`)
- **Map out**: simulator republishes modified `/map` with temporary obstacles

## 5) Launch-Time Topology (default)

From `launch/simulator.launch`, the default active stack is:

- `joy_node`
- `map_server`
- `robot_state_publisher` (via `racecar_model.launch`)
- `simulator`
- `mux_controller`
- `behavior_controller`
- `random_walk`
- `keyboard`
- `rviz`

This gives a complete closed-loop simulation with manual and autonomous control channels.
