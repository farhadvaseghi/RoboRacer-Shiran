# F1TENTH Autonomous Racing – Navigation and Path Tracking

This branch contains the current setup for **autonomous navigation and path tracking** on the F1TENTH simulator.

The workflow is intentionally split into two parts:
1. **Planning**: Nav2 computes a global path on the map and publishes it on `/plan`
2. **Control / Tracking**: a standalone **Pure Pursuit controller** subscribes to `/plan` and publishes `AckermannDriveStamped` commands to `/nav`

The main focus of this branch is therefore **navigation + trajectory tracking**.  
SLAM is kept in the repository as an optional utility, but it is **not** part of the primary evaluation workflow.

---

## Team

| Name                             | Responsibility                                |
| -------------------------------- | --------------------------------------------- |
| Mohammadsadegh Shoushtaridehshal | Team Lead / Perception / Autonomy Integration |
| Farhad Vaseghi                   | Perception / Autonomy Integration             |
| Milad Bahari Qaragoz             | Estimation                                    |
| Kazhal Shirvani                  | Planning                                      |
| **Mohammad Barabadi**            | **Control / Path Tracking**                   |

---

## Main Contribution in This Branch

The main extension in this branch is the explicit integration of the **control layer** into the navigation pipeline.

### What remains from the planning side
- Nav2 still generates the global path
- the path is published on `/plan`
- RViz is used to send navigation goals and visualize the resulting path

### What was added on the control side
- a custom standalone **Pure Pursuit controller**
- subscription to `/plan` for global path tracking
- subscription to `/odom` for vehicle state feedback
- direct publication of `AckermannDriveStamped` commands to `/nav`

This separates **path generation** from **path tracking** and makes the control contribution clearly visible in the system architecture.

---

## Why the Controller Was Extended

A basic forward-only Pure Pursuit controller is usually sufficient when the useful tracking target lies in front of the vehicle.  
However, during testing we observed two important limitations:

1. **Some goals are not naturally reachable by forward motion only**  
   If the goal or the next relevant target on the path is behind the vehicle with respect to its current heading, a forward-only controller may fail or behave unstably.

2. **Sharp turns can lead to corner-cutting**  
   With an overly aggressive lookahead distance or high speed in tight corners, the vehicle may overshoot the path and move too close to walls.

To address these issues, the controller was extended with:
- **forward / reverse motion handling**
- **reduced lookahead distance** for tighter path following
- **speed reduction in sharp corners**

These updates make the controller more robust in narrow turns, close-to-wall scenarios, and goals that require backing up instead of forcing a forward maneuver.

---

## Installation & Setup

### Install Nav2 dependencies

```bash
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

### Build the workspace

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select f1tenth_simulator
source install/setup.bash
```

---

## Main Workflow: Navigation + Tracking

Open the following terminals in order.

### Terminal 1 — Simulator

```bash
cd ~/ros2_ws && source install/setup.bash
ros2 launch f1tenth_simulator simulator.launch.py
```

Wait until the simulator and map are visible.

### Terminal 2 — Navigation stack

```bash
cd ~/ros2_ws && source install/setup.bash
ros2 launch f1tenth_simulator navigation.launch.py
```

This launches the planning stack and the nodes required for the current navigation workflow.

### Terminal 3 — Nav2 RViz

```bash
source ~/ros2_ws/install/setup.bash
ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```

Use this RViz window to send a goal using **Nav2 Goal**.

> `rviz2` only **subscribes** to `/plan` for visualization.  
> It does **not** generate the path and it does **not** control the robot.

### Terminal 4 — Activate navigation mode

```bash
ros2 topic pub --once /key std_msgs/msg/String "data: 'n'"
```

You should see `Navigation turned on` in the simulator terminal.

### Send a goal

In the Nav2 RViz window:
- click **Nav2 Goal**
- choose a destination on the map

Then:
- Nav2 computes the global path
- the path is published on `/plan`
- the Pure Pursuit controller tracks this path and publishes commands to `/nav`

If the robot stops after a collision reset, enable navigation again:

```bash
ros2 topic pub --once /key std_msgs/msg/String "data: 'n'"
```

---

## Control-Focused Architecture

```text
Nav2 planner   →   /plan   →   Pure Pursuit Controller   →   /nav
                                 ↑                             |
                                 |                             |
                               /odom                           v
                                                     mux_controller (index 4)
                                                               |
                                                             /drive
                                                               |
                                                           simulator
```

### Important clarification

- `/plan` is published by the **Nav2 planner**
- `rviz2` may also subscribe to `/plan`, but only for path visualization
- the **Pure Pursuit controller** is the node responsible for consuming the path for tracking
- `/nav` is the Ackermann control topic used by the simulator pipeline

---

## Controller Contribution (Mohammad)

The main control work in this branch is the standalone **Pure Pursuit path tracking** node.

### Inputs

| Topic | Type | Purpose |
|------|------|---------|
| `/plan` | `nav_msgs/msg/Path` | Global path generated by Nav2 |
| `/odom` | `nav_msgs/msg/Odometry` | Current vehicle pose and velocity |

### Output

| Topic | Type | Purpose |
|------|------|---------|
| `/nav` | `ackermann_msgs/msg/AckermannDriveStamped` | Control command generated by Pure Pursuit |

### Core controller tasks
The controller:
- receives the latest global path from `/plan`
- finds the nearest forward point on the path
- selects a lookahead target for tracking
- computes the steering command using the Pure Pursuit law
- publishes Ackermann steering and speed commands to `/nav`

This replaces the path-tracking responsibility of the default Nav2 controller in the current control-focused workflow.

---

## Reverse Motion Logic

### Why reverse was needed
During testing, some target positions were reachable only if the vehicle was allowed to move backward.  
In those cases, a purely forward controller either:
- tried to force a large forward turn
- became unstable near the target
- or left the drivable map area

### How it works
The selected target point is transformed from the global map frame into the **robot local frame**.

In the robot local frame:
- `x_local > 0` means the target lies **in front** of the robot
- `x_local < 0` means the target lies **behind** the robot

Based on this:
- the controller stays in **forward mode** if the target is in front
- the controller switches to **reverse mode** if the target is behind

A small hysteresis was added between forward and reverse switching in order to avoid unstable oscillation near the transition boundary.

### Practical effect
This allows the robot to:
- back into targets that are behind its current heading
- avoid forcing unnecessary forward loops
- handle more goal configurations in a stable way

---

## Corner Handling Improvements

A standard Pure Pursuit controller can suffer from **corner-cutting**, especially in narrow turns or when the lookahead distance is too large.  
In practice, this means the vehicle may try to aim too far ahead and cut across a turn instead of following the path closely.

To reduce this problem, two practical improvements were introduced.

### 1. Reduced Lookahead Distance

The lookahead distance was reduced so that the target remains closer to the vehicle in tight turns.

This helps the controller:
- follow the planned path more closely
- reduce overshoot in corners
- avoid aggressive jumps toward far-away target points

A smaller lookahead is especially useful in:
- tight bends
- narrow map sections
- wall-adjacent turns

### 2. Reduced Speed in Sharp Corners

The controller also reduces the commanded speed when the required steering angle becomes large.

This improves:
- stability during sharp turns
- tracking accuracy near corners
- safety near walls and narrow passages

In other words:
- **straight sections** can still be driven faster
- **sharp corners** are handled more conservatively

This combination of smaller lookahead and lower speed in sharp turns significantly reduces the tendency to leave the path or clip the corner too aggressively.

---

## Topics to Monitor During Debugging

```bash
ros2 topic echo /plan
ros2 topic echo /odom
ros2 topic echo /nav
ros2 topic echo /drive
```

Useful graph inspection commands:

```bash
ros2 topic info /plan -v
ros2 topic info /nav -v
ros2 node list
```

---

## Key Files

| File | Purpose |
|------|---------|
| `launch/simulator.launch.py` | Starts the simulator |
| `launch/navigation.launch.py` | Starts the planning/navigation stack |
| `config/nav2_params.yaml` | Nav2 planning-related parameters |
| `pure_pursuit_controller.cpp` | Standalone Pure Pursuit tracking node |
| `params.yaml` | Vehicle limits, mux, simulator settings |

---

## Vehicle Limits

| Parameter | Value |
|-----------|-------|
| `max_speed` | 7.0 m/s |
| `max_steering_angle` | 0.4189 rad (~24°) |
| `max_accel` | 7.51 m/s² |
| `max_decel` | 8.26 m/s² |
| `max_steering_vel` | 3.2 rad/s |
| `wheelbase` | 0.3302 m |

---

## TF Structure

```text
map --(static identity)--> odom --(simulator)--> base_link --(static)--> laser
```

In this setup, `map -> odom` is a static identity transform, so the planned path and odometry stay consistent for simulator-based path tracking.

---

## Legacy / Optional: SLAM

SLAM is not part of the current main workflow, but the repository still includes the files needed to build a map if required later.

Relevant files:
- `launch/slam.launch.py`
- `config/slam_params.yaml`

This section is optional and not required for evaluating the current **navigation + control** pipeline.
