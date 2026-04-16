# F1TENTH Autonomous Racing – SLAM & Navigation

This branch (`planner-setup`) contains the configuration and launch files required to
map the environment and run the autonomous navigation stack.

---

## Team

| Name                             | Responsibility                                |
| -------------------------------- | --------------------------------------------- |
| Mohammadsadegh Shoushtaridehshal | Team Lead / Perception / Autonomy Integration |
| Farhad Vaseghi                   | Perception / Autonomy Integration             |
| Milad Bahari Qaragoz             | Estimation                                    |
| Kazhal Shirvani                  | Planning                                      |
| Mohammad Barabadi                | Control                                       |

---

## Installation & Setup

### Install Nav2 Dependencies

```bash
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

### Build the Workspace

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select f1tenth_simulator
source install/setup.bash
```

---

## Phase 1: Mapping (SLAM)

Use these commands to generate a map of the race track from scratch.

### 1. Start the Simulator

```bash
source install/setup.bash
ros2 launch f1tenth_simulator simulator.launch.py
```

### 2. Launch SLAM Node (new terminal)

```bash
source install/setup.bash
ros2 launch f1tenth_simulator slam.launch.py
```

### 3. Drive the car to build the map (new terminal)

Use the keyboard to cover the entire track until the map is complete in RViz.

```bash
ros2 run f1tenth_simulator keyboard
```

### 4. Save the map

```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/RoboRacer-Shiran/maps/my_track
```

---

## Phase 2: Navigation (Path Planning + Control)

The navigation stack uses the pre-built `levine.yaml` map. Path planning is fully
working — the robot plans paths across the full map and drives toward the goal.

Open **4 terminals** in this order.

### Terminal 1 — Simulator

```bash
cd ~/ros2_ws && source install/setup.bash
ros2 launch f1tenth_simulator simulator.launch.py
```

Wait until RViz opens and the levine map appears (~5 seconds).

### Terminal 2 — Navigation stack

```bash
cd ~/ros2_ws && source install/setup.bash
ros2 launch f1tenth_simulator navigation.launch.py
```

Wait ~10 seconds for Nav2 to fully activate (planner, controller, costmaps all go `active`).

### Terminal 3 — Nav2 RViz

```bash
source ~/ros2_ws/install/setup.bash
ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```

### Terminal 4 — Activate navigation mode

```bash
ros2 topic pub --once /key std_msgs/msg/String "data: 'n'"
```

You should see `Navigation turned on` in Terminal 1.

### Send a goal

In the Nav2 RViz window click **Nav2 Goal** in the toolbar, then click a destination on
the white (free) area of the map. The robot will plan a path and drive to it.

> **If the robot stops mid-path:** a collision was detected and the mux reset.
> Re-run the Terminal 4 command to re-enable navigation.

---

## Navigation Architecture

```
Nav2 planner  →  /plan  →  Nav2 controller  →  /cmd_vel_nav
                                                      |
                                            velocity_smoother
                                                      |
                                                  /cmd_vel
                                                      |
                                          twist_to_ackermann.py
                                                      |
                                                    /nav
                                                      |
                                          mux_controller (index 4)
                                                      |
                                                   /drive
                                                      |
                                                 simulator
```

**TF tree:**

```
map --(static identity)--> odom --(simulator)--> base_link --(static)--> laser
```

The `map → odom` transform is a static identity because the simulator publishes
the robot's true world position as `odom → base_link` — no AMCL or SLAM needed
for navigation.

---

## For the Controller Teammate (Mohammad)

Path planning is complete and working. The robot receives a goal, plans a path over
the levine map, and publishes it. Your job is to make the robot follow that path well.

### Where the planned path comes from

```
/plan   (nav_msgs/msg/Path, frame: map)
```

Inspect it live:

```bash
ros2 topic echo /plan
```

It also appears as a green line in the Nav2 RViz window after a goal is set.

### Current controller

Configured in `config/nav2_params.yaml` under `controller_server.FollowPath`:

```yaml
FollowPath:
  plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
  desired_linear_vel: 2.0
  lookahead_dist: 1.5
  min_lookahead_dist: 0.5
  max_lookahead_dist: 2.5
  lookahead_time: 1.5
  rotate_to_heading_angular_vel: 1.8
  transform_tolerance: 0.1
  use_velocity_scaled_lookahead_dist: True
  min_approach_linear_velocity: 0.05
  use_collision_aware_lookahead: False
```

### How to replace or tune the controller

**Option A — Tune the existing pure pursuit controller:**
Edit the parameters above in `config/nav2_params.yaml`.

**Option B — Swap in your own Nav2 controller plugin:**
Change the `plugin:` field to your custom class and add its parameters.
Your controller must implement the `nav2_core::Controller` interface.
Reference: https://navigation.ros.org/plugin_tutorials/docs/writing_new_nav2controller_plugin.html

**Option C — Write a standalone path-following node:**
Subscribe to `/plan` (`nav_msgs/msg/Path`) and publish `AckermannDriveStamped`
directly to `/nav`. Then remove `twist_to_ackermann.py` from `navigation.launch.py`.

### Command output topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel_nav` | `geometry_msgs/Twist` | Raw controller output |
| `/cmd_vel` | `geometry_msgs/Twist` | After velocity smoother |
| `/nav` | `ackermann_msgs/AckermannDriveStamped` | After Twist→Ackermann conversion |
| `/drive` | `ackermann_msgs/AckermannDriveStamped` | Final command to simulator |

### Vehicle limits (from `params.yaml`)

| Parameter | Value |
|-----------|-------|
| `max_speed` | 7.0 m/s |
| `max_steering_angle` | 0.4189 rad (~24°) |
| `max_accel` | 7.51 m/s² |
| `max_decel` | 8.26 m/s² |
| `max_steering_vel` | 3.2 rad/s |
| `wheelbase` | 0.3302 m |

### Re-enabling nav after a collision stop

```bash
ros2 topic pub --once /key std_msgs/msg/String "data: 'n'"
```

The TTC threshold (sensitivity) is in `params.yaml`:
```yaml
ttc_threshold: 0.01   # seconds — lower = more sensitive
```

---

## Key Files

| File | Purpose |
|------|---------|
| `launch/simulator.launch.py` | Simulator, map server, RViz |
| `launch/navigation.launch.py` | Nav2 stack + map server + static TF |
| `launch/slam.launch.py` | Standalone SLAM for map building |
| `config/nav2_params.yaml` | All Nav2 parameters |
| `config/slam_params.yaml` | SLAM toolbox parameters |
| `roboracer_perception/twist_to_ackermann.py` | Converts Twist → AckermannDriveStamped |
| `params.yaml` | Simulator, mux, vehicle dynamics |

## Important Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | `sensor_msgs/LaserScan` | LiDAR (1080 beams, 360°) |
| `/odom` | `nav_msgs/Odometry` | Odometry |
| `/plan` | `nav_msgs/Path` | Planned path from Nav2 |
| `/drive` | `ackermann_msgs/AckermannDriveStamped` | Final drive command |
| `/map` | `nav_msgs/OccupancyGrid` | Navigation map |
| `/gt_pose` | `geometry_msgs/PoseStamped` | Ground-truth pose |
| `/mux` | `std_msgs/Int32MultiArray` | Mux channel state |
