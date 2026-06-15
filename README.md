# RoboRacer-Shiran

ROS 2 Humble workspace for RoboRacer simulation, state estimation, Nav2 path
planning, and Ackermann path tracking. The repository is maintained by a
five-member project team; package ownership should remain separated so each
subsystem can be developed and tested independently.

## Architecture

The runtime is divided into three main parts:

1. **Simulation and estimation**
   - Starts `f1tenth_gym_ros` on `solid_oval_track`.
   - Publishes simulator odometry on `/ego_racecar/odom`.
   - Adds usable covariance values through `adaptive_covariance_node`.
   - Publishes EKF output on `/odometry/filtered`.

2. **Navigation and path planning**
   - Starts the Nav2 navigation stack and SMAC Hybrid-A* planner.
   - Publishes the generated path on `/plan`.
   - `path_relay_node` retains the latest path on `/control/plan` so the
     controller may start before or after a goal is sent.
   - Nav2's built-in `controller_server` remains part of the navigation action,
     but its velocity output is isolated on `/navigation/cmd_vel` and does not
     command the simulator.

3. **Vehicle control**
   - Starts the custom Pure Pursuit controller from `roboracer_control`.
   - Subscribes to `/control/plan` and `/odometry/filtered`.
   - Publishes `ackermann_msgs/AckermannDriveStamped` commands on `/drive`.
   - The simulator bridge subscribes to `/drive` and applies speed and steering.

The previous Stanley controller has been removed. The custom Pure Pursuit node
is the only controller connected to the simulator's `/drive` input.

## Main Topic Flow

```text
/ego_racecar/odom
        |
        v
adaptive_covariance_node -> /estimation/sim_odom_adaptive
        |
        v
ekf_filter_node -> /odometry/filtered --------------------+
                                                          |
Nav2 planner -> /plan -> path_relay -> /control/plan -----+-->
                                                       pure_pursuit_controller
                                                                  |
                                                                  v
                                                               /drive
                                                                  |
                                                                  v
                                                          simulator bridge
```

## Packages

### `roboracer_estimation`

Contains simulation integration, adaptive covariance handling, EKF launch and
configuration, Nav2 configuration, maps, and the retained path relay.

Important files:

- `launch/sim.launch.py`: simulator, RViz, adaptive covariance node, and EKF.
- `launch/navigation.launch.py`: Nav2 navigation/planning and path relay.
- `launch/estimation.launch.py`: standalone estimation launch for simulation or
  real hardware.
- `config/ekf_sim.yaml`: simulator EKF configuration using
  `ego_racecar/odom` and `ego_racecar/base_link` frames.
- `config/ekf_real.yaml`: real-hardware sensor fusion configuration.
- `config/nav2_params.yaml`: planner, costmap, behavior-tree, and Nav2 controller
  parameters.
- `roboracer_estimation/path_relay_node.py`: republishes `/plan` as retained
  `/control/plan`.

### `roboracer_control`

Contains the custom Pure Pursuit path-tracking controller.

Important files:

- `src/pure_pursuit_controller.cpp`: steering, speed, reverse handling,
  tracking-error output, and RViz debug markers.
- `config/controller_params.yaml`: topics, vehicle geometry, speed, lookahead,
  steering limits, and goal tolerance.
- `launch/controller.launch.py`: standalone controller launch with visible INFO
  logging.

### `deps/f1tenth_gym_ros`

Vendored simulator bridge and configuration. The active autonomous scenario
uses `sim_moving_obstacle.yaml` with keyboard `/cmd_vel` control disabled, so
vehicle commands come from `/drive` only.

## Build

From the ROS 2 workspace root:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Rebuild after changing C++ source, launch files, parameters, or Python entry
points.

## Run

Open three terminals. Source the workspace in every terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

### Terminal 1: Simulator and estimation

```bash
ros2 launch roboracer_estimation sim.launch.py
```

This launch also starts RViz.

### Terminal 2: Navigation and planning

```bash
ros2 launch roboracer_estimation navigation.launch.py
```

### Terminal 3 — RViz
```bash
source ~/ros2_ws/install/setup.bash
ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```

### Terminal 4: Pure Pursuit control

```bash
ros2 launch roboracer_control controller.launch.py
```

Use RViz's **Nav2 Goal** tool to select a target. The recommended order is
simulator, navigation, controller, then goal. Because `/control/plan` is
retained, it is also valid to send the goal before launching the controller.

The stack uses wall time by default because this simulator configuration does
not publish `/clock`.

## Controller Topics

| Topic | Type | Direction | Purpose |
|---|---|---|---|
| `/control/plan` | `nav_msgs/Path` | Input | Latest retained Nav2 path |
| `/odometry/filtered` | `nav_msgs/Odometry` | Input | EKF vehicle state |
| `/drive` | `ackermann_msgs/AckermannDriveStamped` | Output | Vehicle speed and steering command |
| `/tracking_error` | `geometry_msgs/Vector3Stamped` | Output | Cross-track and heading errors |
| `/control_debug_markers` | `visualization_msgs/MarkerArray` | Output | RViz controller visualization |

`/nav` is not used. It belonged to the removed Stanley controller.

## Controller Logs

The Pure Pursuit terminal reports:

- `Received path with ...`: confirms path reception.
- `mode`: forward or reverse tracking.
- `x`, `y`, `yaw`: estimated vehicle pose.
- `nearest`, `progress`, `target_idx`: path-tracking progress.
- `dist_goal`: remaining distance to the goal.
- `cte`, `heading_err`, `alpha`: tracking errors.
- `steer`: steering command in radians.
- `v_ref`, `v_meas`, `v_err`, `v_cmd`: speed reference, measurement, error,
  and command.
- `Goal reached`: confirms an intentional stop at the final waypoint.

## Quick Diagnostics

Check that the required streams exist:

```bash
ros2 topic echo /odometry/filtered --once
ros2 topic echo /control/plan --once
ros2 topic echo /drive --once
```

Inspect connections:

```bash
ros2 topic info /control/plan -v
ros2 topic info /drive -v
ros2 node list
```

Expected `/drive` endpoints:

- Publisher: `/pure_pursuit_controller`
- Subscriber: `/bridge`

If the controller starts but does not publish `/drive`, first verify that both
`/control/plan` and `/odometry/filtered` contain messages.

## Real-Hardware Estimation

The estimation package also supports ZED IMU/odometry and VESC odometry. Use
the standalone estimation launch and select real-hardware mode:

```bash
ros2 launch roboracer_estimation estimation.launch.py is_sim:=false use_sim_time:=false
```

Review `config/ekf_real.yaml` and the actual hardware topic/frame names before
deployment.
