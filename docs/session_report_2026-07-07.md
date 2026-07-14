# Session Report — On-Car SLAM Navigation (2026-07-07)

Robot: RoboRacer / F1TENTH (NVIDIA Jetson, Ubuntu 22.04, ROS 2 Humble)
Access: `ssh roboracer@192.168.50.10`
ROS domain: `ROS_DOMAIN_ID=7` (the whole robot stack lives on domain 7; domain 0 shows nothing)

## Goal of the session

See the environment map live and command the car to a goal point on it. This
evolved into a cleaner approach: instead of localizing on a pre-saved map, run
SLAM live so the car's **current position becomes the origin (0,0,0)**, drive
out manually a short distance, then have Nav2 autonomously drive the car back to
the origin (an out-and-back test).

## Software stack brought up

Three independent, detached process groups were used so each can be restarted
without killing the others:

1. **Base sensor/drive stack** — `~/t_stack.sh` → `ros2 launch f1tenth_stack
   bringup_launch.py`. Provides:
   - Hokuyo LiDAR (`urg_node`) → `/scan` (~40 Hz)
   - VESC driver + odometry (`vesc_driver_node`, `vesc_to_odom_node`) → `/odom`
     (~38–50 Hz), `/sensors/core`, `odom → base_link` TF
   - Joystick (`joy_node`, DualSense) + `ackermann_mux`
   - Static TF `base_link → laser` = translation `[0.27, 0.0, 0.11]`
2. **Foxglove bridge** — `ros2 launch foxglove_bridge foxglove_bridge_launch.xml
   port:=8765`. Connected from the Foxglove **desktop** app on Windows via
   `ws://192.168.50.10:8765` (the browser web app blocks `ws://` due to
   mixed-content over https, so the desktop app is required).
3. **SLAM** — `ros2 run slam_toolbox async_slam_toolbox_node` with a real-car
   params file. Publishes `/map` + `map → odom` TF; the car's start spot = origin.
4. **Nav2 navigation servers** — `ros2 launch nav2_bringup navigation_launch.py`
   (planner + controller + bt_navigator + behaviors + smoother +
   velocity_smoother), consuming SLAM's `/map`. Plus `cmd_vel_to_ackermann`
   converting Nav2 `/cmd_vel` → `/drive` (into the mux at priority 10).

## Key fixes applied this session

- **SLAM params were sim-only.** `config/slam_params.yaml` targets simulator
  frames (`ego_racecar/odom`, `ego_racecar/base_link`). Wrote a real-car copy at
  `~/rr/slam_params_real.yaml` with `odom_frame: odom`, `base_frame: base_link`,
  `scan_topic: /scan`. Both `slam.launch.py` and `slam_mapping.launch.py` in the
  repo start the `f1tenth_gym_ros` simulator, so they are NOT usable on hardware
  — slam_toolbox was run directly against the real `/scan` + odom TF.
- **Nav2 `/map` collision.** amcl + map_server (from `autonomous_real.launch.py`)
  publish `/map` and `map → odom`, which collide with slam_toolbox doing the
  same. Fix: tore down the Nav2 localization half and let SLAM own `/map`.
- **`use_composition:=True` hang.** Running `navigation_launch.py` standalone
  with composition tries to load nodes into a `nav2_container` that only
  `bringup_launch.py` creates → silent hang. Fix: relaunch with
  `use_composition:=False` (each nav server as its own process). Then all
  managed nodes activated.
- **Stale FastRTPS shared memory.** After killing Nav2, ~100+ stale
  `/dev/shm/fastrtps_*` segments accumulated and made the next DDS bring-up hang.
  Fix: kill all ROS, `rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*`, relaunch.
- **Duplicate base stack.** A second `f1tenth_stack bringup` was started from
  another login shell (user's own second terminal at 192.168.50.195). Two VESC
  drivers contended for the USB and knocked out `/odom` telemetry. Fix: killed
  the duplicate by PID, restarted the single base stack cleanly. VESC device
  (`/dev/sensors/vesc → /dev/ttyACM0`) was healthy, so a software restart
  recovered `/odom` (no power-cycle needed).
- **System clock jump.** The Jetson clock jumped from 1970 to the correct 2026
  time mid-session (no RTC/NTP on the isolated robot LAN). A SLAM instance
  launched under the old clock dropped every scan (discontinuous TF buffer).
  Fix: relaunch SLAM under the stable clock. Residual sparse "queue is full"
  drops (~1 per 2.5 s) are benign at 40 Hz input.
- **`pkill -f <string>` self-match footgun.** `pkill -f "vesc_driver_node"`
  matched the running SSH command line itself and killed the shell (exit 255).
  Rule: kill by explicit PID or process-group (`kill -TERM -<pgid>`), never
  `pkill -f` with a string that appears in the command being run.

## Out-and-back test result

- SLAM tracked the manual drive-out perfectly: bt_navigator reported the car at
  **(3.13, 0.00)** relative to the origin.
- Goal `(0,0,0)` was accepted: *"Begin navigating from current location
  (3.13, 0.00) to (0.00, 0.00)."*
- **Planner failed to find a path** — *"GridBased: failed to create plan, no
  valid path found."* `/cmd_vel` stayed 0.0; the car did not move.

### Cause and next step

The SMAC-Hybrid planner is set to **`motion_model_for_search: DUBIN`
(forward-only)** with `minimum_turning_radius: 0.90 m`. The car drove straight
forward to (3.13, 0) facing +x. Returning to (0,0) without reversing needs a
U-turn loop that doesn't fit the corridor → no valid path. The params file
itself notes the fix: switch to **`REEDS_SHEPP`** to allow reverse maneuvers so
the car can simply back up to the origin.

**Pending action (interrupted):** set
`GridBased.motion_model_for_search = REEDS_SHEPP` (runtime `ros2 param set`, or
edit `config/nav2_params_real.yaml`) and re-send the `(0,0,0)` goal.

## Helper scripts on the robot (`~/rr/`)

- `rr/slam_params_real.yaml` — real-car slam_toolbox params (frames fixed).
- `rr/rr_goal.sh X Y YAW` — publish `/goal_pose` (PoseStamped, frame `map`).
- `rr/rr_initpose.sh X Y YAW` — publish `/initialpose` for amcl (not needed in
  the SLAM-origin workflow).
- `rr/rr_healthcheck.sh` — full-chain PASS/FAIL diagnostic.

## Safety model

Manual joystick teleop is mux priority **100**; Nav2 is priority **10**.
Grabbing the stick + holding the deadman instantly overrides Nav2 — that is the
e-stop. First-run speed capped to 0.5 m/s in `nav2_params_real.yaml`.

## Current state at end of session

Base stack, Foxglove bridge, SLAM (map building, `map → odom` live), and Nav2
navigation servers are all up and wired end-to-end (`/goal_pose` → bt_navigator,
`/cmd_vel` → converter → `/drive`). The only blocker to the out-and-back test is
the DUBIN vs REEDS_SHEPP planner setting described above.
