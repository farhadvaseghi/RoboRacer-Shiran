# RoboRacer — Re-map with SLAM, then Drive Start→End Autonomously (Nav2)

Step-by-step runbook for the **physical car**. You run every step; nothing here
auto-runs. Verified against the live car on 2026-06-30.

This guide supersedes the earlier `slam-and-autonomous-guide.md` draft — the
autonomous part now uses the **full Nav2 stack** (installed today).

Connect: laptop on the `roboracer` Wi-Fi, then `ssh roboracer@192.168.50.10`.

---

## What runs, and where the path comes from

The car has the *follower* (your `roboracer_control` Pure Pursuit) and a *relay*,
but **no planner of its own** — the path (`/plan`) has always come from Nav2.
For this first hardware bring-up we use **Nav2 end-to-end** because it is fully
TF-aware (your Pure Pursuit is not — see "Phase 2" at the end):

```
~/t_stack.sh ── /scan, /odom, TF odom→base_link→laser, ackermann_mux
      │
autonomous_real.launch.py:
   map_server ── /map ────────────────┐
   amcl ── TF map→odom, /amcl_pose ────┤ localization
   planner (SMAC-Hybrid) ── /plan      │
   controller (Reg. Pure Pursuit) ─────┤ navigation
   bt_navigator / behaviors / smoother │
   velocity_smoother ── /cmd_vel       │
   cmd_vel_to_ackermann ── /drive ─────┘
      │
ackermann_mux (/drive prio 10) → ackermann_to_vesc → VESC → wheels
```

- **Start point** = where amcl thinks the car is (initial pose).
- **End point**   = the navigation goal you send.

## Safety (read once)

- The **joystick always wins**: `/teleop` is mux priority 100, autonomy `/drive`
  is priority 10. Keep the controller in hand — push the sticks (or release the
  deadman) to override instantly.
- If `/drive` goes silent for >0.2 s the mux commands **zero speed** (fail-safe).
- **First run is capped at 0.5 m/s** in `nav2_params_real.yaml` (hardware max is
  ~2.0 m/s). Do not raise it until a clean slow lap (see §7).
- First test: car on blocks or a wide clear area.

---

## 1. One-time deploy (from the PC)

Prepared files live on the PC. Push them to the car once (and again whenever you
change them). From **Git Bash** on the PC:

```bash
bash /c/Users/Student/Documents/Shiran-Hozuri/nav2-realcar-deploy/deploy.sh
```

This copies onto the car:
- `roboracer_estimation/config/nav2_params_real.yaml`  (real-car Nav2 params)
- `roboracer_estimation/launch/autonomous_real.launch.py`
- `~/rr/rr_healthcheck.sh`, `rr_initpose.sh`, `rr_goal.sh`, `rr_record.sh`

## 2. One-time build (on the car)

```bash
ssh roboracer@192.168.50.10
source /opt/ros/humble/setup.bash
cd ~/roboracer_ws
colcon build --packages-select roboracer_estimation
source install/setup.bash
# sanity: the new launch + params are installed
ros2 pkg executables roboracer_estimation | grep cmd_vel_to_ackermann
ls install/roboracer_estimation/share/roboracer_estimation/config/nav2_params_real.yaml
```

---

## 3. Pre-flight (every session)

Open SSH session **#1**. Do these in order.

**3.1 Fix the clock** — the car boots to 1970 (no RTC). A wrong clock breaks TF,
rosbag and SLAM. Set real current UTC time:
```bash
sudo date -u -s "2026-06-30 16:00:00"   # use the actual current UTC time
date
```

**3.2 Make the log dir** (all logs land here):
```bash
mkdir -p ~/rr_logs
```

**3.3 Bring up sensors + drive stack** (leave running in session #1):
```bash
~/t_stack.sh 2>&1 | tee ~/rr_logs/t_stack.log
```

**3.4 Health check B1–B2** — open SSH session **#2**:
```bash
~/rr/rr_healthcheck.sh
```
Do not continue until **B1 (sensors)** and **B2 (base TF)** are all PASS.
If not, see §7.

---

## 4. Re-run SLAM to build a NEW map

Skip this if you already have a good map in `~/rr_maps/`. Otherwise:

**4.1 Clear stale state.** A leftover SLAM/localization node from a previous run
keeps publishing an old `map→odom` and corrupts the new map. Kill any:
```bash
ros2 node list | grep -Ei 'slam|map_server|amcl'   # should be empty before mapping
pkill -f slam_toolbox ; pkill -f map_server ; pkill -f amcl   # if any were listed
```
Old saved maps in `~/rr_maps/` (`my_track*`, `live*`) are only overwritten if you
reuse the name — so save the new one under a **new name** (4.4).

**4.2 Start mapping** (session #2; `rr_slam.sh` = slam_toolbox async, real frames):
```bash
~/rr/rr_slam.sh 2>&1 | tee ~/rr_logs/slam.log
```

**4.3 Drive the course** slowly with the joystick (hold the deadman), a full
loop, hugging neither wall, until the outline closes. Watch it fill in — from a
3rd session snapshot the map and pull it to the PC:
```bash
# on the car:
~/rr/rr_snap.sh                 # writes ~/rr_maps/live.png
# on the PC (Git Bash):
scp roboracer@192.168.50.10:rr_maps/live.png .
```

**4.4 Save the new map** while `rr_slam.sh` is still running:
```bash
~/rr/rr_savemap.sh track2       # -> ~/rr_maps/track2.pgm + track2.yaml
cat ~/rr_maps/track2.yaml       # note resolution (0.05) and origin
```

**4.5 Stop mapping:** `Ctrl-C` the `rr_slam.sh` session. The map files remain.
**Important:** amcl (next part) and slam_toolbox must not run at the same time —
make sure `rr_slam.sh` is stopped before §5.

> Note the SLAM **origin**: slam_toolbox starts the map with the car at `(0,0,0)`.
> So if you place the car at the same spot before the autonomous run, the default
> initial pose `(0,0,0)` is already correct.

---

## 5. Drive autonomously from a start point to an end point

`~/t_stack.sh` from §3 must still be running. SLAM (§4) must be stopped.

**5.1 Place the car** at a known spot on the map. Easiest: the SLAM start spot
(then start pose = `0,0,0`). Start the recorder in a spare session if you want a
full replay later:
```bash
~/rr/rr_record.sh run1          # optional; Ctrl-C to stop after the run
```

**5.2 Launch Nav2 + the drive bridge** (session #2). Use your saved map:
```bash
source /opt/ros/humble/setup.bash && source ~/roboracer_ws/install/setup.bash
ros2 launch roboracer_estimation autonomous_real.launch.py \
    map:=/home/roboracer/rr_maps/track2.yaml 2>&1 | tee ~/rr_logs/nav2.log
```
> Every new SSH session needs `source /opt/ros/humble/setup.bash && source
> ~/roboracer_ws/install/setup.bash` before any `ros2` command. The `~/rr/*.sh`
> helper scripts already source it themselves.
This starts map_server, amcl, planner, controller, bt_navigator, behaviors,
velocity_smoother, the lifecycle managers, and `cmd_vel_to_ackermann`.

**5.3 Set the START pose** (so amcl knows where the car is). The config already
sets `(0,0,0)`; if the car is elsewhere, set it explicitly (session #3):
```bash
~/rr/rr_initpose.sh 0 0 0        # x y yaw  (meters, radians, map frame)
```
(Or, if you have RViz: use **2D Pose Estimate**.)

**5.4 Verify localization — health check B3–B5** (session #3):
```bash
~/rr/rr_healthcheck.sh
```
Require PASS on: `/map` has data, `/amcl` active, **`map→odom` TF present**, and
all nav servers active. The single most important line is `map→odom` — without
it the car has no idea where it is. If missing, re-run 5.3 (and see §7-B4).
Sanity-look: in RViz the laser scan should overlap the map walls.

**5.5 Send the END point (goal)** (session #3):
```bash
~/rr/rr_goal.sh 5.0 1.0 0        # x y yaw of the destination, map frame
```
(Or RViz **Nav2 Goal** / **2D Goal Pose**.) The planner makes a path, the
controller follows it, and the car drives — slowly (0.5 m/s cap).

**5.6 Watch it work — health check B6–B7** (run while the goal is active):
```bash
~/rr/rr_healthcheck.sh
```
Expect `/plan` present, `/cmd_vel` and `/drive` publishing, `/commands/motor/speed`
flowing. The car stops itself at the goal (xy tolerance 0.25 m).

**5.7 Stop** any time: grab the joystick (overrides instantly), or `Ctrl-C` the
Nav2 session (mux then zeroes `/drive` within 0.2 s).

---

## 6. The logging system (for debugging later)

Everything writes under **`~/rr_logs/`**:

| File | What |
|---|---|
| `t_stack.log` | sensor/drive stack (LiDAR, VESC, mux) |
| `slam.log` | SLAM mapping session |
| `nav2.log` | **all Nav2 nodes** — planner/controller/amcl/lifecycle errors land here |
| `healthcheck_<ts>.txt` | one report per health-check run, PASS/FAIL per bottleneck |
| `bag_run1_<ts>/` | full rosbag (scan, tf, amcl, plan, cmd_vel, drive, …) |

Pull them to the PC for inspection:
```bash
scp -r roboracer@192.168.50.10:rr_logs ./car_logs
```

Re-run `~/rr/rr_healthcheck.sh` at any stage — it is the fast way to find *which*
bottleneck (B1–B7) is broken before digging into `nav2.log`. To watch a specific
node live: `ros2 topic echo /rosout` or grep the log, e.g.
`grep -iE 'warn|error' ~/rr_logs/nav2.log`.

---

## 7. Troubleshooting by bottleneck

Find the failing `Bx` in the health check, then:

**B1 — no `/scan`:** LiDAR is on the wired net. Check `eno1` is up and the Hokuyo
at `192.168.0.10` is reachable; check `t_stack.log`. **no `/odom` / no VESC
symlink:** VESC USB unplugged or `t_stack` not up; replug, restart `~/t_stack.sh`.

**B2 — no `odom→base_link`:** `vesc_to_odom` not running → `t_stack` failed, check
`t_stack.log`. **no `base_link→laser`:** the static transform in bringup didn't
start; restart `t_stack`.

**B3 — `/map` empty or `/map_server` not active:** wrong `map:=` path or the file
isn't in `~/rr_maps/`; check `nav2.log` for "map yaml" errors. Confirm
`ls ~/rr_maps/track2.yaml`.

**B4 — no `map→odom` / amcl not localizing (most common):** amcl has no initial
pose, or the pose is wrong. Re-send `~/rr/rr_initpose.sh X Y YAW` with the car's
actual location, then push the car forward a meter by joystick so amcl converges.
Confirm `/scan` overlaps the map in RViz. If amcl is "activating" forever, check
`nav2.log` for lifecycle errors.

**B5 — a nav server not active:** `lifecycle_manager_navigation` failed to bring
one up — read `nav2.log` (usually a bad parameter in `nav2_params_real.yaml` or a
missing plugin). The failing node name is in the log.

**B6 — goal sent but no `/plan`:** planner couldn't find a path. Causes: goal is
in an occupied/unknown cell, start pose wrong (fix B4), or
`minimum_turning_radius` too large for a tight map — lower it toward `0.75` in
`nav2_params_real.yaml`, rebuild config (`colcon build --packages-select
roboracer_estimation`), relaunch. **`/plan` exists but no `/cmd_vel`:** controller
problem — check `nav2.log` (costmap/footprint). **`/cmd_vel` but no `/drive`:**
`cmd_vel_to_ackermann` not running (check `nav2.log`).

**B7 — `/drive` but car doesn't move:** mux not forwarding (joystick override
active? deadman?), or `/drive` stale (>0.2 s gaps → mux zeroes it). Confirm
`ros2 topic hz /drive` is steady. Check `/commands/motor/speed` is non-zero.

**Car drives but wanders off the path / oscillates:** localization drift (improve
the map / initial pose), or speed too high — keep 0.5 m/s for now.

---

## 8. After a clean slow lap: raising the speed

Only once the car tracks a path cleanly at 0.5 m/s. Edit
`roboracer_estimation/config/nav2_params_real.yaml`, raise in **small steps**
(e.g. 0.5 → 0.8 → 1.2 m/s), keeping below the ~2.0 m/s hardware cap:
- `controller_server.FollowPath.desired_linear_vel`
- `velocity_smoother.max_velocity[0]` (and `min_velocity[0]` for reverse)

Redeploy + rebuild config:
```bash
bash /c/Users/Student/Documents/Shiran-Hozuri/nav2-realcar-deploy/deploy.sh   # PC
# on car:
cd ~/roboracer_ws && colcon build --packages-select roboracer_estimation && source install/setup.bash
```

---

## 9. Phase 2 — using YOUR Pure Pursuit instead of Nav2's controller

Your `roboracer_control` Pure Pursuit is currently bypassed because it reads the
robot pose **directly from the odom message and does no TF transform** — so it
needs a pose already in the **map** frame, while the real car's `/odom` is in the
drifting `odom` frame. To switch to it later:

1. Add a tiny node that publishes `nav_msgs/Odometry` in the **map** frame by
   looking up TF `map→base_link` (amcl) + velocity from `/odom`.
2. Run Nav2 for the **plan only**; relay `/plan`→`/control/plan` with
   `path_relay_node` set to `use_forward_oval_route:=false` (its default oval is
   sim-only and must be off on the real map).
3. Launch `pure_pursuit_controller` with a real-car param file: plain frames,
   `wheelbase:=0.25`, `odom_topic` = the map-frame odometry from step 1,
   `enable_mpc_overtaking:=false`, low `speed`.

This is more moving parts than Nav2 end-to-end, so do it only after §5 works.

