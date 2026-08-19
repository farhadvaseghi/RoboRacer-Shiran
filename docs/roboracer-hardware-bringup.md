# RoboRacer — Sim → Hardware Bring-up

Running our simulation stack (`farhadvaseghi/RoboRacer-Shiran`, branch
`dynamic-overtaking+slam`) on the real car. Captured 2026-06-30.

Flow goal: **SLAM (scan the track) → save map → localization → control (drive)**.
Dynamic overtaking is intentionally out of scope for now.

## Status at a glance

| Step | State |
| --- | --- |
| Backup other group's work to PC | ✅ `car-backup-2026-06-30/` (custom 37 MB + autoware-src 817 MB) |
| Our repo on PC + car | ✅ PC: `RoboRacer-Shiran/`; car: `~/roboracer_ws/src/RoboRacer-Shiran` |
| Build `roboracer_estimation` + `roboracer_control` | ✅ on car (`~/roboracer_ws/install`) |
| LiDAR `/scan` | ✅ ~40 Hz (Hokuyo @ 192.168.0.10) |
| VESC `/odom` + `odom→base_link` TF | ✅ ~50 Hz **while a drive command flows** |
| slam_toolbox on real frames | ✅ params ready (`~/rr/slam_params_real.yaml`) |
| **Phase 1: SLAM mapping** | 🟢 **Ready to run** (needs you to drive) |
| Phase 2: Nav2 + Pure Pursuit driving | 🔴 Blocked — see "Blockers" |

## Blockers for the full control flow (Phase 2)

1. **No internet on the car** → cannot `apt install` the 3 missing Nav2
   packages (`nav2_bringup`, `nav2_smac_planner`, `nav2_controller`). The
   wired LiDAR Ethernet is the default route and has no uplink; the internal
   Wi-Fi currently has no internet either.
2. **ZED wrapper not installed.** `ekf_real.yaml` expects ZED visual odometry +
   IMU (`/zed/zed_node/odom`, `/imu/data`). The ZED 2i camera is physically
   connected, but the ZED ROS 2 wrapper / SDK is not installed (also needs
   internet, and is heavy). Until then, estimation must run on **VESC wheel
   odometry only** (or SLAM pose), not the ZED-based EKF as written.
3. **Sim-tuned config** must be adapted (frames, wheelbase, speeds) — see
   "Config adaptation".

## Fix the clock first (recommended)

The car's clock reads ~1970 (no internet → no NTP, likely dead RTC battery).
This makes rosbag/log timestamps useless and breaks RViz run from the laptop
(TF "extrapolation into the future"). Set it once per boot, in a terminal **on
the car**, replacing the time with the current UTC:

```bash
sudo date -u -s "2026-06-30 12:30:00"
```

(Approximate is fine — within a minute. Re-run after each reboot until the RTC
battery / NTP is fixed.)

## Phase 1 — SLAM mapping (validated, ready)

Helper scripts live in `~/rr/` on the car. Use separate terminals (SSH or the
car's own monitor). Source is handled inside each script.

**Terminal 1 — drivers** (LiDAR + VESC + joystick + mux; no motion):
```bash
~/t_stack.sh
```

**Terminal 2 — SLAM** (slam_toolbox async mapping on the real `/scan`):
```bash
~/rr/rr_slam.sh
```

**Terminal 3 — record a bag** (optional but recommended for debugging):
```bash
~/rr/rr_record.sh track1
```

**Terminal 4 — RViz** (optional, to watch the map build). Run on the car's
display, or on the laptop only **after** fixing the clock. Fixed frame `map`;
add displays `Map` (`/map`), `LaserScan` (`/scan`), `TF`.

**Drive the track:** with the gamepad, **hold the deadman (button 4 / LB)** and
use the sticks to drive **slowly** around the **full** loop at least once so
slam_toolbox can close the loop. Keep moving — `/odom` (and the `odom→base_link`
TF) only publishes while a command flows, so SLAM only ingests scans while the
car is being driven.

**Save the map** when it looks complete in RViz:
```bash
~/rr/rr_savemap.sh my_track     # -> ~/rr_maps/my_track.pgm + .yaml
```

Then copy the map into the repo for navigation use:
`cp ~/rr_maps/my_track.* ~/roboracer_ws/src/RoboRacer-Shiran/roboracer_estimation/maps/`

> Note: the repo's `slam_mapping.launch.py` is **sim-only** (it starts the
> f1tenth_gym simulator). On hardware we run `slam_toolbox` directly via
> `rr_slam.sh` against the real LiDAR instead.

## Logging & debugging

Working hardware + software at once — keep these habits:

- **Record every run:** `~/rr/rr_record.sh <name>` → `~/rr_logs/<name>_<ts>/`
  (a rosbag2 of `/scan /odom /tf /tf_static /map /drive /teleop /joy
  /odometry/filtered /diagnostics /rosout`). Replay later with
  `ros2 bag play <dir>` to debug offline without the car.
- **One-shot health check:** `~/rr/rr_doctor.sh` — date, VESC symlink, node
  list, `/scan` `/odom` `/drive` rates, and the `odom→base_link` TF.
- **Live introspection:** `ros2 node list`, `ros2 topic hz /scan`,
  `ros2 topic echo /drive --once`, `ros2 run rqt_graph rqt_graph`.
- **Pull logs to the PC** for analysis:
  `scp -r roboracer@192.168.50.10:~/rr_logs/<name>_<ts> ./logs/`
- **Gotcha:** `ros2 topic hz` reporting "not published yet" for `/odom` at idle
  is expected — drive (or send a zero-speed `/drive`) to make it publish.
- Keep `ROS_DOMAIN_ID` identical (default 0) across all terminals and the
  laptop, or nodes won't see each other.

## Phase 2 — Localization + driving (after unblocking)

Once the car has internet:

```bash
sudo apt update
sudo apt install -y ros-humble-nav2-bringup ros-humble-nav2-smac-planner \
    ros-humble-nav2-controller
```

Then the intended chain is: drivers → localization (slam_toolbox in
**localization** mode against the saved map, or `particle_filter`) →
`estimation.launch.py is_sim:=false` (EKF) → `navigation.launch.py` (Nav2 +
SMAC planner) → `roboracer_control controller.launch.py` (Pure Pursuit) →
`/drive`. Send a goal with RViz's **Nav2 Goal** tool.

**Estimation without the ZED wrapper:** run the EKF on VESC odom only. The
adaptive node expects `/vesc/odom`, but the real stack publishes `/odom` — remap
or point the EKF `odom1` at `/odom`, and drop the ZED `imu0`/`odom0` inputs
until the ZED wrapper is installed. Simplest first pass: skip the EKF entirely
and feed the controller `/odom` directly.

## Config adaptation checklist (sim → real)

In `roboracer_control/config/controller_params.yaml`:

- `base_frame: ego_racecar/base_link` → `base_link`
- `odom_frame: ego_racecar/odom` → `odom`
- `odom_topic: /odometry/filtered` → `/odom` (until EKF runs on hardware)
- `wheelbase: 0.3302` → **`0.25`** (real car, from `vesc.yaml`)
- `enable_mpc_overtaking: true` → **`false`** (overtaking out of scope)
- **Cap speeds for the first floor run:** `speed`, `straight_speed`,
  `max_speed_command`, `turn_speed` → ~`0.8–1.0 m/s`. Defaults (3.7+ m/s) are
  unsafe indoors.
- `opponent_odom_topic` / `/opp_*` — not present on hardware; ignored when MPC
  is off.

All `/drive` output already matches the car's `ackermann_mux` "navigation"
input (priority 10); the joystick (`/teleop`, priority 100) overrides it at any
time — that is your safety stop.

