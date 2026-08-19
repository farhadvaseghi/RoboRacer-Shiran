# Session Report — Custom Pure-Pursuit Controller Live + One-Shot Bring-Up (2026-07-16)

**Date**: 2026-07-16
**Robot**: RoboRacer (F1TENTH on Jetson Orin Nano, ROS 2 Humble, `ROS_DOMAIN_ID=7`)
**Access**: `ssh roboracer@192.168.50.10` (Wi-Fi `roboracer`); car is offline, the dev box bridges internet ↔ LAN
**Goal for the day**: replace Nav2's controller with the team's own pure-pursuit controller and drive the car with it; consolidate the whole stack into a single idempotent bring-up script.

## Headline outcome

**The team's custom C++ `pure_pursuit_controller` drove the car autonomously for the first time on hardware.** With Nav2 acting only as the planner, the custom controller followed the plan to a `/goal_pose` at **0.5 m/s with under 2 cm cross-track error and self-stopped at the goal**. This is the first custom-controller run on the car (previous autonomous runs used Nav2's built-in Regulated Pure Pursuit).

Along the way the real cause of the car clipping walls was found and fixed — it was a **miscentered steering servo**, corrected by physical VESC calibration.

## Deploying `roboracer_control` to the car

`roboracer_control` was **missing from the car** — only `roboracer_camera` and `roboracer_estimation` had been deployed. Pulled it from GitHub, `farhadvaseghi/RoboRacer-Shiran`, **Hardware** branch, using the dev box as the bridge:

```
# on the dev box (has internet + LAN):
git clone --depth 1 -b Hardware <repo>
scp -r <that repo>/roboracer_control  roboracer@192.168.50.10:~/roboracer_ws/src/
# on the car:
colcon build --packages-select roboracer_control
```

- The controller is C++ (`pure_pursuit_controller`); its dependencies are all stock ROS (rclcpp, tf2, ackermann_msgs, nav_msgs, geometry_msgs, visualization_msgs), so it built cleanly.
- The repo on the car is **not** a git repo (deployed as plain files), which is why only the one package was copied in.
- `roboracer_control` also exists on branches Control-V2, Planning-V2, camera-costmap, and dynamic-overtaking.

## Control pipeline (custom controller)

```
nav2 planner  →  /plan (VOLATILE)
              →  plan_qos_relay      (QoS bridge: /plan → /control/plan, RELIABLE + TRANSIENT_LOCAL)
              →  pure_pursuit_controller
              →  /drive
              →  ackermann_mux
              →  VESC
```

Pose to the controller:

```
slam (map→base_link)  +  /odom
              →  map_odom_relay  (roboracer_camera)
              →  /odometry/map   (map-frame odom; the controller reads pose straight from the message, no TF)
```

- **`plan_qos_relay`** is required because the controller's subscription demands RELIABLE + TRANSIENT_LOCAL, while Nav2 publishes `/plan` as VOLATILE.
- **Do not run `cmd_vel_to_ackermann`.** Leaving it out isolates Nav2's Regulated Pure Pursuit so it cannot fight the custom controller for `/drive`. Nav2's FollowPath still runs, but it is harmless because nothing bridges its `/cmd_vel` to `/drive`.

### Controller params — `~/rr/controller_params_real.yaml`

Real-robot frames (`base_link` / `odom`, **not** the sim's `ego_racecar/*`):

- `odom` topic: `/odometry/map`
- `path` topic: `/control/plan`
- `drive` topic: `/drive`
- `wheelbase`: 0.25
- `max_steer`: 0.32
- **speeds capped at 0.5 m/s** (the branch defaults were 2.8–3.8 m/s racing speeds)

## VESC physical calibration — the real wall-clipping fix

Earlier sessions attributed wall-clipping to the planner routing paths too close to walls. A ruler test proved otherwise. Commanded a 2 m drive at steering = 0:

- The car went **2.2 m forward and ~30 cm to the LEFT** — a constant left bias from a miscentered steering servo, curving the car into walls. This, not path-hugging, was the dominant cause.
- Odom read **2.005 m for a real 2.22 m** — odom under-reported distance by ~10 %.

Fixes in `f1tenth_stack/config/vesc.yaml`:

- `steering_angle_to_servo_offset`: 0.4715 → **0.499** (centers the servo; the gain is negative, −0.8825, so a higher offset steers more right)
- `speed_to_erpm_gain`: 4532 → **4093**  (= 4532 × 2.005 / 2.22)

Re-tested: 2 m straight, no drift.

- `rr_bringup.sh` now **enforces both values** (seds the install-space and source `vesc.yaml`) before starting the base stack, so a rebuild or revert cannot silently lose the calibration.
- Live `ros2 param set` cannot apply these (the daemon reports "Node not found" — see DDS note below); they need a base-stack restart to load, which resets the odom origin → give a Foxglove **2D Pose Estimate** afterward to re-localize slam.
- Repeatable procedure (`~/rr/cal_drive.py`): drive until odom reads 2 m, tape-measure the real forward distance and the lateral drift, back out the two gains.

## Map despeckle → clean `/map`

`corridor_clean` had ~404 isolated occupied specks (mapping noise) that the costmap inflated into phantom obstacles, producing weaving paths and lethal-start failures.

- **`~/rr/rr_despeckle.py`** (numpy + scipy connected-components): drops blobs ≤ 20 px, keeps the walls → `~/rr_maps/corridor_despeck.{pgm,yaml}` (439 → 35 components).
- Served by a dedicated `map_server` (node `map_clean_server`, `topic_name:=/map`) **on `/map`**, so Foxglove shows the clean map by default with no manual toggle. SLAM's noisy grid is remapped **off** `/map` to `/map_posegraph` (slam still provides the `map→odom` TF).
- `map_clean_server` **must** be activated by a `nav2_lifecycle_manager` (`map_clean_lifecycle`, autostart). A CLI `ros2 lifecycle set` fails for the remapped node on a fresh-boot daemon.
- Global costmap `static_layer.map_topic` = `/map`.

## Costmap / Nav2 param changes (persisted, `.bak_*` backups made)

`nav2_params_real.yaml`:

- **`robot_radius`: 0.22 → 0.10** (both costmaps). The car parks ~0.14–0.18 m from a wall — inside the old 0.22 inscribed radius — which put the start cell in lethal space and produced a "starting point in lethal space" loop that clearing the costmap could not fix (the wall is in the static layer). 0.10 keeps the start valid unless the car is literally < 0.10 m off a wall. Trade-off: lower safety margin; to restore 0.22 the car must sit ≥ 0.25 m off walls.
- **Inflation radius: local 0.12, global 0.40** (global raised this session). With global inflation at 0.12 the planner routed paths hugging the walls, and the car (~0.13 m half-width) plus pure-pursuit tracking error clipped them. `inflation_radius` controls the cost **gradient** that centers the path and is separate from `robot_radius` (the lethal/inscribed radius), so global 0.40 pushes paths ~0.4 m off walls **without** breaking start validity. Key separable-knobs insight: robot_radius = where the start is legal, inflation_radius = how far paths are pushed off walls.
- **Global costmap plugins = `["static_layer", "inflation_layer"]` — `obstacle_layer` removed.** The fixed (non-rolling) global obstacle_layer accumulated phantom LiDAR marks (reflections, glass, people) forever, so the planner kept weaving. Consequence: the planner now ignores unmapped obstacles and plans purely on the static map; the LB joystick e-stop is the only safety for anything not on the map. The **local** costmap keeps its obstacle_layer (rolling and self-clearing; unused by the custom controller).

> **GOTCHA**: the `/global_costmap/costmap` topic is scaled **0–100** (99 = inscribed = lethal-for-start, 100 = wall), not 0–255. A 0.10 robot_radius drops the start cell below inscribed so it can plan close to walls everywhere.

## `~/rr/rr_bringup.sh` — one idempotent script for the whole stack

A single script (domain 7) that brings up everything, in order, each piece as its own detached process group, with no `pkill -f`:

1. Clock fix (`RR_UTC="YYYY-MM-DD HH:MM:SS"` env, or prompts)
2. Shared-memory clear (cold start only)
3. Base stack (with the `vesc.yaml` calibration enforced first)
4. `rr_fix_joy.sh` (deadman / e-stop config)
5. Foxglove bridge on `:8765`
6. Despeckle + serve the clean map on `/map`
7. slam_toolbox localization
8. Nav2 planner
9. `plan_qos_relay`
10. `map_odom_relay`
11. `pure_pursuit_controller`
12. Gated auto-keeper (`rr_autokeep.py`)
13. Costmap reset on `/initialpose` (`rr_costmap_reset.py`)

Run: `RR_UTC="YYYY-MM-DD HH:MM:SS" ~/rr/rr_bringup.sh`

Status: **validated piece-by-piece and the two new nodes ran live, but the script has not yet been run from a full cold boot.** All helpers live in `~/rr/`: `rr_autokeep.py`, `rr_costmap_reset.py`, `rr_despeckle.py`, `plan_qos_relay.py`.

- **Gamepad handling**: the script now waits up to 30 s for `/dev/input/js0` before starting the base stack, and restarts `joy_node` if it holds no `/dev/input/event*` device — this self-heals a controller connected late or on a re-run. Just turn the gamepad on when prompted; no full restart needed. (`joy_node` uses the evdev `/dev/input/event*` device, not `js*`; `js0` presence just means the controller is connected.)
- Only manual pre-bringup step the script cannot do: fix the clock (needs sudo).

### `rr_autokeep.py` — gated zero-speed keeper

The VESC only emits `/odom` while a drive command is flowing. The keeper publishes a zero-speed `/drive` at rest to keep odom (and the TF chain) alive, **but auto-pauses for 0.6 s whenever it sees a nonzero `/drive`** (i.e. the controller is driving). This removes the old manual "stop the keeper before sending a goal" handoff — the keeper and controller no longer fight over `/drive`.

### `rr_costmap_reset.py` — full reset on every `/initialpose`

Triggered by the Foxglove **2D Pose Estimate** tool. On each `/initialpose` it:

- clears the global and local costmaps,
- cancels the active `navigate_to_pose` goal (empty `action_msgs/CancelGoal` = cancel all),
- publishes an **empty Path** to `/plan` and `/control/plan`, which stops the controller (it sets `have_path_ = false` on an empty path). The `/control/plan` publisher must be RELIABLE + TRANSIENT_LOCAL to overwrite the latched path.

slam-loc also relocalizes on `/initialpose`, so **one 2D Pose Estimate = relocalize + full clean slate** (verified this session).

## Opponent detector — added then removed

`opponent_detector` was added to the bring-up and then **removed** the same day. On this map its LiDAR classifier emits **phantom opponents** (e.g. a bogus "opponent" at map (18.8, 37.7), 37 m sideways). When a phantom lands within `obstacle_slowdown_distance` (2.2 m) ahead, the controller slows and "follows" it and **stops ~2 m short of the goal, intermittently**. In point-to-point tests with no real opponent it only false-positives, so it is not run. The controller reaches goals cleanly without it (MPC stays inactive, `have_opponent_ = false`).

The custom controller has no wall input — it only takes an opponent point on `/opp_racecar/odom`, and the opponent_detector classifies walls out. Wall avoidance therefore comes from (1) the planner via the static map + inflation [done] and, optionally, (2) a wall-AEB pass-through `/drive` filter on `/scan` (mirroring how `emergency_brake` filters on `/perception/persons`). **Re-add and tune the classifier only when actually racing a real opponent.**

## Notes / gotchas carried forward

- **1970 clock**: the RTC is dead and the car is offline, so the clock resets to 1970 at every boot. Fix it (sudo, UTC) **first**, only at cold start — a `date -s` jump while the base stack is running breaks its TF.
- **DDS on this car**: `ros2 daemon` and `ros2 topic info -v` hang, and `ros2 param set` reports "Node not found", because the 1970 clock breaks service/discovery even though pub/sub and TF flow fine. Verify with `tf2_echo` and `ros2 topic echo --once` (direct), not the daemon graph. Tune by editing YAML and restarting the relevant group, not `param set`.
- **VESC "out-of-sync / invalid end-of-frame"** after a reboot clears with a clean base-stack restart.
- **Foxglove goal delivery**: the earlier "goal published but no plan/motion" was never `bt_navigator` failing to receive — it does get the Foxglove `/goal_pose` (QoS compatible). The real blockers were the lethal-start loop and the keeper fighting `/drive`. `ros2 topic pub --once /goal_pose` drops the message (the publisher dies before delivery); use Foxglove or a persistent publisher.

## Status summary

| Item | State |
|---|---|
| Custom `pure_pursuit_controller` on hardware | **Drove to goal**, 0.5 m/s, < 2 cm cross-track, self-stopped |
| VESC steering/speed calibration | Applied, enforced by `rr_bringup.sh` |
| Despeckled clean map on `/map` | Working, lifecycle-managed |
| Costmap params (robot_radius 0.10, global inflation 0.40, no global obstacle_layer) | Applied, backups saved |
| `rr_bringup.sh` full cold-boot run | **Not yet done** — validated piecewise only |
| `opponent_detector` | Removed for point-to-point; re-add + tune when racing |
