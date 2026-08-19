# Session Report — Reboot Recovery + First Autonomous Run (2026-07-14)

**Date**: 2026-07-14 (afternoon session; follows the morning problem-analysis report `session_report_2026-07-14.md`)
**Robot**: RoboRacer (F1TENTH on Jetson Orin Nano, ROS 2 Humble, `ROS_DOMAIN_ID=7`)
**Access**: `ssh roboracer@192.168.50.10` (Wi-Fi `roboracer`)
**Goal for the day**: get the car to drive autonomously from a fixed start point (a cross marked on the ground).

## Headline outcome

**First autonomous motion from the cross was achieved.** With Nav2 driving on a saved SLAM map, the car planned and drove ~2 m straight forward on a terminal-sent goal, and the joystick **LB e-stop successfully interrupted it**. This is the first confirmed autonomous run on the hardware, and the safety override works.

The run was functional but **not clean** — the planner first failed with "starting point in lethal space" and ran a spin recovery before driving (root cause found and a fix prepared but not yet applied — see below).

## Starting state

Mid-way through the earlier session the car **dropped off the network entirely** (both Wi-Fi interfaces `.10` and `.147` unreachable, ARP unresolved) while the SLAM + Nav2 + costmap stack was running — most likely a reboot/crash, possibly a power/undervoltage brownout under compute load. The user power-checked and confirmed the car **rebooted**.

On reconnect: uptime ~39 min, **clock back at 1970** (dead RTC, no NTP), nothing ROS running, but all `~/rr/` scripts and the good SLAM map `corridor_clean.*` (cross = origin 0,0,0) intact on disk.

## What we did — reboot recovery (in order)

1. **Fixed the clock first, before launching anything.** Set the car's UTC from the laptop's UTC → `2026-07-14 12:36 CEST`. Order matters: `rr_up.sh` warns never to `date -s` while the base stack is running (the time jump breaks its TF). Clock was fixed at a true cold start, so this was clean.

2. **Wrote `~/rr/rr_up_slam.sh`** — a variant of `rr_up.sh` that swaps AMCL for slam_toolbox **localization** on the saved map. This was deliberate: we previously pivoted away from AMCL because its initial-pose init was fragile on a map whose origin didn't match the cross. `corridor_clean` now has cross = origin (0,0,0), and `localize_slam_real.yaml` has `map_start_at_dock: true`, so the car comes up localized at the origin with no manual initial pose. The script brings up (each in its own detached process group, idempotent, no `pkill -f`): base stack → Foxglove bridge (`ws://…:8765`) → slam_toolbox localization.

3. **Diagnosed and fixed slam dropping every scan.** slam_toolbox loaded the map but logged "Message Filter dropping message … queue is full" for every scan, and there was **no `/odom`, no `odom→base_link` TF, no `map→odom`**. Root cause: the known VESC quirk — **the VESC only emits `/odom` + `odom→base_link` TF while a drive command is flowing** (even zero-speed). Parked on the cross with no command, the TF chain is broken and slam can't transform scans.
   - Fix: a **zero-speed `/drive` keepalive at 20 Hz** (`~/rr/rr_keep.sh` to start, `~/rr/rr_keep_stop.sh` to stop). After starting it: `/odom` flows, `odom→base_link` = identity (car still), `map→odom` ≈ `[0.20, 0.25, ~2°]` (slam localizing near origin), and scan drops fell from *all* to **~1 in 5 s** (~200 scans). Healthy.
   - **Important handoff rule discovered**: the keeper publishes to `/drive` (mux priority 10), the same input as Nav2's `cmd_vel_to_ackermann`. If both run, they fight (0/nav/0 stutter). So the keeper must be **stopped before sending a Nav2 goal**; during active navigation Nav2's own commands keep odom alive; restart the keeper at rest afterward.

4. **Re-applied the joystick deadman fix** (`rr_fix_joy.sh`). The base stack comes up with the wrong default joy config; the fix maps deadman to **button 4 (LB)** and keeps `/teleop` silent unless LB is held. This is also the e-stop: grab stick + hold LB → teleop (priority 100) overrides Nav2.

5. **Verified the tuned Nav2 params survived the reboot** (`nav2_params_real.yaml`): `desired_linear_vel: 0.3`, `use_collision_detection: false`, `motion_model_for_search: REEDS_SHEPP`, `minimum_turning_radius: 0.40`, `inflation_radius: 0.25`. All good (these had reverted to the 07-07 known-bad DUBIN/0.90 values earlier in the day and were re-applied; they persisted across the reboot).

6. **Brought up Nav2** (`rr_nav.sh` → `navigation_launch.py` + `cmd_vel_to_ackermann`). All lifecycle nodes reached **active**: planner_server, controller_server, bt_navigator, behavior_server.

## The "black circles" on the map — root cause + fix prepared

The user reported extra black circles on the map in Foxglove that "should not exist." Investigated and **fully explained**:

- The costmaps use **only `/scan` (LaserScan)** as their obstacle source — no camera/pointcloud/depth injecting phantom obstacles, and no such nodes running.
- Analysis of the saved map (`/tmp/mapblobs.py`, numpy+scipy connected-components): the map has **725 occupied components, but only 29 are actual walls** (≥61 px). The other **~688 are tiny isolated specks** (610 of them just 1–5 px) — mapping noise from reflections/glass/people during the mapping drive.
- The Nav2 **inflation layer (0.25 m)** blows each speck into a ~0.5 m black circle in Foxglove. That is exactly what the user saw.

**Fix prepared (not yet applied):** produced a despeckled map `~/rr_maps/corridor_despeck.pgm` + `.yaml` (`/tmp/despeckle.py`): removed every isolated occupied blob ≤20 px (set to free), **kept all walls** → **725 → 53 components**. Non-destructive (new file; `corridor_clean` untouched).

To actually get the clean map into planning without disturbing SLAM localization, the proposed (unapplied) plan was: run a `map_server` publishing `corridor_despeck` on a separate topic `/map_clean`, point only the **global costmap's static layer** at `/map_clean` (slam_toolbox keeps publishing `/map` + `map→odom`), and restart just Nav2. The car does not need to move for this.

## First autonomous run

- Confirmed the terminal goal path is fully wired: `rr_goal.sh X Y YAW` → `/goal_pose` (PoseStamped, map frame, correct quaternion) → `bt_navigator` (subscribed). This established that the **earlier "no motion" on goals (07-07 and earlier today) was Foxglove never publishing the goal** (the user's Foxglove lacks the Publish tool), **not** a stack problem.
- Costmap sanity check before moving (`/tmp/costcheck.py`): path straight ahead clear (cost 0 at 0.5/1.0/1.5 m), **zero lethal cells** in the front box; the car's own cell read cost 99 (inflation, not lethal).
- With the user positioned and holding the LB e-stop, we: stopped the keeper → sent `rr_goal.sh 1.5 0 0`.
- **Result: the car drove ~2 m straight forward autonomously**, and the user interrupted it with the LB e-stop. Goal cancel afterward reported no active goal → the goal had completed. Keeper restarted to hold odom at rest.

**What the nav log revealed about the run:** the planner repeatedly failed with `GridBased: failed to create plan … Starting point in lethal space! Cannot create feasible plan`, the behavior tree ran a **spin recovery** and cleared the costmaps, and only then did a plan succeed and the controller drive forward. Cause: the map specks inflate into lethal cells overlapping the car's footprint at the start, so the *first* planning attempt at every goal fails. The despeckled map (above) is the fix.

## User's Foxglove observations

The user checked the map in Foxglove directly and reported:
- The map showed **deflections** (it looked distorted / off).
- Foxglove appeared to be **creating a new map on top of the existing one**, which made the whole thing look misaligned.
- **Despite this, the car still managed to drive 2 m forward autonomously.**

This is consistent with slam_toolbox localization drift / a scan-match offset relative to the saved graph (an apparent "second map" laid over the first), and/or the periodic `/map` republish rendering on top of the loaded map. It did not prevent the run, but it needs follow-up (see below). The user's Foxglove also does **not** show a `/tf` topic (transforms are consumed automatically — set the 3D panel frame to `map`) and has **no Publish tool** (so goals were driven from the terminal instead).

## What we missed / did not do (pending for next session)

1. **Apply the despeckled map to Nav2.** The clean map is saved and the plan is written, but it was **not applied** (session ended first). Until then, every goal will still hit "starting point in lethal space" → spin recovery before driving. This is the top next-session task and is quick (map_server on `/map_clean` + retarget the global costmap static layer + Nav2 restart; car stays put).

2. **Investigate the Foxglove map deflection / "new map over old".** Determine whether slam_toolbox localization is drifting (scan-match offset vs the saved graph) or it's just a `/map` republish rendering artifact. If it's real drift, localization quality needs attention before faster/longer runs. Check `map→odom` stability over time and whether localization is adding to the map (it shouldn't in localization mode).

3. **Fix Foxglove usability.** The user's reinstalled Foxglove lacks the Publish tool and doesn't display `/tf`. Either configure the 3D panel (frame = `map`, add a Publish/PoseStamped tool on `/goal_pose`) or standardize on **driving goals from the terminal** (`rr_goal.sh`) — which proved reliable and bypasses the missing tool.

4. **Confirm the reboot root cause.** The car dropped off the net and rebooted under SLAM+Nav2 load — likely a power/undervoltage brownout, unconfirmed. No battery/voltage monitoring set up. The recurring dual-NIC Wi-Fi flakiness (`.10` vs `.147`) is also unresolved.

5. **Longer / turning / full-loop autonomy and speed.** Only a single short straight run was done. No turns, no return-to-origin loop, no speed increase above 0.3 m/s.

6. **Automate the keeper handoff.** Stopping the keeper before a goal and restarting it after is currently manual. Could be scripted into the goal-send flow, or replaced by making the VESC publish odom continuously.

## Current car state (as left)

- **Running:** base stack (LiDAR/VESC/joystick/mux), Foxglove bridge on `8765`, slam_toolbox **localization** on `corridor_clean`, zero-drive **keeper** (holding `/odom` at rest), Nav2 (all servers active) + `cmd_vel_to_ackermann`.
- **No active Nav2 goal.** Car is ~1.5–2 m forward of the cross (it did not return).
- **Clock:** correct (`2026-07-14` CEST).
- **Map in use by Nav2:** still slam's `/map` (with specks). The despeckled map is saved but **not wired in**.
- Releasing LB is safe (no pending motion; keeper holds it stopped).

## Files / scripts created or changed this session

On the car:
- `~/rr/rr_up_slam.sh` — base + Foxglove + slam_toolbox localization on `corridor_clean` (new).
- `~/rr/rr_keep.sh` / `~/rr/rr_keep_stop.sh` — zero-drive odom keepalive start/stop (new; file-based so `pgrep -f 'topic pub /drive'` can't self-match).
- `~/rr_maps/corridor_despeck.pgm` + `.yaml` — despeckled map, 53 vs 725 occupied components (new; `corridor_clean` untouched).
- `/tmp/mapblobs.py`, `/tmp/despeckle.py`, `/tmp/costcheck.py` — analysis scripts (numpy/scipy + rclpy).

## Gotchas / lessons (this session)

- **VESC odom only flows while a drive command is published.** At rest, no `/odom`, no `odom→base_link` TF, and slam localization silently drops every scan ("queue is full"). The zero-drive keeper is the fix — but it must be **stopped before a Nav2 goal** (same `/drive` topic, priority 10) or it fights the controller.
- **Fix the clock only at a true cold boot, before launching the base stack** — a `date -s` jump while the stack runs breaks TF.
- **"Starting point in lethal space" = costmap specks, not a planner bug.** Map noise inflated into lethal cells at the robot footprint. Clean the map, don't touch the planner.
- **`pgrep -f 'topic pub /drive'` self-matches** the running SSH command line — run such checks from a script file so the pattern isn't in the shell's own cmdline.
- **Foxglove `/tf` isn't a toggleable topic**, and if the Publish tool is missing, terminal `rr_goal.sh` is the reliable way to send goals.
- **slam_toolbox localization publishes `/map` from the posegraph**, so editing the `.pgm` alone does nothing for planning — the costmap must be pointed at a separately-served clean map.

---

**Session status:** First autonomous run from the cross achieved; e-stop verified. Stack left up and healthy. Main open item: wire in the despeckled map to remove the lethal-start/spin recovery, then investigate the Foxglove map deflection before longer runs.
