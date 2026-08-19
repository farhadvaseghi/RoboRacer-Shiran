# Session Report — Hardware Resume + Problem Identification (2026-07-14)

**Date**: 2026-07-14  
**Robot**: RoboRacer (F1TENTH on Jetson Orin Nano, ROS 2 Humble)  
**Access**: `ssh roboracer@192.168.50.10` (Wi-Fi `roboracer`)  
**Context**: User absent for the 2026-07-07 session. This report reviews that session's report + prior bring-up / architecture / guide docs, then identifies probable problems before starting fresh work. New session begins here.

## Quick recap of missed session (2026-07-07)

Goal evolved to: live SLAM (current pose = map origin 0,0,0), manual drive-out a short distance, then command Nav2 goal back to (0,0,0) for out-and-back validation.

**Stack brought up (separate terminals / detached):**
- `~/t_stack.sh` (f1tenth_stack bringup_launch.py): LiDAR `/scan`, VESC `/odom` + odom→base_link, joy + ackermann_mux, static base_link→laser.
- Foxglove bridge (desktop app required for ws://).
- `ros2 run slam_toolbox async_slam_toolbox_node` with real-car params (`~/rr/slam_params_real.yaml`: odom/base_link/scan, not ego_racecar names).
- `ros2 launch nav2_bringup navigation_launch.py` (use_composition:=False) + cmd_vel_to_ackermann (/cmd_vel → /drive, prio 10).

**Fixes applied during 07-07:**
- Real SLAM params (sim launch files are unusable on hardware).
- Avoided amcl + map_server collision with live slam_toolbox by using slam only.
- `use_composition:=False` to prevent hang.
- Cleared stale `/dev/shm/fastrtps_*`.
- Killed duplicate t_stack (VESC contention from second login).
- Fixed clock jump mid-run (SLAM drops scans on time discontinuity).
- Kill discipline (never `pkill -f` on strings that appear in own SSH cmd).

**Result:**
- SLAM tracked drive-out to (3.13, 0).
- Goal (0,0,0) accepted.
- **No path generated**: "GridBased: failed to create plan".
- Root cause: SMAC-Hybrid `motion_model_for_search: DUBIN` (forward-only). Minimum turning radius 0.90 m + straight-out pose cannot produce a forward-only return in the space. 
- Pending at end of session: set `REEDS_SHEPP` (allows reverse) via param set or edit of the nav2 params, re-test goal.

ROS_DOMAIN_ID=7 was used for the stack in that session (domain 0 was empty).

Safety model same: joystick prio 100 overrides Nav2 prio 10.

## Documented target flows (guide + bringup + CLAUDE)

Two related but not identical flows exist in the notes:

1. **Prepared "guide" flow** (nav2-realcar-deploy + guide.md + healthcheck):
   - `~/t_stack.sh`
   - `autonomous_real.launch.py` (map_server + amcl + planner_server + controller_server (Reg. Pure Pursuit) + bt_navigator + ... + cmd_vel_to_ackermann)
   - Saved map (e.g. track2 or my_track) + initial pose
   - Uses amcl localization against static map.
   - Healthcheck (B1–B7) is written for exactly these node names.

2. **07-07 experimental flow**:
   - Live `slam_toolbox` (map built on the fly, car start = origin).
   - Standalone `nav2_bringup navigation_launch.py`.
   - No amcl / map_server (slam owns /map + map→odom).
   - Simpler for "return to where I started" tests.

3. **Longer-term intended (CLAUDE + roboracer packages)**:
   - Nav2 for global plan only (`/plan`).
   - `path_relay_node` → `/control/plan`
   - `roboracer_control` Pure Pursuit (C++) for tracking → `/drive`
   - Estimation (adaptive_cov + EKF or direct /odom).
   - **Phase 2 / not yet ready on hardware**: Pure Pursuit currently reads odom message directly (no TF), so needs map-frame odometry bridge. `path_relay` defaults to sim oval route.

Current practical path on hardware is **Nav2 (planner + its controller) + cmd_vel_to_ackermann**, per the prepared guide and what was running in 07-07.

## Probable problems / risks (identified from docs cross-reference)

### 1. State left from 07-07 session (unknown because user absent)
- Processes may still be running (t_stack, slam_toolbox, nav2 servers, foxglove_bridge, cmd_vel_to_ackermann).
- Stale FastRTPS shared memory segments (`/dev/shm/fastrtps_*` and semaphores) — cause silent hangs on next bringup.
- SLAM or amcl may be publishing old `map→odom` / `/map`.
- Clock may be in 1970 or have jumped.
- ROS_DOMAIN_ID may be left at 7 in some shells/scripts or not.
- VESC driver may be locked or in bad state if not cleanly shut down.
- `/odom` only appears while commands are flowing (drive or teleop active).

**Impact**: High. Almost every previous bringup hit one of these.

### 2. Clock / time problems (recurring)
- Boots to ~1970 (no RTC battery + no NTP on isolated LAN).
- Breaks TF extrapolation ("into the future"), rosbags, SLAM scan ingestion, RViz on laptop, apt Release validation.
- Even after manual set, a jump can occur.

**Fix always first**: `sudo date -u -s "2026-07-14 HH:MM:SS"` (UTC) in a car terminal. Re-apply after reboots.

### 3. Network / connectivity gotchas
- Must join `roboracer` Wi-Fi on laptop (192.168.50.x). Ping to car is filtered — test with SSH / TCP 22.
- Car is multi-homed (roboracer WiFi control, eno1 wired LiDAR net 192.168.0.x, internal upstream WiFi). Default route can be wrong.
- LiDAR requires eno1 + "Hokoyu" NM profile to reach 192.168.0.10.
- Foxglove: browser web UI often fails on ws:// (mixed content); use desktop Foxglove app.
- Multiple logins / second terminals on car have previously started duplicate stacks.

### 4. ROS domain / visibility
- 07-07 explicitly used domain 7. Older docs and scripts assume default 0.
- If `ROS_DOMAIN_ID` not consistent across all terminals + laptop tools, nodes are invisible.
- `ros2 topic list` / `node list` on wrong domain shows nothing.

### 5. Localization / map / SLAM vs AMCL conflicts
- slam_toolbox (mapping or localization mode) and (map_server + amcl) both want to publish `/map` and `map→odom` TF → collision or wrong pose.
- Last session deliberately avoided amcl to use live SLAM origin.
- If using guide flow: amcl needs accurate initial pose (rr_initpose.sh or RViz). Wrong pose = no `map→odom`, planner has no idea where it is.
- Map origin from SLAM (car start = 0,0) vs saved map origin in .yaml must match physical placement.
- Drive required to make `/odom` + odom→base_link appear.

### 6. Planner / path issues (directly from 07-07)
- `motion_model_for_search: DUBINS` (forward only) in SMAC/GridBased → cannot reverse or make tight return loops.
- `minimum_turning_radius: 0.90` may be too conservative for the physical car (~0.75 m from wheelbase 0.25 + max steer).
- Goal in occupied/unknown cell, or start pose bad → "no valid path".
- Even with REEDS_SHEPP, tight spaces or bad costmaps can still fail.

**Action from last session never confirmed completed.**

### 7. Config / param drift (sim → real)
- Frames: many places still have `ego_racecar/odom` or `ego_racecar/base_link`.
- Wheelbase: 0.3302 (sim) vs real 0.25.
- odom source: `/odometry/filtered` (EKF) vs real `/odom`.
- ZED 2i camera is physically present but wrapper/SDK **never installed** (no internet at the time, heavy). Any EKF config expecting `/zed/...` or `/imu/data` will fail or produce no data.
- Speed limits in nav2_params: must stay low (0.5 m/s first runs). Hardware ~2 m/s max.
- `path_relay_node` `use_forward_oval_route: true` by default (ignores real map).

### 8. Launch / integration mismatches
- Many repo launch files (`slam.launch.py`, `slam_mapping.launch.py`, sim ones) start f1tenth_gym simulator — unusable on hardware.
- `autonomous_real.launch.py` (the one the healthcheck and guide target) may or may not be the one used in 07-07.
- Standalone `navigation_launch.py` (what 07-07 used) does not start map_server/amcl/cmd_vel_to_ackermann in the same way — healthcheck will report FAIL on B3/B4.
- Pure Pursuit (our roboracer_control) + path_relay not wired into the current Nav2 flow on hardware yet (Phase 2 pending; requires map-frame odom bridge).
- `cmd_vel_to_ackermann` must be present for /cmd_vel → /drive.

### 9. Process / kill / resource issues
- VESC USB contention (only one driver).
- `pkill -f "string"` can match the SSH command line itself and kill the shell.
- Lifecycle nodes stay in "activating" or "inactive" if params bad or dependencies missing.
- Stale `/dev/shm` after any crash/kill.

### 10. Observability & tooling
- Healthcheck (`~/rr/rr_healthcheck.sh`) is excellent but assumes the amcl-based launch. It will lie or show FAILs if using pure slam + nav2_bringup.
- No `/odom` at idle is expected (not a bug).
- Laptop RViz requires close clock or TF fails.
- rosbags are the only reliable post-run debug (pull with scp).

### 11. PC-side / deploy problems (this Windows folder)
- `RoboRacer-Shiran/` clone is **absent** from `C:\Users\Student\Documents\Shiran-Hozuri\` (OneDrive dehydration happened before; deploy.sh hard-codes the path).
- Cannot easily edit + redeploy `nav2_params_real.yaml` or launch files from here without re-cloning the repo locally (outside OneDrive).
- Any config changes in 07-07 or since live only on the car.
- Scripts in `nav2-realcar-deploy/rr/` are only a subset; others (rr_slam.sh, rr_savemap.sh, etc.) live only on the car.

### 12. Hardware / power / safety
- Traction LiPo must be managed (don't leave on after runs).
- Joystick deadman (usually LB / button 4) must be held for any manual motion or override.
- First autonomous runs must be in open space, car on blocks if possible for initial tests.
- No ZED odom/IMU → estimation is wheel odom only (or SLAM pose).

### 13. Unknown current physical state
- Exact map(s) present in `~/rr_maps/`.
- Battery voltage.
- Whether the car was moved / powered since 07-07.
- Whether the REEDS_SHEPP change was ever applied.
- Current running processes and domain.

## Recommended actions to start this session (in order)

1. **Connect from Windows**:
   - Join `roboracer` Wi-Fi.
   - Test: `ssh roboracer@192.168.50.10`
   - If needed, power-cycle car and wait ~60s.

2. **On car, new terminals (source every time)**:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/roboracer_ws/install/setup.bash
   export ROS_DOMAIN_ID=7   # match whatever was used last; confirm with `ros2 topic list`
   ```

3. **Fix clock** (UTC, approximate):
   ```bash
   sudo date -u -s "2026-07-14 12:00:00"
   date
   ```

4. **Clean house**:
   - `ros2 node list`
   - Kill known offenders carefully (by PID preferred).
   - `rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*`
   - `pkill -f slam_toolbox || true; pkill -f amcl || true; ...` (be careful)
   - Restart only one clean `~/t_stack.sh`

5. **Run healthcheck** (even if flow differs, it will surface missing pieces):
   ```bash
   ~/rr/rr_healthcheck.sh
   ```

6. **Decide workflow for this session**:
   - Quick validation of last pending item: live SLAM + fix planner to REEDS_SHEPP + out-and-back to (0,0).
   - Or follow the prepared guide end-to-end with saved map + amcl + autonomous_real.launch.py.
   - Record a bag from the start: `~/rr/rr_record.sh today1`

7. **Inspect the critical param** (after Nav2 up):
   ```bash
   ros2 param get /planner_server GridBased.motion_model_for_search
   # or look in the installed nav2_params_real.yaml
   ```
   Set to REEDS_SHEPP if still DUBIN:
   ```bash
   ros2 param set /planner_server GridBased.motion_model_for_search REEDS_SHEPP
   ```

8. **Drive manually first** (confirm motion, /odom, TF, mux override works).

9. **Only then** attempt autonomous goal.

10. **Log everything**. Pull bags + healthcheck reports + nav2.log to PC after.

## Next steps after basic motion works

- Confirm REEDS_SHEPP allows the return path.
- Validate full B1–B7 chain with healthcheck while goal active.
- If using saved map: re-SLAM a fresh one if the environment changed.
- Once slow reliable runs exist, raise speed gradually (0.5 → 0.8 etc.) and redeploy config.
- Decide if/when to integrate the custom Pure Pursuit (requires extra bridge node).

## Safety reminder (every time)

Grab the gamepad + hold deadman = instant takeover. Release deadman or push sticks = stop. Autonomy is always lower priority.

---

**Session status**: New session opened. Probable problems listed above. Car state unknown — first actions are discovery + cleanup. Ready for user to drive the checklist on the hardware.

Report written from analysis of:
- session_report_2026-07-07.md
- roboracer-hardware-bringup.md
- roboracer-architecture.md
- guide.md
- session.md + session2.md
- ssh-connection-guide.md
- nav2-realcar-deploy scripts + healthcheck
- CLAUDE.md (repo layout)
