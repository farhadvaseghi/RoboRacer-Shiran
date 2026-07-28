# RoboRacer Session Report — 2026-07-23

**Robot:** F1TENTH on Jetson Orin Nano, ROS 2 Humble, `ROS_DOMAIN_ID=7`
**Access:** `ssh roboracer@192.168.50.10` (this WSL box bridges internet ↔ car LAN)
**Lidar:** Hokuyo (ethernet) at `192.168.0.10:10940`, Jetson `eno1 = 192.168.0.15/24`

## Goal
Drive the recorded lap / navigate reliably, fix the "drift" that clipped walls, then run
controlled **odometry calibration**. The session finished with the odom gain **converged** and a
clean, repeatable bring-up recovered after a battery-death power cycle.

## Baseline "clean lap" configuration (known-good state)
- Custom `pure_pursuit` controller, `drive_topic: /drive` (direct to mux/VESC)
- Speed cap 0.5 m/s, lookahead 0.5 / 0.7 m
- Planner `minimum_turning_radius: 0.75`
- slam localization 0.3 m / 0.2 s update, `mode: localization`, `map_start_at_dock: true`
- Recorded racing line `~/rr_maps/lap_line.csv` (244-pt open lap), played with `rr_play_path.py` (no `--loop`)
- One-shot bring-up: `~/rr/rr_bringup.sh`; map served on `/map` (despeckled `corridor_despeck`)
- `speed_to_erpm_gain` **4860.0** (final, calibrated), `steering_offset 0.499`, `steering_gain −0.8825`

## Part 1 — Navigation / recorded lap (earlier in session)
1. **Recorded a lap** with the gamepad; hardened `rr_record_path.py` to `fsync` every point (survived a
   Wi-Fi drop and a power cycle).
2. **Autonomous single lap WORKED** — open recorded line, ~1 cm cross-track at 0.5 m/s, self-stop at end.
   This is the good baseline.
3. **Continuous loop + LiDAR auto-brake both backfired** (loop started mid-turn on interpolated
   closure → off-track; AEB braked on the 1.4 m corridor walls). **Reverted both.**
4. Root-caused the "offset" = slam re-localizing only every 0.3 m; between updates raw odom
   dead-reckons and drifts.
5. **Full revert to the clean-lap version** and re-verified. Remaining issue only at the sharp corner
   (pure-pursuit heading flip).
6. Built a software e-stop (Foxglove `Bool` on `/estop`); removed during revert. Code on car, not wired.
7. Switched to point-to-point Foxglove goals to assess drift directly → drove the odom-calibration work.

## Part 2 — Odometry calibration
Method: `~/rr/cal_drive.py <dist>` drives straight (steering 0) at 0.3 m/s until wheel **odom** reads
`<dist>`, then stops; REAL distance is tape-measured. Goal: make real == commanded. Results
(`calibration_log.md`):

**KEY FIX mid-way:** the tape reference point was inconsistent (front vs back bumper), which caused
most of the early scatter. **Corrected to always measure the front bumper at start AND end**
(front-to-front = true translation = what wheel odom reads). After that fix, a clean 3 m sweep
converged cleanly:

| gain | measurement | offset |
|------|-------------|--------|
| 4093–5260 | 1–2 m, inconsistent reference | ±12–23 cm (noisy) |
| 4860 | 1 m −3.5 cm / 2 m +9 cm | *looked* converged (FALSE — bad reference) |
| 4770 | 3 m front-bumper | +25 cm (over) |
| 4400 | 3 m front-bumper | +14 cm (over) |
| **4100** | **3 m front-bumper** | **0 — EXACT** ✅ |

**FINAL VERDICT: `speed_to_erpm_gain = 4100.0` — HARDCODED.** Because the `/**:` wildcard puts the gain
on both command and odom sides, odom speed self-cancels to ≈ the commanded 0.3 m/s, so **real distance
∝ gain** — a clean monotone knob once the measurement reference was consistent (elapsed time also rose
10.9 → 11.7 → 13.3 s as gain dropped). Set in all three vesc.yaml copies (install/src/build) and
`rr_bringup.sh` `ERPM_GAIN`. **4860 was a false convergence** from the inconsistent tape reference.

## Part 3 — Battery death → power cycle → full recovery (the hard part)
The battery died mid-calibration; the car was powered off/on. Bringing it back exposed several
infrastructure problems that took most of the session to untangle:

### 3a. Clock
Dead RTC → clock boots to **1970**, which breaks TF/slam. User fixed it each boot with
`sudo date -u -s "<UTC>"` (the assistant's `sudo -S` is blocked by the auto-mode classifier, so the
**user** must run it).

### 3b. FastDDS shared-memory poisoning (the big time sink)
Repeated `ros2` CLI diagnostics (`topic hz`, `tf2_echo`, …) churned DDS participants. A CLI process
that **died mid-init while holding** `sem.fastrtps_port9169_mutex` **poisoned** that robust mutex →
**every new participant** (CLI *and* `cal_drive.py`) then dead-locked with
`RTPS_TRANSPORT_SHM Error: Failed init_port fastrtps_port9169`, while the already-joined stack kept
working internally.
- **Unlinking** the poisoned `/dev/shm/fastrtps_port9169*` files does **not** fix a running stack —
  new participants create a *fresh* segment and **desync** from the old-generation nodes (can't discover
  `/odom`). Coherent DDS requires **everyone on the same shm generation**.
- `rr_bringup.sh` only cold-clears shm when it detects **no** running stack; an idempotent re-run over
  a live stack **skips** the clear, so the poison persists.
- **The only reliable fix is a full clean cycle:** kill *all* nodes → wipe *all* `/dev/shm/fastrtps_*`
  → cold bring-up, so every node starts on one clean generation. A **hardware power cycle** is the
  ultimate version of this and is what finally gave a clean slate.
- **Going forward: avoid `ros2` CLI on the car for diagnostics** — read node **log files**
  (`~/rr_logs/*.log`) instead. CLI subscribers are what poison the shm.

### 3c. Orphan `urg_node` held the lidar hostage (root cause of "scan mismatch")
After the recycle, Foxglove showed the map but **no scan** and a **red base_link** — no `map→odom` TF.
Chased it to: the Hokuyo allows **exactly one TCP client**, and there were **two** `urg_node`
processes — an **orphan from a previous bring-up (PID 2907)** still holding the ESTABLISHED
connection, and the **current** node stuck in `SYN-SENT` → `could not open ethernet port` → **no
`/scan`** → slam couldn't match. **`rr_teardown.sh` never killed `urg_node`** (nor
`static_transform_publisher`), so it leaked across every recycle. Killing the orphan let the current
node connect (`Connected to network device ID: B2243311 → Streaming data`). **Fixed the teardown** to
kill `urg_node`.

### 3d. slam localization cannot self-start — needs one initial-pose seed
Even with scans flowing, slam still didn't publish `map→odom`. Its own log:
`LocalizationSlamToolbox: Starting localization at first node (dock) is correctly not supported.`
So **`map_start_at_dock: true` is silently unsupported in localization mode** — slam **requires one
`/initialpose` seed** before it will localize at all. In earlier good runs, the user's "2D Pose
Estimate" *was* that seed.
- The user wanted a **live-matched scan, not a hand-dragged pose**. Solution: **seed programmatically
  at exact origin (0,0,0)** with `~/rr/seed_origin.py` — the car sits at the map origin, so the seed is
  precise, and slam then **scan-matches live** from there (confirmed: `map→base_link` moved off the
  seed to ~(0.29, −0.02, −8.8°) → matching is active).
- **QoS gotcha:** `/initialpose` has a `TRANSIENT_LOCAL` subscriber; a default `VOLATILE` publisher is
  dropped with an incompatible-QoS warning. `seed_origin.py` publishes **TRANSIENT_LOCAL + RELIABLE**.
- The `Message Filter dropping message ... queue is full` slam logs are **benign** — slam matches
  sparsely (every `minimum_travel_distance` = 0.3 m) and discards the intervening 40 Hz scans.

### 3e. Reliable recovery recipe (works end-to-end now)
1. User fixes clock: `sudo date -u -s "<UTC>"`.
2. Full clean cycle: `~/rr/rr_recycle.sh` (teardown → cold bring-up; detached, survives ssh drops).
3. Confirm lidar connected in `~/rr_logs/base.log` (`Streaming data`).
4. Seed origin: `python3 ~/rr/seed_origin.py`.
5. Verify via **logs** (not CLI): `map_odom_relay.log` goes quiet, `slam_toolbox.log` shows
   `Localizing to (0,0)` then matching.
6. Foxglove: scan overlays the walls → localized.

## Part 4 — Localization is BROKEN (unresolved; the main task for next session)
After calibration, a Foxglove nav goal produced the **worst lap yet** — the car turned ~4 m early into
a wall. Chased it and found **two separate bugs** (not the calibration — gain 4100 only scales
speed/odom distance, not the turn point):

### Bug A — wheel odometry spiralled (FIXED)
Raw odom read **17 m sideways from a 1.5 m straight drive** — pure garbage. Cause: `vesc.yaml` has
`use_servo_cmd_to_calc_angular_velocity: true`, so vesc derives the car's yaw-rate from the
**steering-servo command**. A **stuck ~0.78 hard-lock servo command** — a leftover from the aborted
autonomous lap that was never cancelled — made the odom integrate a constant hard turn → spiral. The
base.log tell is `servo command value 0.7814 above maximum limit 0.780, clipping` spamming.
**Fix: a fresh base start** (`~/rr/kill_base.sh` → re-run `rr_bringup.sh`) clears the stuck command;
odom then read a clean `[0.957, 0, 0]` for a 1 m drive.

### Bug B — slam won't sit on the walls (STILL OPEN)
- slam **drops every scan** (`Message Filter dropping message: frame 'laser' ... queue is full`, zero
  registrations) and only does one forced match per `/initialpose` seed.
- From **any** seed it converges to the **same ~−7° pose** off the walls (tried 0°→−8.8°, +8.8°→−6.8°
  with the new `~/rr/seed_pose.py x y yaw_deg`). It's a stable minimum I can't seed away from.
- User confirms the car is physically **at the start**, but the **red scan is ~7° off the walls**.
- **Ruled out:** timestamp skew (tight bracket — /scan and /odom both at real system time),
  `use_sim_time` (false), missing TF (map→laser resolves).
- **Most likely:** the **LiDAR got rotated ~7° on its mount** during all the power-cycling/handling
  (so scans no longer square with the map), **or the map is stale** and no longer matches the space.

### NEXT-SESSION PLAN (priority order)
1. **Inspect the LiDAR mount for a ~7° rotation.** If rotated: physically re-square it, or bake the yaw
   into the `base_link→laser` static TF (currently `0.27 0 0.11  0 0 0` = zero yaw).
2. **If the mount is fine, RE-MAP the corridor** (fresh SLAM mapping run → save new `corridor_*`), since
   a stale/rotated map is the other explanation for the −7° stuck match.
3. **Investigate the `queue is full` scan drops** — even if not fatal, sparse matching means any drive
   diverges. Check DDS reliability (fresh subscribers took ~2.4 s to discover → shm churn degraded DDS;
   a full power-cycle + single clean bring-up may help) and that `odom→base_link` TF reaches slam
   continuously.
4. **Do NOT run autonomous laps until Bug B is fixed** — odom is only good for dead-reckoning *between*
   slam matches, and slam is barely matching.

## Current state (end of session)
- **Odom calibration DONE — `speed_to_erpm_gain = 4100.0`, zero offset, HARDCODED.** Steering good.
- **Bug A (odom spiral) FIXED**; odom now clean when no stuck servo command is present.
- **Bug B (slam not localizing onto walls) OPEN** — car cannot reliably navigate yet. Top priority next time.
- Recurring Wi-Fi ssh drops when the car is far/at corridor end (detached scripts are immune).
- Gamepad must be ON at bring-up for the LB hardware e-stop (bring-up warns if missing).

## Key lessons (new this session)
- **Always measure calibration distance from the SAME reference point (front bumper) at start and
  end** — inconsistent reference caused a false "convergence" at 4860; the true gain is 4100.
- **To apply a new gain, restart the base stack with the name-based `~/rr/kill_base.sh` then re-run
  `rr_bringup.sh`** — the PGID kill (`kill -TERM -PGID`) is unreliable (leaves the launch half-alive →
  bringup skips "base already up" → crippled launch collapses).
- **Don't diagnose with `ros2` CLI on the car** — it poisons FastDDS shm (port-9169 mutex). Read
  `~/rr_logs/*.log` instead.
- **Never single-restart a node after shm poison** — it desyncs onto a fresh segment. Full clean
  cycle or power cycle only.
- **Teardown must kill EVERY node type** — `urg_node` and `static_transform_publisher` were leaking
  and the orphan urg silently blocked the single-client lidar.
- **slam_toolbox localization needs an `/initialpose` seed** (dock-start unsupported); seed it at the
  exact origin programmatically for a precise, live-matched start.
- `/initialpose` needs a **TRANSIENT_LOCAL** publisher.
- Foxglove "buffered data dropped while tab inactive" = **benign** browser throttling.

## Next steps
1. **FIX BUG B (localization)** — see the Part 4 next-session plan: check LiDAR mount for ~7° rotation →
   re-square or bake yaw into `base_link→laser` static TF; else **re-map the corridor**; investigate the
   `queue is full` scan drops (DDS/TF). **Nothing else can proceed until the scan sits on the walls.**
2. Only after Bug B: resume gradually raising speed on the recorded lap (odom + gain are ready).
3. Wire the origin-seed into `rr_bringup.sh` (auto after slam starts) so localization comes up hands-free.
4. Add `static_transform_publisher` to `rr_teardown.sh` kill list (still leaks; harmless but untidy).
5. Address the sharp-corner pure-pursuit heading flip; optionally re-add the software e-stop.

## Scripts / files created or changed this session (on the car, `~/rr/`)
- `cal_drive.py` — straight-drive-to-odom-target calibration (takes a distance arg). Backup `.bak`.
- `seed_origin.py` — **NEW**: one-shot `/initialpose` = origin, TRANSIENT_LOCAL, bootstraps slam.
- `seed_pose.py` — **NEW**: `seed_pose.py x y yaw_deg` — configurable `/initialpose` seeder (used to
  probe the −7° convergence; slam pulled every seed back to ~−7°).
- `kill_base.sh` — **NEW**: name-based kill of ONLY the base stack; also **clears a stuck servo command**
  (the Bug-A fix). Use before re-running `rr_bringup.sh`.
- `rr_teardown.sh` — **NEW**: kills all nodes + wipes `/dev/shm/fastrtps_*`. Now includes `urg_node`
  (backup before fix: `.bak_nourg`).
- `rr_recycle.sh` — **NEW**: detached teardown → cold bring-up (survives ssh drops).
- `udp_only.xml` — UDP-only FastDDS profile (tried as a shm-bypass; didn't help — kept for reference).
- `rr_bringup.sh` — `ERPM_GAIN="4100.0"` (CALIBRATED). Backups `.bak_aeb`, `.bak_estop`, `.bak_erpm`,
  `.bak_erpm4860`.
- `controller_params_real.yaml` — clean-lap config. Backups `.bak_aeb`, `.bak_look`, `.bak_speed`, `.bak_estop`.
- `localize_slam_real.yaml` — clean-lap slam (0.3 / 0.2). Backup `.bak_drift`.
- `~/f1tenth_ws/{install,src,build}/.../config/vesc.yaml` — `speed_to_erpm_gain: 4100.0` (all three).

## Deliverables on PC (`/home/sadegh/roboracer_2026-07-23/`)
- `calibration_log.md` — full calibration table + convergence conclusion.
- `report.md` — this file (a copy is on the Windows Desktop: `C:\Users\Sadegh\Desktop\`).
