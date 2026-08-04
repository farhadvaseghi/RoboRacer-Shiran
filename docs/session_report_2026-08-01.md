# RoboRacer localization session — 2026-08-01

**Developer: Milad Bahari Qaragoz**

Goal that started it: **"odom has a lot of unmodelable noise and inaccuracies —
drop odom from localization and localize the robot on sensor data only."**

Outcome in one line: the idea was right about odom being the problem but wrong
about the remedy. The LiDAR *cannot* replace odom on this track, and the actual
defect turned out to be that odom's heading was never measured at all. That is
fixed and verified — localization went from unusable to 76.8% on-wall while
driving, and the car now drives autonomously to Foxglove goals. Multi-waypoint
routes still fail, for an unrelated reason found along the way (§7d).

Two of the faults in here were in the measuring tools, not the robot, and one
was a safety bug that kept the car driving with nothing commanding it. Both are
written up rather than quietly fixed, because the wrong conclusions they
produced were believed for several rounds each.

---

## 1. Starting constraint

slam_toolbox **cannot run without an `odom→base_link` transform**. It uses odom
as the initial guess for every scan match and publishes `map→odom` as a
correction on top. So "drop odom" could only mean one of two things:

- **A** — make the odom prior nearly irrelevant (match far more often)
- **B** — replace the *source* of odom with something sensor-derived (rf2o
  laser odometry)

We chose A first as a cheap test, with B to follow. B was later killed by
measurement (§3).

---

## 2. First results — BUG B closed

`rr_scan_align.py` had been written on 07-31 and never run. It ran clean:

```
ALIGNMENT AT THE CURRENT POSE : 90.5% of returns on walls
BEST OVER A YAW SWEEP         : +0.0 deg -> 90.5%
VERDICT: aligned.
```

**The long-standing "scan won't sit on the walls" bug (BUG B) is resolved.**
The live `base_link→laser` static TF is now `x=0.370 y=0.154 yaw=4.27°` — the
rotation correction had already been baked in by someone; the older notes
saying `0.27 0 0.11` zero-yaw were stale.

---

## 3. rf2o (laser-only odometry) — rejected on evidence

Built `rr_scan_observability.py`: perturbs the pose in 3 DOF and scores the
likelihood field (mean distance from each return to the nearest wall).

| pose | longitudinal blind zone | lateral blind zone | long. cost spread | lat. cost spread |
|---|---|---|---|---|
| start (x≈0) | ±0.10 m | ±0.05 m | — | — |
| mid-corridor x=7.45 | flat over ±0.6 m, min at −0.30 | — | 0.012 m | — |
| mid-corridor x=14.60 | **0.90 m wide** | **0.10 m** | **0.063 m** | **0.396 m** |

Lateral carries ~6× more information than longitudinal, and at both
mid-corridor poses the believed position was *not* the best-fitting one.

**Conclusion: the LiDAR cannot see where the car is ALONG this corridor.**
Laser-only odometry would slide exactly where you drive fastest. rf2o was
dropped. Wheel odom has to stay in the loop — it is the only source for that
axis.

---

## 4. The A/B on slam matching rate

Built `rr_loc_monitor.py` (scan fit + map→odom correction + path length) and
`rr_slam_prior.sh {scan|odom|show}` to flip profiles and restart+re-seed slam.

- `scan` = `minimum_travel_distance 0.05`, `minimum_travel_heading 0.05`,
  `minimum_time_interval 0.1`
- `odom` = stock 0.3 / 0.3 / 0.2

The parameter that mattered was **`minimum_travel_heading: 0.3` rad = 17°** —
the car could turn 17° before slam ever re-checked. `correlation_search_space_
dimension` was deliberately left at 0.5 (a wider window only helps when the
prior is grossly wrong; matching more often makes the prior error *smaller*).

Result: `scan` needed ~35-40% less correction (total pull 54.5 m vs 90.4/78.8 m
across two `odom` repeats). Real, but second-order. **`scan` is the live
profile.**

Then every driving run collapsed to ~20-26% on-wall regardless of profile or
speed — which is what sent us to the root cause.

---

## 5. Root cause — odom's heading was never measured

From `vesc_to_odom.cpp:105-127`:

```cpp
current_steering_angle = (last_servo_cmd_->data - offset) / gain;
current_angular_velocity = current_speed * tan(current_steering_angle) / wheelbase_;
yaw_ += current_angular_velocity * dt.seconds();
```

Heading is integrated from the **commanded servo position** — open-loop, no
sensor, assuming zero servo lag and zero tyre slip. `base.log` was logging
`servo command value 0.799 above maximum limit 0.780, clipping`, so the value
being integrated was not even the value being sent.

Also found: `use_servo_cmd_to_calc_angular_velocity: false` is **not** a usable
knob — with it off, `yaw_` is never updated at all. And speed is deadbanded
(`|v| < 0.05 → 0`).

### Measured with a tape, slam not involved (`rr_odom_ruler.py`)

Straight line, front bumper marked at both ends:

| | reading |
|---|---|
| tape | **3.000 m** |
| odom, run 1 | 2.709 m |
| odom, run 2 | 2.694 m |

→ odom **under-reports 10%**, repeatable to 0.55%.

Return-to-floor-marks loops (truth = 0 by construction):

| run | path | position error | heading read |
|---|---|---|---|
| small, fwd+rev | 22.9 m | 1.48 m | +319° |
| full lap, fast | 39.4 m | **12.62 m** | +637° |
| full lap, slow | 38.9 m | **12.90 m** | +581° |

One lap = one full rotation, so true heading is 360° → **heading error ~250°
per lap**. Position error was 32% of distance driven. No scan matcher can work
from a prior that wrong.

---

## 6. The fix, applied and verified

`rr_imu_probe.py` confirmed the VESC's own IMU is alive at 44-50 Hz.

**Units trap worth remembering:** `vesc_driver.cpp:234-240` copies raw VESC
packet fields into `sensor_msgs/Imu` with **no conversion**. Acceleration is in
**g** and angular rate in **deg/s**, despite the message type meaning m/s² and
rad/s. Gravity read 1.070 → real data, just mislabelled. The VESC's own AHRS
`orientation` quaternion drifted **10.4° in 20 s at rest**, so it is unusable —
it just integrates the same biased gyro.

### What was changed

- **New `~/rr/rr_gyro_odom.py`** owns `odom→base_link`: heading from the
  measured gyro (deg/s→rad/s, bias auto-measured over 4 s at standstill,
  midpoint integration), speed still from `/odom`'s twist.
- **`vesc.yaml`** (all three copies) and **`rr_bringup.sh`** so a rebuild can't
  silently undo it:

| | was | now |
|---|---|---|
| `speed_to_erpm_gain` | 4100 | **3690** |
| `publish_tf` | true | **false** |
| `use_servo_cmd_to_calc_angular_velocity` | true | **false** |

- `rr_bringup.sh` gained **step 3.6** (spawns the node *before* slam, which
  needs the transform) and a `gyro_odom` roll-call entry.
- Backups: `*.bak_gyro_20260801_211740`.

### Verified

Measured gyro bias **−0.5881 deg/s** from 200 samples — independently matching
the probe's −0.6075.

| | before | after |
|---|---|---|
| path driven | 39 m | **47.8 m** (longer) |
| position error | 12.62 m | **0.490 m** |
| as % of path | 32% | **1.0%** |
| heading error | ~250° | **+2.83°** |

**~26× better on position, ~88× on heading.** In driving runs this showed as
max yaw correction 41.9° → 3.11°, total correction pull 54.5 → 13.9 m, and
`map→odom` staying bounded within ~3 m instead of wandering tens of metres off
a 38×10 m map.

---

## 7. Corrections to earlier conclusions

Recorded because they were stated confidently and were wrong:

1. **"odom over-reports by 55%"** — WRONG, and the opposite. It came from
   comparing against a slam pose that was itself lost. Tape says odom
   *under*-reports 10%. That comparison is circular whenever localization is
   broken; the ruler exists to break the circle.
2. **"on-wall will reach the 80s after the gyro fix"** — looked wrong at the
   time (21% → 33.7%), but that reading came from the broken monitor. With a
   working tool and ZUPT it measured **76.8%**. The prediction was roughly
   right; the instrument disagreed. Retraction retracted — but note it was
   believed to be a failed prediction for several rounds, and the temptation
   then was to invent new physical causes for what was an instrument fault.
3. **`rr_scan_observability.py` crashed** on the first mid-corridor run (a
   `None` formatting bug) — and the crashed section contained the most
   important finding. Fixed.
4. **The TF-health probes were run parked**, which is the wrong condition;
   parked was already known healthy.
5. **`rr_loc_monitor.py` was itself broken for every moving run.** It called
   `rclpy.spin_once()` once per 0.25 s sample, which services *one* callback,
   while `/tf` arrives at ~100 Hz and `/scan` at 40 Hz. Its TF buffer fell
   behind by **2.2 s mean, 4.8 s max**, so the fit was scored against a pose
   the car had long since left. **Every moving scan-fit number produced before
   the rewrite is invalid**, including the §4 A/B fit columns (the map→odom
   correction statistics are less affected). Rewritten to run the sampler on a
   ROS timer inside a continuous spin; pose age is now 0.035 s and the tool
   reports its own staleness so this can never fail silently again.

---

## 7b. Zero-velocity update — and the first trustworthy driving numbers

After the monitor was fixed, a standstill check read **39.2% on-wall, best fit
at +8.5° yaw**. The position was fine; the *heading* had drifted 8.5° while the
car sat parked. Cause: `rr_gyro_odom` integrated the gyro on every message
regardless of motion, so residual bias walked into yaw forever while stopped.

**Fix — ZUPT** (`ZUPT_HOLD_S = 0.5`, `BIAS_TAU_S = 30`): an Ackermann car
cannot rotate without its wheels turning, so any yaw rate reported while
stopped *is* bias. While `|speed| < 0.02` the node holds heading and folds the
reading back into the bias estimate. This also removes the earlier
once-per-bringup limitation — temperature drift is now tracked continuously.
New helper `~/rr/rr_restart_gyro.sh` (detached; car must be still; **re-seed
after — a restart zeroes x/y/yaw**).

### Results — fresh timestamps, 92-97% of samples moving

| | scan (`gyro3`) | odom (`odom3`) |
|---|---|---|
| **on-wall while moving** | **76.8%** | 71.0% |
| fit mean | **0.075 m** | 0.098 m |
| fit p90 | **0.171 m** | 0.323 m |
| fit worst | **0.307 m** | 0.458 m |
| lookup age | 0.065 s | 0.062 s |

**76.8%, against the 21-33% that the broken tool had been reporting.**

Post-drive alignment check: **84.8% at +0.5° after a full lap** (the same check
read 39.2% at +8.5° before ZUPT). **Heading now survives a drive.**

### One more metric retraction

**Total map→odom correction pull is not a quality metric.** Here `scan` needs
*more* pull (29.6 m / 125 jumps) than `odom` (22.9 m / 104) while fitting
clearly better. Matching 6× more often produces more frequent small
corrections, so the sum grows with match rate regardless of accuracy. **Scan
fit (on-wall, p90) is the metric.** The §4 conclusion that `scan` wins still
stands; the evidence quoted for it did not.

---

## 7c. SAFETY: the car kept driving with nothing commanding it

Symptom: *"as soon as I take my finger off the deadman it starts moving"*, and a
fresh `/initialpose` would not stop it.

**It was not a stale nav2 goal** — both action servers reported **0 goals to
cancel**.

**Root cause: `rr_wall_aeb.py`.** That node owns `/drive` and ticks at 20 Hz
republishing `self.last_cmd`, which was **never invalidated**. It had a
`scan_timeout` but no *command* timeout. So when the controller was killed
mid-run, the AEB kept re-issuing its final `speed=0.40` indefinitely, with
nothing upstream commanding anything.

**The deadman cannot save you from this.** It is a mux override with a 0.2 s
timeout — release it and control goes straight back to the stale command.

Fixed: added `CMD_TIMEOUT_S = 0.3` and `last_cmd_time`; `tick()` now zeroes and
returns when its input goes quiet. **The rule: a pass-through that owns an
actuator topic must fail to ZERO, never to its last input.**

**Secondary finding — an empty `/control/plan` does not reliably stick.** The
controller was following a *latched* path long after its goal ended.
`rr_costmap_reset` did fire on `/initialpose` (its log confirms) but the path
came back: `/control/plan` is TRANSIENT_LOCAL with **two** publishers
(`rr_costmap_reset` and `plan_qos_relay`), so one can undo the other's empty.
Repeating the empty 8× over 2 s is what made it stick. Hence
**`rr_cancel_goals.py`** — cancels all goals on **both** `navigate_to_pose` and
`navigate_through_poses` (the mission uses the latter; cancelling one leaves the
other driving) then empties the path 8×. **Use that, not a pose estimate, to
call a run off.**

**The AEB is now out of the chain** (`rr_aeb_off.sh`): `drive_topic` back to
`/drive`, controller publishes direct, bringup skips the AEB. It was braking on
walls at 0.23 m in a ~1.4 m corridor — the same verdict as 07-23. **Consequence:
there is no automatic obstacle braking at all now.**

---

## 7d. Driving it: Foxglove works, the waypoint mission does not

**Foxglove single goals drive close to perfect.** `Goal succeeded`, clean
tracking through the corner.

**Mission attempt 1 — ABORTED 2.5 s in.** Chain from nav.log:

```
controller_server: Lookup would require extrapolation into the future.
  Requested 1785614766.686, latest data 1785614766.664       (22 ms)
controller_server: Unable to transform robot pose into global plan's frame
controller_server: Controller patience exceeded
[follow_path] Aborting handle  →  BT recovery  →  spin/wait/backup all time out
bt_navigator: Goal failed
```

**nav2's `controller_server` killed the mission — the controller this stack does
not even use.** The custom pure_pursuit drives the car; nav2's `FollowPath` was
assumed harmless because nothing bridges its output. It is not harmless: the BT
still runs it, and its failure **aborts the whole goal**. `failure_tolerance` was
**0.3 s**, and the box is CPU-starved (`Behavior Tree tick rate 100.00 was
exceeded!`, `Control loop missed its desired rate of 20.0000Hz`).

Fixed: `failure_tolerance: 0.3 → 10.0`, nav2 restarted via the new
`rr_restart_nav2.sh` (these servers read params once in `configure()`, so a live
`ros2 param set` is accepted and silently ignored).

**Mission attempt 2 — it drove, but "forgets to turn and keeps going straight".**
The path was *correct*: 353 points ending at Goal 3. The controller was not.

```
[Robot] x=10.95 y=0.95 yaw=0.25
[Path]  nearest=278 progress=278 target_idx=278 target=(0.53, 6.44)
        x_local=-8.70 y_local=7.92 dist_goal=11.77
[Control] steer=0.03
```

The path had 279 points and `nearest` is **278 — the last one**. The progress
index has run to the end, so the target is the final goal 11.77 m away in a
straight line, `dist_goal` *grows* every cycle, and it steers 0.03: dead ahead.
When a fresh path arrives it reads `nearest=5` for one cycle, then jumps to
`nearest=309` of 310.

**Cause: the route doubles back on itself** — out along y≈0.5, north, then west
along y≈6.5. The controller's monotonic progress index cannot handle a path that
returns near itself. A Foxglove single goal never doubles back, which is exactly
why the same corner works there. **This is the same defect as the `--loop`
failure noted on 07-23** ("it treats the LAST path point as the goal").

Two fixes, **neither applied** — the decision was to keep driving with Foxglove
for now:

- **A** — send the waypoints one at a time (`navigate_to_pose` each), so every
  path is simple and monotonic. Contained Python change, no rebuild. Cost: the
  car pauses at each waypoint.
- **B** — fix the progress-index search in `roboracer_control` (constrain it to a
  window ahead instead of letting it jump). The real fix, needed for any lap or
  figure-eight route, but it is a C++ change.

---

## 8. Where it stands

**Closed:**
- BUG B (scan/map alignment)
- odometry heading — was fabricated, now measured
- odom scale — recalibrated 4100 → 3690
- rf2o — ruled out on measured evidence
- **localization while driving — 76.8% on-wall, verified with a fresh tool**
- **heading holds across a lap** (84.8% @ +0.5° post-drive)
- **slam profile — `scan` wins on fit** (p90 0.171 vs 0.323); it is live
- **runaway-car safety bug** — AEB stale-command republish
- **mission aborting at 2.5 s** — nav2 `failure_tolerance`
- **autonomous driving works via Foxglove goals**

**Open:**
- **The waypoint mission cannot follow a doubling-back route** (§7d). Fix A or B
  required before multi-waypoint autonomy works at all.
- **No automatic obstacle braking.** The AEB is out of the chain. Only the
  deadman (wheels, while held) and `rr_cancel_goals.py` stop the car.
- **Fit dips mid-run.** p90 0.171 m means ~10% of samples sit >17 cm off;
  `gyro3` dropped to 47-54% around t=18-26 s. Prime suspect is the featureless
  stretch where the longitudinal blind zone is 0.9 m.
- **CPU headroom.** Only 6-10% idle, load ~11-12 on 6 cores.
  `controller_server`, `smoother_server`, `velocity_smoother` and
  `waypoint_follower` are unused by this stack and burn roughly a full core.
  Note `controller_server` cannot simply be killed — the default BT calls it.
- **`max_lookahead_distance` is capped at 0.80 m**, already reached at 1.0 m/s,
  so going faster gives no extra lookahead and the car will start cutting
  corners. Raise it with speed (~1.20 at 1.5 m/s, ~1.60 at 2.0).
- **Everything is car-only and unpushed** — same trap that caused the 07-30
  regression.

---

## 9. Next steps, in order

1. **Raise speed on Foxglove goals**, in steps, measuring each time:
   ```bash
   bash ~/rr/rr_set_speed.sh 1.3
   python3 ~/rr/rr_loc_monitor.py 40 speed13     # drive during it
   ```
   Watch **on-wall while moving** and **p90**. Baseline at 1.0 m/s is
   76.8% / 0.171 m. If p90 climbs past ~0.25 m, that is localization losing the
   car — stop there. Raise `max_lookahead_distance` alongside speed.
   Keep `rr_cancel_goals.py` ready in a second terminal; there is no AEB.
2. **Fix the multi-waypoint route** — A (sequential goals) to get it working, B
   (controller index search) to fix it properly.
3. **Find where the fit dips.** Check whether the 47-54% stretches line up with
   the featureless corridor. If so it is the known 0.9 m longitudinal blind
   zone, and the answer is a route that keeps geometry in view, not more tuning.
4. **Push all of it to git.** None of this exists off the car — the same trap
   that caused the 07-30 regression.
5. Optional, if CPU becomes the limit: stop `smoother_server`,
   `velocity_smoother` and `waypoint_follower`. **Not `controller_server`** —
   the default BT calls it, and a missing server fails the goal outright. The
   clean version is a custom BT with no `FollowPath` node.

---

## 10. Tools built this session (all in `~/rr/` on the car)

| tool | what it answers |
|---|---|
| `rr_scan_align.py` | does the scan sit on the map's walls at this pose (pre-existing, first run this session) |
| `rr_scan_observability.py` | can the scan alone constrain the pose — 3-DOF cost sweep |
| `rr_loc_monitor.py` | live localization quality; **read LOOKUP TIMING first** |
| `rr_slam_prior.sh` | flip slam between scan-heavy and odom-heavy, restart + re-seed |
| `rr_pose_capture.py` | print the believed pose so a restart doesn't teleport to the origin |
| `rr_odom_ruler.py` | raw odom vs a tape measure — no slam involved |
| `rr_imu_probe.py` | is the VESC gyro usable, and in what units |
| `rr_gyro_odom.py` | **the fix** — publishes `odom→base_link` from the gyro |
| `rr_tf_rate.py` | how a transform is actually delivered (rate, gaps, duplicate publishers) |
| `rr_cancel_goals.py` | **the stop button** — cancels both action servers, empties the path |
| `rr_restart_gyro.sh` | restart just the gyro odom (car still; re-seed after) |
| `rr_restart_nav2.sh` | restart just nav2 after a params edit |
| `rr_set_speed.sh` | set drive speed and restart the controller |

### Hard-won gotchas

- `set -u` + ROS `setup.bash` = `AMENT_TRACE_SETUP_FILES: unbound variable`.
  Source inside `( set +u; ... )`.
- Restarting slam discards the pose — always re-seed, and **at the captured
  pose**, not the origin.
- The car must be **still** for the first ~6 s of bringup or the gyro bias is
  measured wrong. The log prints the value; it should be near −0.6 deg/s.
- The car now drives ~10% slower for the same stick input. That's the gain fix.
- `rclpy.spin_once()` services **one** callback. Never drive a sampling loop
  with it.
- **A pose estimate does not stop the car.** Use `rr_cancel_goals.py`.
- **The deadman does not stop the car either** — it is a mux override with a
  0.2 s timeout, so it only masks whatever is still publishing.
- **`pgrep -c` double-counts `ros2 run` nodes** (python wrapper + real
  executable) and miscounted elsewhere too. Confirm instance counts with `ps`.
- nav2 servers read parameters once in `configure()`. A live `ros2 param set` is
  accepted and **silently ignored** — restart nav2.
- Killing nav2 by pattern: match `lifecycle_manager_navigatio[n]` in full. A
  bare `lifecycle_manager` also kills `map_clean_lifecycle` and drops the map.
- Wi-Fi drops SSH mid-command at range (`exit 255`). Run anything that restarts
  nodes under `nohup setsid` so a drop cannot kill it half-done.

---

Work carried out and documented by **Milad Bahari Qaragoz**, 2026-08-01.
