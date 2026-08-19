# RoboRacer Session Report — 2026-08-03

## Scope

This session collected the RoboRacer-owned files from the physical robot,
reviewed the current waypoint mission and control stack, and diagnosed why a
normal Foxglove goal followed its path correctly while the continuous waypoint
mission departed from its generated path.

The robot was accessed on ROS 2 domain `7`. Its project directories, maps,
logs, workspaces, team repositories, and recorded artifacts were copied into a
separate WSL snapshot. The robot and its running software were not modified
during collection or diagnosis.

## Control stack confirmed

`rr_waypoint_mission.py` sends every intermediate pose as one Nav2
`NavigateThroughPoses` action goal. Nav2's SMAC Hybrid-A* planner produces the
path, but Nav2's controller does not command the physical vehicle. The runtime
control flow is:

```text
rr_waypoint_mission.py
  -> /navigate_through_poses
  -> Nav2 SMAC Hybrid-A* plan
  -> /nav2/plan_raw
  -> path_relay
  -> /control/plan
  -> custom roboracer_control/pure_pursuit_controller
  -> /drive
  -> ackermann_mux
  -> VESC
```

The physical-car Pure Pursuit configuration observed during the session used:

- map-frame odometry from `/odometry/map`;
- the path on `/control/plan`;
- Ackermann output on `/drive`;
- a 0.25 m wheelbase;
- a 0.32 rad steering limit;
- velocity-scaled lookahead from 0.5 m to 0.8 m;
- 1.0 m/s nominal and straight speed;
- 0.8 m/s turn speed;
- a 0.25 m goal tolerance;
- reverse effectively disabled.

## Failure reproduced from the robot logs

The controller log contains a direct example of the waypoint-mission failure.
While the car was still near the start of a 351-point loop, progress changed
from index 47 to the final index 350 in one control update:

```text
nearest=47 progress=350 target_idx=350
target=(-0.02, 4.99) x_local=-2.63 y_local=4.84
```

The new target was behind and far to the side of the vehicle. The controller
therefore stopped following the nearby part of the path and steered toward the
end of the loop. The logged cross-track error then grew from approximately
0.064 m to 0.500 m.

Normal Foxglove `NavigateToPose` goals usually produce point-to-point paths
that do not return close to an earlier segment. The waypoint mission produces
a loop whose final section is spatially close to its initial section, exposing
the controller's ambiguous nearest-point matching.

## Root causes

### Unbounded Pure Pursuit progress search

`find_nearest_forward_index()` previously searched every point from the
current progress index to the end of the path. This prevented backward jumps,
but allowed an arbitrarily large forward jump whenever a later portion of a
loop passed near the car.

The segment-based lookahead had a related unsafe fallback. If it found no
lookahead-circle intersection, it selected `path_.back()` directly. That could
recreate the same start-to-finish shortcut even after constraining nearest-point
matching.

### Path relay replacing Nav2 geometry

The old `path_relay` mode defaulted to `use_forward_oval_route=True`. In that
mode it extracted only the final point of each Nav2 plan and continuously
generated a hardcoded oval between the current pose and that goal. It did not
faithfully preserve the SMAC plan or the geometry represented by a multi-pose
mission.

The hardcoded oval values were also stale for the resized physical track and
could cut through the infield wall.

## Changes made

### Bounded controller progress matching

A new Pure Pursuit parameter was added:

```yaml
max_progress_search_distance: 3.0
```

The nearest-point search now considers only the next 3.0 m of path arc length
from `last_progress_idx_`. A spatially nearby segment later in a loop is not
eligible until the vehicle has physically progressed close to that part of the
route.

The parameter is validated at controller startup and must be greater than
zero. Its default is also 3.0 m, so physical-car parameter files that do not
yet contain the new key still receive the protection after the new controller
binary is deployed.

### Bounded lookahead fallback

The segment-based lookahead search now uses the same bounded forward distance.
If no circle intersection exists inside that local section, it selects the end
of the bounded section rather than the final point of the complete path.

This prevents a localization disturbance, path gap, or loop intersection from
turning a local tracking failure into an immediate command toward the finish.

### Preserve Nav2's real plan

`path_relay_node.py` now defaults to:

```python
use_forward_oval_route = False
```

With this setting, the relay publishes the actual Nav2 SMAC plan on `/plan`,
`/control/plan`, and `/control/forward_route`. The legacy oval generator
remains available only when explicitly enabled. Its optional dimensions were
updated from the old 20 m geometry to the current 60 m track geometry.

## Files changed

- `roboracer_control/src/pure_pursuit_controller.cpp`
  - added `max_progress_search_distance`;
  - bounded nearest-point matching by path arc length;
  - bounded segment-lookahead search and fallback;
  - added startup validation for the new parameter.
- `roboracer_control/config/controller_params.yaml`
  - documented and configured the 3.0 m progress-search window.
- `roboracer_estimation/roboracer_estimation/path_relay_node.py`
  - made retained Nav2-plan relay the default;
  - retained the oval generator as an explicit legacy option;
  - updated optional oval dimensions for the current track.

## Validation

- `path_relay_node.py` passed Python bytecode compilation.
- The modified C++ Pure Pursuit controller compiled and linked successfully.
- `git diff --check` reported no whitespace errors.
- No physical-car motion test was performed during this session.

The isolated `colcon` build reached and completed the controller's compile and
link target. Its later isolated install step stalled in the local WSL test
environment and was interrupted; this did not affect the successful compiler
result or any project workspace.

## Deployment update

The controller fix was deployed to the Jetson on 2026-08-03 and the
`roboracer_control` package rebuilt successfully. The running controller was
restarted from that build with `max_progress_search_distance: 3.0`; the live
ROS parameter query returned `3.0`.

The physical command chain was also changed to:

```text
pure_pursuit_controller (/drive_nav)
  -> rr_wall_aeb
  -> /drive
  -> ackermann_mux
  -> VESC
```

The LiDAR AEB is configured brake-only for the initial validation
(`enable_recovery: false`). Automatic recovery was deliberately disabled
because the rear 90 degrees are outside the Hokuyo field of view. With no
active controller command, the verified `/drive` output was `0.0 m/s` and the
AEB reported `PASS (no command)`.

During the first supervised mission attempt, the controller correctly issued
`1.0 m/s` on `/drive_nav`, but the AEB stopped it. Diagnostics identified two
returns at the Hokuyo's extreme `+135 degree` scan edge, transformed to points
inside the car body near `(0.12, 0.15)`. The closest was interpreted as an
obstacle 0.21 m along the steering arc, causing `PASS -> BRAKE -> HOLD`.

The AEB now excludes scan points only inside the known vehicle footprint
(`-0.20 < x < 0.35 m`, `|y| < 0.17 m`). A global minimum-range increase was
not used because it could hide a genuinely close obstacle directly ahead.
After deployment and restart, the AEB returned to `PASS (no command)` and its
Python source passed bytecode compilation.

### Rejected experiment: stop-and-steer local avoidance

An experimental `AVOID` state was subsequently added to the robot-side
`~/rr/rr_wall_aeb.py`. This was requested because the static-map planner's
path passed the mapped column, but Pure Pursuit's instantaneous steering arc
approached it closely enough for the AEB to stop. The dynamic global obstacle
layer remains unsuitable for this run because scan/TF misalignment leaves
phantom marks on its non-rolling costmap.

After a normal AEB brake, the experimental state samples steering arcs from
`-0.32` to `+0.32 rad` in `0.04 rad` increments using only the current raw
LiDAR scan. It chooses the smallest change from the controller request having
at least `0.80 m` of predicted swath clearance, latches that steering angle,
and creeps at no more than `0.25 m/s`. It hands control back only after the
requested controller arc remains clear for 10 consecutive cycles. It stops
if its chosen arc falls below the speed-dependent braking distance or if the
maneuver exceeds 4 seconds. Reverse recovery remains disabled.

This behavior was subsequently tested on the physical race car. It was not
preferable: braking in the middle of the route before selecting and applying a
new heading introduces too many interruptions for racing. The experiment has
therefore been removed from the active AEB implementation, and the robot is
back on the brake-only version with reverse recovery disabled.

Do not restore this experiment as the default race behavior. A future solution
should anticipate static-map clearance continuously in the controller or path
generation, preserving speed and steering continuity instead of reacting with
a stop-and-redirect maneuver.

The restored active brake-only version is also preserved on the robot at:

```text
~/rr/rr_wall_aeb.py.pre_experimental_avoidance_20260803
```

The rejected experimental source is retained only for historical comparison
at:

```text
~/rr/rr_wall_aeb.py.rejected_experimental_avoidance_20260803
```

It is not active. The normal AEB brake is again the only forward collision
intervention.

### Rejected experiment: static-map column buffer

To increase clearance in the wide corridor without reducing the usable width
of the narrow corridor, a separate planning-map variant was created on the
robot:

```text
~/rr_maps/corridor_column_buffer_10cm.yaml
~/rr_maps/corridor_column_buffer_10cm.pgm
```

The active column occupies approximately pixels `x=299..313`, `y=150..163`
in the 5 cm/pixel planning map, near world position `(9.46, 1.14)`. The variant
adds a 2-pixel (10 cm) local occupied buffer around only that object. Exactly
342 image pixels differ from `corridor_despeck.pgm`, all inside the target box
`x=297..315`, `y=148..165`; no narrow-corridor map pixel was changed.

The variant was loaded successfully through `map_clean_server/load_map`, both
Nav2 costmaps were cleared, and `rr_bringup.sh` now selects it for future
starts. Localization still uses the original `corridor_clean` pose graph, so
the change affects global planning only.

This map variant was motion-tested and the car still stopped near the column.
It was therefore judged unsuitable and reverted. The original
`corridor_despeck.yaml` was reloaded successfully into the live map server,
both Nav2 costmaps were cleared, and `rr_bringup.sh` was restored to select the
original map on future starts. The experimental map files remain separate and
inactive only for historical comparison.

The same run exposed a more fundamental controller failure before the AEB
stop. Despite the deployed 3 m search bound, the controller again selected the
last point of newly replanned paths far too early:

```text
x=5.22 y=0.55: nearest=315 progress=315 of a 316-point path
x=6.13 y=0.60: nearest=287 progress=287 of a 288-point path
```

The controller then targeted the mission endpoint `(-0.02, 4.99)` from the
lower straight, so adding static clearance around one column cannot reliably
correct the resulting trajectory. Further work must fix why replans still
permit this end-of-path progress jump before attempting more map shaping.

The pre-experiment bring-up copy is retained at:

```text
~/rr/rr_bringup.sh.pre_column_buffer_20260803
```

The rejected bring-up file is retained at
`~/rr/rr_bringup.sh.rejected_column_buffer_20260803`; it is not active.

### Controller progress/lookahead separation — deployed, not motion-tested

Further inspection found why the earlier 3 m search bound did not fully stop
progress racing. After nearest-point matching, every 20 Hz control cycle also
persisted the temporary lookahead `target_idx` into `last_progress_idx_`. With
a roughly 0.8 m lookahead, persistent progress could therefore advance much
faster than the car's approximately 0.05 m movement per cycle, eventually
reaching the path endpoint while the vehicle was still on the lower straight.

The controller now advances `last_progress_idx_` only from
`find_nearest_forward_index()`. The lookahead index remains available for the
current steering calculation and MPC/debug computations but is no longer
treated as physically reached progress. The existing 3 m bounded nearest and
lookahead searches remain active.

This change compiled locally and on the Jetson. The Jetson build was forced to
recompile after verification detected that an initially preserved source
timestamp had left the previous executable up to date. The final installed
executable timestamp is `2026-08-03 23:01:09 +0200`, and the restarted live
controller reports `max_progress_search_distance: 3.0` and output topic
`/drive_nav`.

**This progress/lookahead separation has not yet been motion-tested.** During
the next supervised run, `progress` should follow `nearest`; `target_idx` may
remain ahead for steering but must no longer pull persistent progress forward.
The pre-change deployed source is backed up at:

```text
~/rr/deploy_backup_20260803/pure_pursuit_controller.before_progress_lookahead_fix.cpp
```

### Clustered LiDAR AEB filtering — deployed, not motion-tested

The first physical run after separating progress from lookahead completed the
route without the previous endpoint-index jump. It nevertheless suffered many
short AEB interruptions, especially during turns. Most triggers reported
extremely close values of `0.10..0.19 m` and cleared immediately after the
brake hold, which is characteristic of isolated scan-edge/chassis glints or
individual wall-edge beams rather than a persistent solid obstacle.

The brake-only AEB now requires an obstacle return to belong to a spatially
continuous run of at least three consecutive original LiDAR beams. Adjacent
beam numbers are joined only when their Cartesian points are no more than
`0.08 m` apart, so invalid scan gaps or depth discontinuities cannot merge
unrelated returns. A normal-distance hazard must also persist for three 20 Hz
control cycles. A clustered obstacle at or below `0.25 m` still requests an
immediate brake without waiting for the persistence counter.

Synthetic checks confirmed that isolated points and two-beam glints are
rejected, a continuous four-beam wall is retained, and a range discontinuity
splits a cluster. The deployed node passes Python bytecode compilation and its
live startup configuration reports:

```text
cluster=3 beams/0.08m confirm=3 immediate=0.25m recovery=False
```

**This clustered-return behavior has not yet been motion-tested.** It changes
only AEB evidence filtering; the protected swath width and speed-dependent
stopping distance are unchanged. The prior brake-only AEB is backed up at:

```text
~/rr/rr_wall_aeb.py.before_cluster_filter_20260803
```

### Minimum obstacle-radius filter — deployed, not motion-tested

Physical testing of the three-beam clustered AEB still produced repeated
false stops. The logs showed size-unqualified clusters at `0.14..0.18 m`
repeatedly entering the immediate-brake path after only one cycle. The
three-beam requirement alone was therefore insufficient because a small close
artifact can cover several adjacent laser beams.

The real-time AEB now estimates each contiguous scan cluster's visible size
from the Cartesian distance between its first and last beam. A cluster must
have an estimated radius of at least `0.10 m`, meaning an endpoint span of at
least `0.20 m`, before it is included in swept-path collision checking. This
size filter applies before both the normal three-cycle confirmation and the
`0.25 m` immediate-brake rule.

This is an intentional behavior tradeoff: **real objects with a visible
cluster radius below 10 cm are also ignored by the AEB.** Walls, columns, and
other larger obstacles must still satisfy the existing three-beam continuity,
0.08 m neighbor-distance, and persistence rules.

Boundary checks confirmed that a 15 cm-wide cluster is rejected while 20 cm
and 30 cm clusters are retained. The deployed node passes Python bytecode
compilation and reports:

```text
cluster=3 beams/0.08m radius>=0.10m confirm=3 immediate=0.25m recovery=False
```

**This minimum-radius filter has not yet been motion-tested and must be tested
with the gamepad override ready before it is considered validated.** The
previous clustered AEB is backed up on the robot at:

```text
~/rr/rr_wall_aeb.py.before_radius_filter_20260804
```

The read-only `roboracer_freeze_2026-08-04_last_night` baseline was not
modified by this post-freeze experiment.

### Fresh odometry-transform verification — deployed, not mission-tested

The waypoint mission occasionally aborted after gyro calibration with a stale
non-zero `odom -> base_link` offset even though the newly restarted
`rr_gyro_odom` had reset its internal pose. Log timestamps exposed a race: the
mission queried TF about 10 ms after the gyro node printed its ready marker,
before the next IMU callback published the new zero transform. The mission TF
buffer could therefore return the killed process's final cached transform.

`rr_waypoint_mission.py` now clears its TF buffer immediately after the gyro
restart, records the post-restart ROS timestamp, and rejects transforms whose
timestamps are not newer than that marker. It requires three distinct fresh
transforms, each within `0.05 m` of zero, during a 5-second post-calibration
window. A single fresh transform above 5 cm is treated as a genuine reset
failure and aborts immediately; missing fresh transforms also abort rather
than weakening the safety check.

Expected successful arming output now includes:

```text
Fresh odom sample 1/3: age=... offset=...m
Fresh odom sample 2/3: age=... offset=...m
Fresh odom sample 3/3: age=... offset=...m
Odom zeroed: odom -> base_link ...m, gyro calibrated
```

The deployed script passes robot-side Python bytecode compilation. **The full
mission sequence with this fresh-TF check has not yet been run and must be
tested while the car is physically still during calibration.** The previous
mission script is backed up at:

```text
~/rr/rr_waypoint_mission.py.before_fresh_tf_verify_20260804
```

The read-only last-night freeze remains unchanged.

Robot-side backups of the replaced controller source, real-controller
configuration, and AEB script are stored in
`~/rr/deploy_backup_20260803/`.

The active robot relay is `plan_qos_relay.py`, a direct QoS relay from Nav2's
plan to `/control/plan`; the legacy oval-generating `path_relay_node.py` is not
running and was therefore not deployed.

## Remaining supervised test

Before the next physical run:

1. Keep the vehicle raised or reduce the speed cap for the initial mission;
2. test with the gamepad emergency override
   ready;
3. monitor controller logs and confirm `progress` advances locally instead of
   jumping to the final index;
4. compare `/nav2/plan_raw` and `/control/plan` in Foxglove to confirm that the
   relay preserves the Nav2 path.

## 2026-08-04 turn-stop investigation: stable waypoint plan and trace

The latest physical waypoint runs exposed a turn-specific deadlock that was
much less common when the same lap was commanded as separate Foxglove goals.
The correlated existing logs show two mechanisms, not a simple controller
failure:

- the stock Humble `NavigateThroughPoses` behavior tree replaced the path
  every three seconds;
- during a close turn, the global planner repeatedly reported `Starting point
  in lethal space`, canceled the valid controller path, and invoked
  `Spin`, `Wait`, and `BackUp` recoveries;
- those holonomic recovery motions are unsuitable for this Ackermann car;
- at the same positions, the independent AEB correctly remained authoritative
  and repeatedly entered `BRAKE/HOLD` for wall/column clusters at roughly
  `0.14..0.18 m`. One hold lasted about nine seconds. The recovery tree and
  AEB could therefore wait on each other indefinitely;
- the controller log still commanded `0.95..1.00 m/s` while measured speed was
  zero, confirming the controller itself was requesting motion during many of
  the stops.

The mission now supplies `~/rr/rr_waypoint_no_replan.xml`. It computes the
complete saved-map route once, then follows that stable path without periodic
replacement and without `Spin/Wait/BackUp`. This is mission-scoped: normal
Foxglove navigation is unchanged. A planning/controller failure now aborts and
stops instead of attempting race-car-inappropriate recovery. The brake-only
`rr_wall_aeb` remains enabled and unchanged.

The mission also automatically starts the read-only recorder
`~/rr/rr_mission_trace.py`. Every run creates
`~/rr_logs/waypoint_trace_YYYYMMDD_HHMMSS.csv` at 20 Hz, correlating map pose,
odometry speed, controller command, post-AEB command, AEB state, three LiDAR
sectors, and every received path (sequence, size, start, and end). It stops
with the mission. This should distinguish an AEB stop, lost/empty plan,
controller zero command, pose jump, and physical failure from one timestamped
file.

Robot-side rollback:

```text
~/rr/rr_waypoint_mission.py.before_stable_plan_trace_20260804
```

The Python files compile and the behavior-tree XML parses. **This stable-plan
change and recorder have not yet been physically tested. Test with the gamepad
override ready.** The read-only last-night freeze was not modified.

### First stable-plan run and all-return AEB change

Trace `~/rr_logs/waypoint_trace_20260804_172546.csv` confirms that the new
mission computed one 352-pose path and did not periodically replace it. The
earlier replanning failure was therefore removed. During this run AEB made a
short stop near `(4.37, 0.44)`, then later braked near `(8.88, 0.79)` with a
reported swept-path obstacle distance of `0.44 m`. It held zero output for
about 4.7 seconds while the controller continued requesting `1.0 m/s`. The
negative odometry speed near the end occurred while AEB still output zero and
was therefore operator/mux motion, followed by mission cancellation.

No wall/opponent aspect-ratio classifier was running. The live AEB consumes
raw `/scan` directly from `urg_node`. However, the previously added radius
filter was itself a spatial classifier: it discarded clusters with visible
diameter below 20 cm, and the three-beam rule also discarded one/two-beam
returns. This can hide fan legs, narrow column edges, and partially occluded
obstacles, contradicting the requirement that safety see every obstacle.

The active AEB now includes every finite LiDAR return outside the explicit car
chassis footprint in swept-path collision checking:

```text
cluster=1 beams/0.08m radius>=0.00m confirm=3 immediate=0.25m
```

The normal three-cycle temporal confirmation remains, and obstacles at or
inside 0.25 m still brake immediately. This deliberately reverts the unhelpful
10 cm radius experiment. Robot-side rollback is:

```text
~/rr/rr_wall_aeb.py.before_all_returns_20260804
```

The updated process is live as `rr_wall_aeb` and the startup line above was
verified. **The all-return behavior has not yet been physically tested and may
restore some false stops; test with the gamepad override ready.**

### Physical odometry and steering calibration recheck

Repeated manual traversals between rigid endpoints exactly `3.000 m` apart
showed that the frozen `speed_to_erpm_gain=3690` over-reported distance. An
initial correction to `4330` reduced the error; twelve post-change traversals
then had median `3.183 m` (forward/reverse averages differed by only 0.7%).
The refined calibration is therefore:

```text
speed_to_erpm_gain = 4330 * 3.183 / 3.000 = 4594 -> deployed 4595
```

`4595` is enforced in `~/rr/rr_bringup.sh` and both source/installed
`vesc.yaml` copies. The stack was cold-recycled after deployment. Backups with
suffixes `before_erpm4330_20260804` and `before_erpm4595_20260804` remain on
the robot. **A final exact 3 m validation at 4595 is still required.**

The same nominally straight traversals physically changed heading by roughly
5–11 degrees, reversing sign when the direction reversed. This is repeatable
physical steering-centre bias, not gyro noise. The live servo offset remains
`0.499`; it was deliberately not changed until distance calibration is
validated and steering direction is calibrated separately.

### Final odometry validation, frozen working state, and scan-motion tool

Eight subsequent exact 3.000 m runs with `speed_to_erpm_gain=4595` measured
2.993, 2.945, 2.894, 3.053, 3.106, 3.105, 3.043, and 3.044 m straight-line
distance. Their mean was 3.023 m (+0.76%); mean integrated path was 3.036 m
(+1.2%). This validates 4595 for the current drivetrain. A new immutable copy
of the working car, runtime state, logs, workspaces, maps, and WSL repository
is stored at:

```text
/home/sadegh/roboracer_freeze_2026-08-04_working_gain4595
```

At that freeze and at the end of this session, AEB was off, the controller
published directly to `/drive`, the steering offset was 0.499, and the global
costmap obstacle layer was disabled at runtime. The older last-night freeze
also remains untouched.

Because scan alignment looked wrong specifically while moving, the read-only
tool `~/rr/rr_scan_motion_calibrate.py` was added. It scores `/scan` endpoints
against the saved map using the transform at each scan's own timestamp, sweeps
residual LiDAR yaw, separates parked/moving samples, and records scan age, TF
lookup failures, and `map->odom` jumps. It does not publish commands or modify
calibration. Its first parked smoke test preferred 0.0 degrees residual yaw,
so there is currently no evidence for changing the static LiDAR mounting
transform; moving tests are still required.

The tool now gives explicit inline operator feedback. Every run begins with a
five-second `keep the car STILL` countdown and prints `>>> GO NOW <<<`. At 1 Hz
it reports moving/still state and remaining time. The `straight_3m` task shows
odometry path while instructing the operator to stop at the physical 3.000 m
marker; `turns_90` shows accumulated odometry rotation while deferring to the
physical 90-degree marker; `full_lap` shows path and closure while instructing
the operator to stop at the starting marker. These readings are feedback, not
automatic stopping criteria. The previous script is preserved as:

```text
~/rr/rr_scan_motion_calibrate.py.before_inline_feedback_20260804
```

**The moving scan-calibration tasks and the new inline feedback have not yet
been physically tested.**

### Scan-motion test results

Three straight runs, two 90-degree runs, and one lap run were recorded. Scan
age stayed approximately 0.18--0.23 s, so gross scan transport delay was not
the dominant error. Parked residual-yaw estimates varied by run/location from
-10 to +1.5 degrees, while moving estimates varied from -10 to 0 degrees. The
improvement obtained by the yaw sweep was only about 0.01--0.03 m, whereas
raw scan-to-map fit error sometimes reached 0.4--0.5 m. Therefore these tests
do **not** justify changing the static `base_link->laser` transform.

Every motion run instead showed large `map->odom` corrections. Worst observed
translation steps were 2.22--5.19 m, with yaw steps up to 9.6 degrees. The lap
contained 20 threshold-exceeding corrections. At the same time,
`slam_toolbox.log` continuously reported its laser message-filter queue as
full, and Ceres repeatedly reported a requested 50 threads being bounded to
the Jetson's six available threads. The leading diagnosis is overloaded or
unstable SLAM localization/scan matching while moving, not a fixed LiDAR
mounting-angle error.

The first inline-feedback implementation incorrectly derived path and heading
from the `/odom` message pose. On this car that message supplies wheel speed
but does not contain the fused gyro heading, causing the 90-degree display to
remain at 0 and lap closure to equal path length. Feedback was corrected to
integrate the live fused `odom->base_link` transform at 5 Hz. The scan/map CSV
measurements from the completed tests remain valid because they already used
timestamped TF. A transient Humble Python DDS conversion exception is now
reported cleanly while preserving the collected report rather than printing a
traceback. Rollback for this feedback correction is:

```text
~/rr/rr_scan_motion_calibrate.py.before_fused_tf_feedback_20260804
```

**The corrected 90-degree/lap feedback has not yet been physically tested.**

Three subsequent physical 90-degree tests validated the corrected fused-TF
feedback: it reported 91.8, 93.1, and 93.3 degrees. The parked residual-yaw
estimate still changed from -5.75 to -4.25 to +0.75 degrees, while the yaw
sweep improved mean map fit by only 0.001--0.010 m. This confirms that no
single static LiDAR yaw correction is supported by the data. All three runs
still had `map->odom` steps around 0.72--0.76 m.

A live resource check found `/scan` arriving at 39.9 Hz. The six-core Jetson
had load average 12.3; approximate CPU consumers while parked included
Foxglove bridge 69%, localization SLAM 54%, `rr_gyro_odom` 49%, and
`map_odom_relay` 38%. The active SLAM setting `minimum_time_interval: 0.1`
only needs at most 10 Hz, but `throttle_scans: 1` admits the full 40 Hz into
the transform/message-filter path, which is continuously reporting a full
queue. A localization-only 4:1 scan throttle is the recommended next
controlled experiment; it has **not** been applied yet.

### Bringup persistence for calibrated speed and SLAM frequency

`~/rr/rr_bringup.sh` now explicitly enforces and reports all relevant speed
conversion and localization-frequency values on every startup:

```text
speed_to_erpm_gain   = 4595.0
speed_to_erpm_offset = 0.0
steering offset      = 0.499
physical /scan       = approximately 40 Hz (unchanged)
throttle_scans       = 4
SLAM target rate     = approximately 10 Hz
minimum_time_interval = 0.1 s
```

The throttle and minimum interval are rewritten into
`~/rr/localize_slam_real.yaml` before localization starts. Bringup also treats
that YAML becoming newer than the running localization process as a reason to
restart SLAM, ensuring config edits are not silently ignored. The status block
prints both calibration and effective localization frequency. Robot backups:

```text
~/rr/rr_bringup.sh.before_slam10hz_erpm_enforce_20260804
~/rr/localize_slam_real.yaml.before_slam10hz_20260804
```

The deployed YAML now contains `throttle_scans: 4`, but the SLAM process that
was already running still reported its old in-memory value of 1. It must be
restarted (which resets the current localization pose) before the 10 Hz
experiment is active. Live VESC odometry gain remained 4595.0. **The 10 Hz
localization setting has not yet been physically tested.**

### Foxglove scan presentation rate

Bringup now starts a presentation-only scan decimator named
`rr_scan_foxglove_10hz`. It republishes one of every four physical scans:

```text
/scan            approximately 40 Hz (unchanged on the robot)
/scan_foxglove   approximately 10 Hz (Foxglove/report visualization)
```

The first version excluded raw `/scan` from Foxglove. This was reverted at the
operator's request: Foxglove now exposes both `/scan` and `/scan_foxglove`, and
the operator can select or disable either in the layout. Live measurement of
the decimated topic after activation was 9.997 Hz. This is persisted in
`~/rr/rr_bringup.sh`; backups are
`~/rr/rr_bringup.sh.before_foxglove_scan10hz_20260804` and
`~/rr/rr_bringup.sh.before_restore_raw_scan_20260804`.

### Confirmed-working immutable freeze

After the operator confirmed this state was working, a new independent freeze
was captured at:

```text
/home/sadegh/roboracer_freeze_2026-08-04_working_foxglove_scan10hz
```

It includes all robot scripts, maps, logs, complete robot workspaces, the WSL
repository, live runtime/process/ROS parameter evidence, and SHA-256 checksums.
The captured live state verifies ERPM gain 4595.0, offset 0.0, SLAM throttle 4,
SLAM minimum interval 0.1 s, global obstacle layer disabled, direct `/drive`
controller output (AEB off), and both raw 40 Hz `/scan` and decimated 10 Hz
`/scan_foxglove` exposed through Foxglove. The two older freezes remain
untouched.
