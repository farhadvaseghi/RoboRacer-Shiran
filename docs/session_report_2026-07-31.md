# RoboRacer Session Report — 2026-07-31

**Robot:** F1TENTH on Jetson Orin Nano, ROS 2 Humble, `ROS_DOMAIN_ID=7`
**Access:** `roboracer@192.168.50.10` over the `roboracer` Wi-Fi
**Foxglove:** `ws://192.168.50.10:8765`

## Goal

Get the waypoint mission driving reliably: diagnose why goals were being
rejected, why the car clipped a wall, and why localization kept wandering.

## Headline outcome

- **Root cause found and proven: the LiDAR sees the car itself.** A persistent
  cluster of returns at +132°..+135° (and a second at −135°..−124°), 9–23 cm
  from `base_link`, present in every scan. It marks a phantom obstacle on top of
  the robot, which made the start footprint lethal and refused every goal.
- Worked around with `obstacle_min_range` / `raytrace_min_range` on both
  costmaps. **The hardware fault itself is NOT fixed.**
- `minimum_turning_radius` corrected 0.40 → 0.98 (physical R_min ≈ 0.754; the
  planner had been emitting arcs the car cannot follow).
- Controller reverse disabled (`reverse_trigger_x: -1000.0`).
- Corridor **re-mapped**; new map cleaned and in service.
- Localization quality quantified for the first time: the corridor is
  **geometrically degenerate along its axis**.
- Waypoint mission gained: wall-clearance control, drive-speed option, slam
  `max_laser_range` option, and route duration reporting.

---

## 1. The LiDAR is looking at the car (root cause)

Measured directly from `/scan`, at two completely different car positions:

```
persistent close bearings (deg:count over 40 frames): {132: 95, 133: 160, 134: 160, 135: 80}
ranges 0.088 .. 0.232 m,  range_min param = 0.020 m  -> nothing filters them
```

The returns follow the car, so they are not a wall. Later scans also showed a
second cluster at −135°..−124°, i.e. **both rear corners**. Beam count is
unstable across the session (12 → 60 → 6 → 18), which suggests something loose
rather than fixed geometry.

Consequences, all confirmed by measurement:

1. **Every goal refused.** The obstacle layer marked the phantom ~9 cm from the
   robot; inflation made the robot's own cell cost 99 (inscribed). No
   `robot_radius` escapes an obstacle marked on top of the robot — 0.10 was
   exactly as lethal as 0.22.
2. **This is why disabling the global obstacle layer "fixed" planning** on
   07-16/07-28 and why re-enabling it on 07-30 broke every goal. That was never
   a tuning win; it was masking this.
3. **It biases slam.** Localization settled ~10 cm behind ground truth
   (measured: hand-measured rear wall 0.31 m vs slam's belief 0.200 m).

### Workaround applied

`nav2_params_real.yaml`, both local and global costmaps:

```yaml
raytrace_min_range: 0.1
obstacle_min_range: 0.1
```

At 0.2 the robot's own cell went from cost 99 to **0** and a plan succeeded with
the obstacle layer ENABLED for the first time all day. Operator reported 0.2
behaved worse in driving, so it was reduced to 0.1 (still plans).

Note `raytrace_min_range` also disables *clearing* inside that radius, so the
two parameters may want different values. Untested.

### Real fix (NOT done)

Physically remove whatever the sensor sees at the rear corners, or clip the FOV:

```bash
sed -i 's/angle_max: 3.14/angle_max: 2.26/' \
  ~/f1tenth_ws/{src/f1tenth_system/f1tenth_stack,install/f1tenth_stack/share/f1tenth_stack,build/f1tenth_stack}/config/sensors.yaml
```

then restart the base stack. Costs 5.5° of rear-left view. A live `ros2 param
set` on `urg_node` is accepted but ignored — the angle limits only apply when
the device is reconfigured at startup.

---

## 2. Turning radius was physically impossible

`minimum_turning_radius` was **0.40 m**. The car's real minimum is
`wheelbase / tan(max_steer) = 0.25 / tan(0.32) = 0.754 m`. Measured curvature of
the planner's own output confirmed it was emitting genuine 0.400 m arcs — which
the car cannot follow, so steering saturates and it drifts wide on turn exits.
Combined with reduced wall margin, that produced the wall contact.

Now **0.98**, verified by measuring planned-path curvature (`min_path_radius =
0.980 m`), and both route legs still plan.

**A live `ros2 param set` on this does not work.** It returns success and reads
back the new value while the planner keeps using the old one — SMAC builds its
state space at configure time. It must be changed in YAML with a nav2 restart.
This silent-failure pattern cost significant time; see §7.

**DUBIN is not viable here.** Switching `motion_model_for_search` to DUBIN
returned NO PATH on this route at 0.40 — forward-only cannot solve it. Raising
the radius makes Dubins need *more* room, so it stays off the table.

---

## 3. Reverse disabled

The car was making unwanted reverse manoeuvres. Two independent sources:

- **The planner** (REEDS_SHEPP) is allowed to plan reverse cusps. Measured: the
  route to Goal 1 contained **zero** cusps, so this was not the cause.
- **The controller** flips itself into reverse when the target falls behind it.
  This was the cause, and it was already known: a 07-30 comment records reverse
  "pinning the steering and spiralling the odometry". That fix had been reverted.

Re-applied in `~/rr/controller_params_real.yaml`:

```yaml
reverse_trigger_x: -1000.0
```

**It cannot be scoped to one mission.** The controller calls
`read_parameters()` once in its constructor and registers no parameter
callback, so nothing can change it at runtime. It applies to all driving now.

**Live mismatch to be aware of:** the planner may still emit a cusp on some
route while the controller can no longer execute one. Neither current leg has
one, but a cusp check before driving would turn that into a clean abort.

---

## 4. Map vs. world, and the re-map

The map in use was from 2026-07-14. A door that is open in it is now closed.
Classifying every beam against the map:

```
                     old map (Jul 14)   new map (re-mapped today)
MATCH                    59%                 56%
EXTRA (real, not mapped) 10%                 15%
MISSING (mapped, not real) 31%               29%
```

**Re-mapping did not improve agreement.** The door was real and did show up
(a 0.77 m wide surface at x ≈ 2.85 in the old frame, laser stopping 0.2–0.3 m
short of prediction) but accounted for only 8 of 226 beams.

The dominant residual EXTRA in the new map is the **sensor self-return** — the
despeckler strips it out of the map while the sensor keeps producing it, giving
a permanent ~8% mismatch that no map can fix.

The MISSING clusters (a 2.9 m long thin structure) are most likely **transient
objects mapped during the lap** — probably the operator walking beside the car.
Next mapping run: stay out of the sensor's view.

### Mapping procedure notes (traps)

- `rr_slam.sh` only kills `async_slam_toolbox_node`. The localizer is
  `localization_slam_toolbox_node` — a different name — so it survives and you
  end up with two nodes publishing `map->odom`. Stop it manually.
- `rr_slam.sh` has no `/map` remap, so it collides with `map_clean_server`.
  Stop the map server too.
- **`rr_slam.sh`'s "the car's pose at launch becomes the map origin" only holds
  if odometry is at zero.** The first attempt today anchored the map 43 m away
  with 170° of yaw error because the base stack had been running and had
  accumulated odometry. **Restart the base stack immediately before mapping.**
- slam_toolbox's `save_map` service failed (`result=255`); the occupancy grid
  had to be saved with `ros2 run nav2_map_server map_saver_cli` instead. The
  posegraph save (`serialize_map`) worked.

### Cleaning the new map

`rr_despeckle.py` (blobs ≤20 px) left 21 components. Filtering the rest by
**shape** — drop if the smaller dimension < 0.25 m or size < 25 px — removed 6
blobs / 174 px: five thin streaks (10–20 cm wide, up to 1.9 m long, consistent
with a self-return trail along the drive path) and one isolated 21 px fragment.

**Do not filter by distance-to-nearest-wall.** That was tried first and deleted
the corridor's **columns** (0.55–0.75 m squares, 40–101 px) along with the
noise, because the wall components sprawl 25 m so everything is near one.
Shape is the correct discriminator: columns are compact in both axes,
artefacts collapse in one.

The columns are at (9.45, 1.19), (16.35, 6.46), (16.62, 1.66), (1.90, 5.76),
(2.10, 0.93), (23.49, 6.63) in the new map frame — and they are useful, see §5.

---

## 5. Localization: the corridor is degenerate

Monitoring scan-vs-map while the car was moved:

```
x:  8.80 -> 7.99 -> 8.88 -> 9.75 -> 10.68 -> 9.45 -> 10.30
y: -1.06   -0.99   -1.08   -1.13   -1.15    -1.02   -1.22
```

**x wanders over 3.4 m while y stays within 0.34 m.** Two long parallel walls
give the scan matcher lateral position and heading but almost nothing about
position *along* the corridor, so it slides freely on that axis. No parameter
fixes this; it is map geometry.

Practical consequence: lateral position (staying off the walls) is reliable;
along-corridor position (knowing you have arrived) is not. That directly
undermines waypoint missions whose legs are ~12 m apart in x.

The **columns** in the new map are the first real features that break this
degeneracy. Whether they are enough is untested.

### Metric note

`agree%` (fraction of beam endpoints landing on an occupied cell) is harsh at
5 cm resolution. A pose off by one cell scores ~29% while the median range
error is only 0.09 m. **Use the median range error**, not the hit fraction.

---

## 6. Initial pose

`localize_slam_real.yaml` has `map_start_at_dock: true`, so every cold boot
initialises at the map origin. On the OLD map the origin was **0.071 m from a
wall** — inside the robot footprint — so every cold start began in a lethal
pose. This is a large part of the "Starting point in lethal space" history.

`~/rr/rr_seed_start.py` was added: publishes a surveyed pose on `/initialpose`
with TRANSIENT_LOCAL + RELIABLE (slam drops volatile). Verified: slam accepts it
and then holds it with **zero drift over 70 s**.

**The wandering pose was not slam being unstable.** The only publisher on
`/initialpose` is `foxglove_bridge` — i.e. manual 2D Pose Estimate clicks. Four
in 25 seconds were logged, each resetting the pose graph before it could settle.

**`rr_seed_start.py`'s coordinates are for the OLD map and are now stale.**
Re-survey after adopting the new map.

---

## 7. Three parameters that accept changes and ignore them

A recurring trap that cost hours. In each case `set_parameters` returns success
and the value reads back, but behaviour does not change:

| Parameter | Node | Why |
|---|---|---|
| `GridBased.minimum_turning_radius` | planner_server | SMAC builds its state space at configure time |
| `reverse_trigger_x` (and all controller params) | pure_pursuit_controller | `read_parameters()` runs once in the constructor |
| `max_laser_range` | slam_toolbox | laser metadata cached per frame on first registration |
| `angle_max` | urg_node | device reconfigured only at startup |

**Always verify a parameter change by measuring behaviour**, not by reading the
value back. For the planner this means measuring the curvature of a generated
path; for the costmap, the cost under the robot.

---

## 8. Bring-up / teardown fixes

- **`rr_bringup.sh` now reloads maps that changed.** `map_server` and
  `slam_toolbox` read their map files once at startup; `spawn` reported
  "already up" and kept serving a stale map. Today `map_server` served a map
  written two minutes before it started, which looked exactly like the edits
  not working. Bringup now compares map file mtimes against process start time
  and restarts the holder. Note the slam restart discards the pose — re-seed.
- **`rr_teardown.sh` misses `joy_teleop` and `static_transform_publisher`.**
  This is the `remaining_ros=1` straggler, and it is how three duplicate
  `base_link→laser` publishers accumulated and tore the TF tree. NOT yet fixed.
- **`rr_autokeep` starts AFTER nav2 in bringup.** `vesc_to_odom` only emits
  `odom→base_link` while drive commands flow, and nav2's costmaps block waiting
  on that TF — so bringup can wait on something scheduled to start later. Move
  it before nav2. NOT yet fixed.
- **The RTC is dead.** Every power cycle returns to 1970. Setting the clock
  while a stack is running poisons every node's TF buffer (56-year jump);
  always tear down first, then set the clock, then bring up.

---

## 9. Files changed on the car

```
~/rr/controller_params_real.yaml        reverse_trigger_x: -1000.0
~/rr/rr_bringup.sh                      map staleness reload (§8)
~/rr/rr_seed_start.py                   NEW - surveyed start pose seeder
~/rr/rr_waypoint_mission.py             wall clearance, speed, max_laser_range, duration
~/roboracer_ws/src/RoboRacer-Shiran/roboracer_estimation/config/nav2_params_real.yaml
                                        minimum_turning_radius 0.40 -> 0.98
                                        obstacle_min_range / raytrace_min_range 0.1
~/rr_maps/corridor_clean.*              NEW map (grid + posegraph)
~/rr_maps/corridor_despeck.*            despeckled + shape-filtered
~/rr_maps/backup_20260731_2100/         pre-mapping originals
```

Every edit has a `.bak_*` backup alongside it.

### Waypoint mission additions

Prompts, in order: obstacle layer → clear interval → **drive speed** → **slam
max_laser_range** → START; and it now reports `ROUTE DURATION` on success.

- **Wall clearance:** widens the global inflation for the run (default 0.30 m,
  sized for the measured 1.00 m pinch) and restores it on exit. Inflation is
  used rather than `robot_radius` deliberately: measured, raising inflation
  0.25→0.40 grew the inflated area 49% while the lethal cell count moved by 4
  cells, so it cannot cause the lethal-start failure a bigger footprint would.
- **Drive speed:** rewrites the controller YAML, restarts the controller,
  restores on exit. Also raises `max_lookahead_distance` to keep
  `speed × lookahead_time` — without this the preview time shrinks as speed
  rises and the car weaves.
- **slam max_laser_range:** rewrites the slam YAML, restarts slam, re-seeds the
  surveyed pose, restores on exit. Only valid with the car at the start.

---

## 10. Still open

1. **The LiDAR obstruction** — the single highest-leverage fix. Everything else
   here is a workaround for it.
2. **Nothing is committed.** The repo still ships `robot_radius: 0.22`,
   `inflation 0.25` and the global `obstacle_layer` — the exact config that
   produced lethal starts on 07-30 — plus `minimum_turning_radius: 0.40`. A
   redeploy re-breaks the car. The on-car scripts (`rr_bringup.sh`,
   `rr_teardown.sh`, `kill_base.sh`, `seed_origin.py`, `rr_seed_start.py`,
   `rr_autokeep.py`, `plan_qos_relay.py`, `rr_despeckle.py`) exist only on the
   car despite being referenced throughout the docs.
3. **Waypoints need re-recording** against the new map frame.
4. **`rr_seed_start.py` coordinates are stale** (old map frame).
5. **Teardown and autokeep ordering** (§8) not yet fixed.
6. **Speed is 0.5 m/s**; the repo's 3.7 is sim-tuned and never hardware-validated.

## 11. Diagnostic errors made this session (recorded so they are not repeated)

- **Shared-memory poisoning was called early on bad evidence.** The probe that
  "hung" had a shell-quoting bug; DDS was healthy. Verify the tool before
  blaming the system.
- **"Max cost within `robot_radius`" was used as a blocked-start test.** That
  double-counts: nav2 already inflates by the inscribed radius, so for a
  circular footprint only the centre cell matters. This made margins look
  tighter than they were.
- **Incremental restarts were used instead of a cold cycle.** Three successive
  nav2/controller restarts left the stack with a stuck lifecycle and a torn TF
  tree. One of them also restarted the base stack as a side effect.
- **The first map cleaning deleted the columns** by filtering on
  distance-to-wall instead of shape.
- **A/B parameter tests were run against a drifting pose** and produced
  physically impossible results (0.22 planned, 0.18 did not). Do not tune until
  localization is stable.
