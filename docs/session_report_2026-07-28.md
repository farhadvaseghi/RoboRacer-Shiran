# RoboRacer Session Report — 2026-07-28

**Robot:** F1TENTH on Jetson Orin Nano, ROS 2 Humble, `ROS_DOMAIN_ID=7`  
**Access:** `roboracer@192.168.50.10` over the `roboracer` Wi-Fi  
**Foxglove:** `ws://192.168.50.10:8765`

## Goal

Bring up the physical car using the shared `Hardware` branch, inspect the saved
maps in Foxglove, initialize localization, send goals, and verify autonomous
movement with the custom Pure Pursuit stack.

## Headline outcome

- Hardware pre-flight passed for LiDAR, VESC, odometry and the base TF chain.
- Existing maps were found and `corridor_despeck` was displayed in Foxglove.
- The integrated `rr_bringup.sh` stack started successfully from a clean state.
- slam_toolbox localization was seeded correctly on ROS domain 7.
- Foxglove goals on `/goal_pose` reached Nav2.
- Initial goals failed because the robot's starting point was classified as
  lethal by the costmap.
- After the start/costmap condition was corrected, the robot moved autonomously
  and reached a goal.
- A delayed-start behavior remains important: sometimes manual movement changes
  the start/localization state enough that a later planning attempt succeeds.

---

## Work completed

### Hardware deployment and pre-flight

The prepared real-car files from the repository were copied to the car and
`roboracer_estimation` was built successfully. The expected
`cmd_vel_to_ackermann` executable and installed Nav2 parameter file were both
present.

The car clock was corrected before ROS was started. The base stack then brought
up the joystick, mux, VESC, LiDAR, odometry and static laser transform.

The first health check showed false failures even though the hardware drivers
were alive. We found two causes:

1. The ROS daemon had stale discovery state after the clock correction.
2. The base stack had started on the default ROS domain, while
   `rr_healthcheck.sh` forces `ROS_DOMAIN_ID=7`.

After restarting the base stack on domain 7, a brief joystick input initialized
the VESC servo feedback required by odometry. The final section 3 checks passed:

```text
[B1]
PASS  VESC symlink
PASS  /scan publishing
PASS  /odom publishing

[B2]
PASS  odom -> base_link
PASS  base_link -> laser
```

### Existing maps

The following map pairs were present in `~/rr_maps/`:

```text
corridor_clean.pgm / corridor_clean.yaml
corridor_despeck.pgm / corridor_despeck.yaml
reglungstechnik_corridor_2.pgm / reglungstechnik_corridor_2.yaml
```

`corridor_clean` also had its `.data` and `.posegraph` SLAM state. The clean
`corridor_despeck` map was selected for display and planning. It is a 605 × 437
grid at 0.05 m/cell.

### Foxglove visualization

The Foxglove bridge started on port 8765 and the desktop app connected
successfully. At first the clean map was not being published, so the documented
`map_clean_server` and its lifecycle manager were started. The server loaded and
activated `corridor_despeck` correctly.

In Foxglove, the map became visible after enabling `/map` in the selected 3D
panel's settings. One UI lesson was that the global **Topics** sidebar is not
where display visibility is configured; the controls are under the 3D panel's
**Panel → Topics** section.

The live `/scan` was initially invisible because Foxglove tried to color it by
`intensity`, but this Hokuyo scan has no useful intensity array. Changing the
scan to a flat red color with a larger point size made it clearly visible.

Before re-seeding, parts of the scan appeared offset from the map. After the
clean bring-up and correct seed, the red scan followed the black wall boundaries
much better. No large global ~7° rotation was obvious in the final screenshot,
although the localization concern recorded on 2026-07-23 should still be kept
in mind.

### Clean teardown and integrated bring-up

We did not run `rr_bringup.sh` on top of the manually started nodes. First,
`rr_teardown.sh` was used to stop the old stack and clear FastDDS shared memory.

The first teardown reported two ROS processes remaining. Inspection found
orphaned `joy_teleop` and `static_transform_publisher` processes, plus stale ROS
daemons on domains 0 and 7. After stopping those leftovers and running teardown
again, the result was clean:

```text
TEARDOWN_DONE remaining_ros=0 shm_fastrtps=0
```

`rr_bringup.sh` was then run with the gamepad already connected. It reported all
expected components UP:

```text
base/VESC
joystick
foxglove
slam-loc
map_clean
nav2
planner
plan_relay
odom_bridge
CUSTOM_CTRL
auto_keeper
costmap_reset
```

The base launch's original `joy_teleop` exited during the joystick-fix handoff,
but `rr_fix_joy.sh` successfully started the fixed replacement using
`joy_teleop_fixed.yaml`.

### Localization seed

Immediately after bring-up, `map_odom_relay` had no `map <- base_link`
transform. The first seed attempt did nothing because it was published from a
fresh SSH shell on the default ROS domain.

Publishing the seed explicitly on domain 7 worked:

```bash
ROS_DOMAIN_ID=7 python3 ~/rr/seed_origin.py
```

slam_toolbox confirmed:

```text
LocalizePoseCallback: Localizing to: (0.00 0.00), theta=0.00
```

The costmap-reset helper also received `/initialpose`, cleared the costmaps,
cancelled stale goals and emptied old paths.

### Goal publishing from Foxglove

Foxglove's default 2D Pose topic was `/move_base_simple/goal`, but this stack
uses `/goal_pose`. The 2D Pose publish topic was changed to `/goal_pose`, while
2D Pose Estimate remained on `/initialpose`.

Bridge and Nav2 logs confirmed that Foxglove successfully delivered the goals.
The lack of motion was therefore not a Foxglove publishing problem.

### Why the first goals did not move the car

Nav2 received the goals but repeatedly rejected the plan:

```text
GridBased: failed to create plan, invalid use: Starting point in lethal space!
Cannot create feasible plan.
Goal failed
```

Since Nav2 produced no `/plan`, the plan relay and Pure Pursuit controller had
nothing to follow, and the car correctly stayed still.

After the start/costmap condition was corrected, the robot planned, moved and
reached the goal successfully.

The user also observed that autonomy sometimes did not begin immediately. The
car had to be moved manually and more than one goal was sent before it started
driving automatically. The session reports and live logs point to two related
effects:

1. Manual movement can move the robot out of a lethal starting cell.
2. Movement can trigger a fresh slam_toolbox localization update.

Repeated goals do not “activate” autonomy; they simply cause new planning
attempts after the robot's start/localization state has changed. The better
procedure is to send one goal, inspect `nav.log` if no plan appears, and fix the
start condition instead of repeatedly publishing goals.

The validated costmap state from the 2026-07-16 report remains important:

```text
robot_radius: 0.10
global costmap plugins: static_layer + inflation_layer
global obstacle_layer: removed
```

### Final driving observation

After the startup and planning problems were resolved, the robot followed the
planned path to the goal almost perfectly. No meaningful lateral offset was
visible during the successful run.

A separate planning problem was observed when a goal was placed in a dark area
of the Foxglove costmap. That location appeared ineligible or unreachable, but
the planner still produced a path and the robot moved into the dark region. This
is undesirable and must be fixed before relying on the stack around unknown or
invalid map space.

The exact cause was not proven during this session. The Nav2 startup log said
that the planner allowed unknown traversal, so the next investigation should
check at least:

- `allow_unknown` / unknown-space handling in the SMAC planner;
- whether the dark Foxglove cells represent unknown, occupied or inflated cost;
- global costmap `track_unknown_space` and static-layer settings;
- goal validation before a goal is accepted;
- whether the footprint and planned path remain inside known free space.

For the next controlled test, several goals should be tried from the laptop so
the team can evaluate path generation and tracking from different start/goal
combinations. These should be **sequential test goals**: send one goal, observe
its complete plan and motion, stop or let it finish, then send the next goal.
Do not rapidly publish several goals or preempt an unresolved failure, because
that can hide the real planner error. Start with goals in clearly free space and
test goals near dark/unknown regions only under controlled conditions with the
joystick override ready.

---

## Important SSH/Wi-Fi recovery learned this session

When this command times out:

```bash
ssh roboracer@192.168.50.10
```

the problem occurs before password authentication. Check both sides of the
wireless connection.

1. Confirm the **laptop** is connected to the `roboracer` Wi-Fi and has a
   `192.168.50.x` address.
2. Confirm the robot's Wi-Fi receiver is powered. Its **blue light should blink
   with a delay/steady interval**.
3. If the laptop and receiver appear correct but SSH still times out, connect
   the robot directly to a monitor using **DisplayPort**, plus a mouse and
   keyboard.
4. On the robot's Ubuntu desktop, check which Wi-Fi network the Jetson itself is
   connected to.
5. If it is connected to anything other than `roboracer`, disconnect that
   network and connect the robot to `roboracer`.
6. Retry SSH from the laptop.

This direct monitor check was the decisive recovery step. Looking only at the
laptop is insufficient: both the laptop and robot must actually be associated
with the `roboracer` network.

USB phone tethering can remain connected for internet. It normally does not
block the on-link `192.168.50.x` Wi-Fi route, but it can be temporarily disabled
when diagnosing unusual routing behavior.

---

## Key lessons

- **Do not repeat every section of `guide.md` for a normal session.** Once the
  software has already been deployed/built and a usable map exists,
  `~/rr/rr_bringup.sh` is the newer integrated replacement for manually starting
  the base stack, Foxglove, clean map server, slam localization, Nav2 planner,
  relays, custom controller, odometry keeper and reset helper in separate
  terminals. After a clean teardown, run the one-shot bring-up, seed localization
  on domain 7, verify Foxglove alignment, and then test a goal.
- `rr_bringup.sh` does **not** replace every possible guide task. The one-time
  deploy/build steps are still required after relevant code changes, and the
  mapping section is still required when a new map must be created. The guide
  remains the reference for architecture, safety, logging and troubleshooting;
  the integrated script is the preferred normal-session startup path.
- Use `ROS_DOMAIN_ID=7` consistently, including one-off helper scripts from new
  SSH shells.
- Fix the clock before starting ROS; never jump the clock while TF is running.
- Do not layer `rr_bringup.sh` on top of separately started base, SLAM or map
  nodes. Teardown cleanly first.
- Require `remaining_ros=0` and `shm_fastrtps=0` before a cold bring-up.
- Start the gamepad before bring-up so the fixed joystick/e-stop process can
  acquire it.
- Foxglove goals must publish a 2D Pose on `/goal_pose`, not the default
  `/move_base_simple/goal`.
- `/initialpose` is for localization and also clears/cancels navigation state.
- If a goal does not move the car, inspect `nav.log`; do not assume Foxglove
  failed and do not keep sending goals blindly.
- “Starting point in lethal space” means the planner intentionally refused to
  create a path.
- Manual movement can make a later goal succeed by changing the legal start
  cell and refreshing localization. It should not be treated as the normal
  autonomy activation procedure.
- Once navigation is ready, test multiple **sequential** goals from the laptop
  to evaluate behavior across the map; finish or stop each attempt before
  sending the next one.
- The successful run tracked its path almost perfectly with no visible offset.
- A goal in a dark costmap region incorrectly produced a path and motion into
  that region. Unknown/invalid-space planning and goal validation remain open
  safety issues.
- Prefer log files and Foxglove over repeated `ros2` CLI diagnostics on this
  car because FastDDS shared-memory discovery has failed after participant
  churn in several sessions.
- Keep the joystick override in hand during every floor test.

## Final state

At the end of the work:

- clean-map display and live scan worked in Foxglove;
- the integrated stack had been brought up successfully;
- localization seeding on domain 7 worked;
- `/goal_pose` delivery from Foxglove was verified;
- Nav2 and the custom Pure Pursuit chain moved the robot to a goal;
- the successful run followed the path almost perfectly without a visible
  tracking offset;
- a goal placed in dark/unknown costmap space could still produce an
  undesirable path and motion into that region;
- delayed initial planning due to start-cell/localization state remained the
  main behavior to watch in the next run.

## Next session

1. Verify the laptop and robot are both connected to `roboracer` Wi-Fi.
2. Fix the clock before starting ROS if necessary.
3. Do not repeat all guide sections: if deployment/build and the saved map are
   already current, perform a clean teardown and run `~/rr/rr_bringup.sh`.
4. Seed localization explicitly on domain 7.
5. Verify `/scan` overlaps `/map` in Foxglove.
6. Confirm the robot start/footprint is in free costmap space.
7. Check unknown-space and goal-validity parameters before testing dark costmap
   regions; do not intentionally drive into an unsafe region.
8. From the laptop, send several short `/goal_pose` goals sequentially in known
   free space and observe each complete plan and run before sending the next.
9. If no plan appears, inspect the logs for `Starting point in lethal space`
   before moving manually or sending another goal.
