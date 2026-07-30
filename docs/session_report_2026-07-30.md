# RoboRacer Session Report — 2026-07-30

**Robot:** F1TENTH on Jetson Orin Nano, ROS 2 Humble, `ROS_DOMAIN_ID=7`
**Access:** `roboracer@192.168.50.10` over the `roboracer` Wi-Fi
**Foxglove:** `ws://192.168.50.10:8765`

## Goal

Extend the navigation workflow so that costmaps are cleared automatically when
the robot reaches a goal, create a program that drives through recorded goals
sequentially, and diagnose why both waypoint and manual navigation later stopped
producing robot motion.

## Headline outcome

- `rr_costmap_reset.py` now clears both Nav2 costmaps after every successfully
  completed `NavigateToPose` goal.
- The original `/initialpose` reset behavior remains intact.
- A new `rr_waypoint_mission.py` program sends two recorded poses in sequence
  and stops after Goal 2.
- Both scripts were deployed to `~/rr/`, syntax-checked on the robot, committed,
  and pushed to the `Hardware` branch.
- The waypoint program successfully delivered its first goal to Nav2, but the
  robot did not move because the planner classified its starting footprint as
  lethal.
- The navigation failure was traced to an old costmap configuration currently
  loaded on the robot. The known-good July settings have not yet been restored.

---

## Work completed

### Reading `/goal_pose` manually

`/goal_pose` is a one-shot command topic. Foxglove creates a publisher, sends
the pose, and then the publisher can disappear. Running `ros2 topic echo` after
the event therefore produced:

```text
WARNING: topic [/goal_pose] does not appear to be published yet
Could not determine the type for the passed topic
```

The reliable procedure is to supply the type explicitly and start listening
before sending the next Foxglove goal:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=7
export ROS_LOCALHOST_ONLY=0
ros2 topic echo /goal_pose geometry_msgs/msg/PoseStamped --once
```

The command waits until the next goal arrives. The important fields are
`pose.position.x`, `pose.position.y`, and the orientation quaternion.

### Costmap reset after a successful goal

The existing `~/rr/rr_costmap_reset.py` handled `/initialpose` only. It was
extended to subscribe to:

```text
/navigate_to_pose/_action/status
```

using `action_msgs/msg/GoalStatusArray` with reliable, transient-local QoS.
The node records the state of each goal UUID and reacts only when a known goal
transitions to `STATUS_SUCCEEDED`.

On success it now:

1. calls `/global_costmap/clear_entirely_global_costmap`;
2. calls `/local_costmap/clear_entirely_local_costmap`;
3. publishes an empty `nav_msgs/Path` on `/plan`;
4. publishes an empty transient-local path on `/control/plan`.

The empty paths make the custom Pure Pursuit controller discard the completed
route. Failed, canceled, rejected, and preempted goals do not trigger this new
success reset.

Historical action statuses present when the reset node starts are recorded but
ignored, preventing an old completed goal from causing a false reset during
bring-up.

The `/initialpose` behavior was preserved. Every new pose estimate still:

- clears both costmaps;
- cancels active navigation goals;
- empties `/plan` and `/control/plan`.

The old live helper was backed up before deployment:

```text
~/rr/rr_costmap_reset.py.bak_20260730
```

The updated script passed Python compilation locally and on the robot. After a
clean bring-up, its log showed successful goals being detected:

```text
goal <UUID> succeeded -> clearing costmaps and emptying paths
```

Nav2 logged the corresponding global and local clear-service requests, and the
custom controller logged receipt of the empty path. This verified the new
post-goal behavior on the live stack.

### Clean restart after deploying the reset helper

The first teardown reported one remaining ROS process:

```text
TEARDOWN_DONE remaining_ros=1 shm_fastrtps=0
```

The remaining process was inspected instead of starting a second stack on top
of it. Orphaned joystick/static-transform processes were stopped, and the stack
was brought back to a clean state before running:

```bash
~/rr/rr_bringup.sh
```

Localization was then seeded explicitly on the robot's ROS domain:

```bash
ROS_DOMAIN_ID=7 python3 ~/rr/seed_origin.py
```

This restart was required because the reset helper is launched by
`rr_bringup.sh`; replacing its file does not replace the already-running Python
process.

### Sequential waypoint mission

A new executable helper was created:

```text
hardware/rr/rr_waypoint_mission.py
```

and deployed as:

```text
~/rr/rr_waypoint_mission.py
```

It uses an `rclpy` `ActionClient` for Nav2's `/navigate_to_pose` action rather
than publishing a short-lived `/goal_pose` message. This provides an explicit
result for each leg of the mission.

The final route contains two goals.

#### Goal 1

```yaml
frame_id: map
position:
  x: 12.67165385576234
  y: 1.0165253135270669
orientation:
  z: 0.6576101747284752
  w: 0.7533583862237048
```

#### Goal 2

```yaml
frame_id: map
position:
  x: 1.6156199176312878
  y: 4.194904403911615
orientation:
  z: -0.7498062035160866
  w: 0.6616575074529064
```

An earlier draft returned to the recorded initial position after Goal 2. The
final requirement removed that third leg: the robot must stop after reaching
Goal 2.

Mission behavior:

- prints both target coordinates before starting;
- requires the operator to type `START` before any goal is sent;
- waits up to 10 seconds for `/navigate_to_pose`;
- prints distance-remaining feedback approximately once per second;
- sends Goal 2 only if Goal 1 returns `STATUS_SUCCEEDED`;
- waits two seconds after Goal 1 so the successful-goal costmap reset can run;
- stops the sequence if a goal is rejected, aborted, or canceled;
- attempts to cancel the active server-side goal when Ctrl+C is received;
- sends no goal after Goal 2 succeeds.

Run it only after bring-up and localization:

```bash
source /opt/ros/humble/setup.bash
source ~/roboracer_ws/install/setup.bash
export ROS_DOMAIN_ID=7
python3 ~/rr/rr_waypoint_mission.py
```

Keep the gamepad ready and hold LB for the emergency override during every
physical test. Do not publish competing Foxglove goals while the waypoint
mission is active.

### Navigation stopped producing motion

Later in the session, neither the waypoint program nor manually published
Foxglove goals caused motion. A process inspection showed that all required
components were still alive, including:

- slam_toolbox localization;
- Nav2 planner, controller, behavior server, and BT navigator;
- clean map server;
- plan QoS relay;
- map-to-odometry relay;
- custom Pure Pursuit controller;
- auto-keeper and costmap-reset helpers.

The live logs proved that both clients delivered their goals. For example, the
waypoint program's Goal 1 reached the planner as `(12.67, 1.02)`. Nav2 then
repeatedly rejected planning with:

```text
GridBased: failed to create plan, invalid use: Starting point in lethal space!
Planning algorithm GridBased failed to generate a valid path
```

Because no valid `/plan` was produced, the plan relay and custom controller had
nothing to follow. The waypoint program therefore was not the cause of the
failure; manual `/goal_pose` commands failed for the same reason.

Nav2 attempted spin and backup recovery behaviors, but these timed out. In this
stack, Nav2's controller/recovery velocity output is not bridged to `/drive`;
physical motion comes from the custom controller after a valid plan exists.

### Costmap configuration regression

The active Nav2 source file on the robot was inspected:

```text
~/roboracer_ws/src/RoboRacer-Shiran/
  roboracer_estimation/config/nav2_params_real.yaml
```

It currently contains the older costmap state:

```yaml
local_costmap:
  robot_radius: 0.22
  inflation_radius: 0.25

global_costmap:
  robot_radius: 0.22
  plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
  inflation_radius: 0.25
```

This differs from the previously validated configuration documented on
2026-07-16 and 2026-07-28:

```yaml
local_costmap:
  robot_radius: 0.10
  inflation_radius: 0.12

global_costmap:
  robot_radius: 0.10
  plugins: ["static_layer", "inflation_layer"]
  static_layer:
    map_topic: /map
  inflation_radius: 0.40
```

The known-good version remains on the robot as:

```text
nav2_params_real.yaml.bak_turnrad
```

A file comparison showed that the costmap block is the relevant difference.
The active file has a 1970 modification time and matches the older repository
version, which strongly suggests that an old copy was deployed while the
robot's clock was unset. The exact overwrite event was not observed, so this is
an inference rather than a proven cause.

At diagnosis time, the estimated robot pose was approximately:

```text
x = 0.579 m
y = -0.114 m
yaw = -0.092 rad (-5.3 degrees)
```

The robot's center cell in the published global costmap was free (`0`), but the
nearest `99` inscribed cell center was only about `0.254 m` away. With a 0.05 m
grid, that cell's edge was approximately `0.219 m` away—effectively on the
boundary of the active `0.22 m` robot radius. Small localization changes can
therefore make the start footprint collide with an inscribed/lethal cell.

This is consistent with the earlier finding that the car parks too close to a
wall for the old 0.22 m radius. The enabled global obstacle layer can also add
persistent LiDAR marks and make planning less repeatable.

No Nav2 parameter was changed during the diagnosis. Restoring the known-good
settings is still pending and requires a clean Nav2 restart before they take
effect. Reducing `robot_radius` also reduces the planner's physical safety
margin, so the operator must retain the LB override and test slowly in known
free space.

---

## Repository update

The two new session scripts were committed and pushed to `origin/Hardware`:

```text
cb619c8 Add sequential waypoint mission and post-goal costmap reset
```

Files in that commit:

```text
hardware/rr/rr_costmap_reset.py
hardware/rr/rr_waypoint_mission.py
```

Both files are executable. The commit contains no credentials or unrelated
files.

This report was created afterward and is intentionally local until it is
reviewed and committed separately.

## Key lessons

- `/goal_pose` is an event, not a continuously available record of the current
  goal. Listen before publishing and supply `geometry_msgs/msg/PoseStamped`.
- Use Nav2 action results for a sequential mission; do not advance merely after
  a fixed travel-time delay.
- Clear costmaps after `STATUS_SUCCEEDED`, not after canceled or failed goals.
- Empty `/plan` and `/control/plan` after completion so the custom controller
  cannot retain a completed path.
- Replacing a helper file does not update the running process. Perform a clean
  teardown and bring-up after deployment.
- A goal being received does not mean a path was generated. If the robot does
  not move, inspect `nav.log` before repeatedly sending goals.
- `Starting point in lethal space` is a costmap/start-footprint failure, not a
  Foxglove or waypoint-client failure.
- `rr_bringup.sh` currently loads whatever Nav2 configuration is present in the
  workspace; it enforces the VESC calibration but does not enforce the
  known-good costmap parameters.
- Keep local repository parameters synchronized with the validated robot
  configuration so a later deployment cannot silently restore old values.
- Keep the joystick/LB override ready for all floor tests.

## Final state

- The successful-goal costmap reset is implemented, deployed, and verified.
- The two-goal sequential mission is implemented and deployed.
- Both scripts are present on the remote `Hardware` branch in commit `cb619c8`.
- The robot stack is alive and accepts navigation goals.
- Planning is currently unreliable from the parked start because the active
  costmap parameters have regressed to the old lethal-start configuration.
- The configuration regression has been diagnosed but not fixed.
- This report exists only in the local `docs/` folder.

## Next session

1. Review the known-good backup and restore only the validated costmap changes
   to both the repository and the robot's active source file.
2. Perform `~/rr/rr_teardown.sh` until `remaining_ros=0`, then run
   `~/rr/rr_bringup.sh`.
3. Seed localization explicitly with `ROS_DOMAIN_ID=7`.
4. Verify that `/scan` aligns with `/map` and the robot footprint begins in
   free global-costmap space.
5. Send one short goal in clearly known free space and check that a `/plan` is
   produced before testing the two-goal mission.
6. Run `~/rr/rr_waypoint_mission.py`, observe Goal 1 complete before Goal 2 is
   sent, and verify that the program stops after Goal 2.
7. Confirm the reset log records one post-success clear for each completed
   mission leg.
