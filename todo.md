# Team TODO - RoboRacer/F1TENTH Simulator

## Goal
Prepare a reliable simulator baseline, then split and execute core autonomy workstreams:
1) Perception
2) Localization
3) Planning
4) Control

---

## Phase 1 - Validate Car Properties (and fix if inaccurate)

### 1.1 Collect reference values
- Review platform specs and measured values for:
  - wheelbase, width, mass, moment of inertia
  - max steering angle, max steering velocity
  - max accel/decel
  - tire/friction-related parameters
- Record references in a short note under `docs/` (or this file) so tuning decisions are traceable.

### 1.2 Audit simulator parameters
- Check `params.yaml` car/dynamics fields:
  - `wheelbase`, `width`, `mass`, `moment_inertia`
  - `max_speed`, `max_accel`, `max_decel`
  - `max_steering_angle`, `max_steering_vel`
  - `friction_coeff`, `height_cg`, `l_cg2rear`, `l_cg2front`, `C_S_front`, `C_S_rear`
- Confirm units (meters, radians, m/s, m/s^2, kg).

### 1.3 Validate behavior in simulation
- Run baseline launch:
  - `ros2 launch f1tenth_simulator simulator.launch.py`
- Perform tests:
  - straight-line acceleration/braking
  - max steering at low speed
  - moderate-speed cornering stability
  - stopping distance consistency
- Compare observed behavior vs expected physical behavior.

### 1.4 Tune and lock values
- If mismatch is observed, tune parameters in `params.yaml`.
- Re-run the same tests after each tuning batch.
- Freeze a validated set of parameters and document rationale.

**Deliverable:** validated car model parameter set + short tuning report.

---

## Phase 2 - Debug RViz Red Items (Display Errors)

### 2.1 Identify failing displays
- Open RViz from launch and list displays shown in red.
- For each red display, note exact error text (topic missing, TF missing, wrong type, etc.).

### 2.2 Check topic availability
- Run:
  - `ros2 topic list`
  - `ros2 topic info /<topic_name>`
- Confirm expected publishers for each RViz display topic.

### 2.3 Check TF integrity
- Verify frame tree and required transforms:
  - `map -> base_link`
  - `base_link -> laser`
- Use RViz TF display and tf tools to locate missing/broken transforms.

### 2.4 Fix configuration issues
- Update as needed in:
  - `launch/simulator.rviz`
  - `launch/simulator.launch.py`
  - `params.yaml`
  - relevant node publishers/subscribers
- Remove or disable stale RViz displays that are not produced by current stack.

### 2.5 Regression check
- Relaunch full stack and confirm all required RViz displays are green.
- Ensure map, robot model, scan, and key debug layers load without errors.

**Deliverable:** clean RViz profile with no critical red displays.

---

## Phase 3 - Start Core Autonomy Workstreams

## 3A) Perception
- Define perception outputs required by downstream modules (e.g., obstacle representation, free space, lane/track boundaries).
- Implement/clean data pipeline from `/scan` and map-related topics.
- Add visualization topics for debugging perception outputs in RViz.
- Validate on multiple maps and starting poses.

**Deliverable:** stable perception node(s) + documented outputs.

## 3B) Localization
- Establish baseline state estimate using available odometry/IMU and map frame consistency.
- Define localization interface (`pose`, covariance if used, update rate).
- Evaluate drift and robustness under aggressive maneuvers.

**Deliverable:** localization module with measurable accuracy and update rate.

## 3C) Planning
- Define planner I/O contract:
  - inputs (state, map/perception)
  - output drive topic and constraints
- Implement baseline planner (safe, deterministic first).
- Add checks for collisions, feasibility, and speed limits.

**Deliverable:** planner that drives valid trajectories in simulation.

## 3D) Control
- Implement/validate controller for path tracking (steering + speed).
- Tune gains against track-following metrics (cross-track error, heading error, smoothness).
- Ensure safe fallback/brake behavior is preserved.

**Deliverable:** controller that tracks planned trajectories reliably.

---

## Integration Milestones

### Milestone 1 - Baseline stability
- Car parameters validated
- RViz errors cleaned
- Core topics and TF stable

### Milestone 2 - First autonomous lap in simulation
- Perception + localization + planning + control integrated
- Car completes lap without collision under nominal conditions

### Milestone 3 - Performance iteration
- Improve lap consistency, speed, and robustness
- Add test scenarios and benchmark metrics

---

## Suggested Team Workflow
- Work in feature branches per module.
- Keep interfaces explicit (topic names, message types, rates).
- Add short changelog notes when modifying shared configs (`params.yaml`, launch, RViz).
- Run a quick integration test before merging.

---

## Immediate Next Actions (This Week)
- [ ] Validate/tune car parameters in `params.yaml`
- [ ] Resolve RViz red display errors and finalize RViz config
- [ ] Freeze module interfaces for perception/localization/planning/control
- [ ] Assign owners and start implementation tasks per module
