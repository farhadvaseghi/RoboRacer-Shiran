# RoboRacer — Session Log 2 (2026-06-30, afternoon)

Continuation of `session.md`. This session finished the Nav2 install, designed
and prepared the **real-car autonomous stack**, pushed everything to the
`Hardware` branch, and cleaned the car. No autonomous drive was performed yet —
all run steps are prepared in `guide.md` for you to execute.

## Summary

Picked up from the previous session's blocker (Nav2 not installed). Installed the
full Nav2 stack on the car, confirmed there is **no custom planner anywhere in
the repo** (the path always came from Nav2), then prepared a complete, safety-
capped, real-car Nav2 autonomous configuration plus a step-by-step runbook and a
per-bottleneck logging system. Committed and force-pushed it to the `Hardware`
branch.

## Accomplished

1. **Workspace `CLAUDE.md`** created at `Documents/Shiran-Hozuri/` (project map +
   conventions for future sessions).
2. **Repo investigation** — searched every branch. Confirmed **no custom
   planner** exists; `/plan` has always been produced by **Nav2's
   SmacPlannerHybrid**. Our code only has `path_relay_node` (a relay) and the
   `roboracer_control` Pure Pursuit (a *follower*).
3. **Nav2 fully installed** — the previous session's planner/controller/
   bt-navigator/smac servers were missing. Installed `ros-humble-navigation2` +
   `ros-humble-nav2-bringup` (38 packages) over the `Milad` 5 GHz hotspot.
   - Gotcha: with the car clock at 1970, `apt` rejects the repo Release file as
     "not valid yet" — advance the clock a few hours past real time first.
   - Ran the install **detached on the car** (`setsid`) so the flaky cellular
     link couldn't kill it; apt resumes from cache if interrupted.
4. **Hardware facts confirmed** (drive the real-car config):
   - wheelbase **0.25 m**; max steering ~**0.32 rad** → min turning radius ~0.75 m.
   - hardware speed cap ~**2.0 m/s** (9250 erpm / 4532 gain).
   - mux: autonomy `/drive` priority **10**, joystick `/teleop` priority **100**
     (joystick always overrides); `/drive` stale >0.2 s → mux commands 0.
   - plain frames `map→odom→base_link→laser`; Nav2's final command topic (after
     velocity_smoother) is `/cmd_vel`.
5. **Architecture decided — Nav2 end-to-end** for the first hardware run:
   `Nav2 (SMAC planner + Regulated Pure Pursuit) → /cmd_vel → cmd_vel_to_ackermann
   → /drive → mux → VESC`. Reason: the team's Pure Pursuit reads the pose
   straight from the odom message and does **no TF transform**, so it needs a
   map-frame pose; Nav2's controller is TF-aware and works directly. Using the
   team's Pure Pursuit is deferred to **Phase 2** (needs a small map-pose bridge).
6. **Discovered `path_relay_node` defaults to a hardcoded sim oval**
   (`use_forward_oval_route=True`) — it ignores the real map and must be set
   `false` if/when the team's Pure Pursuit path is used.
7. **Prepared all real-car files** (on the PC; not yet deployed to the car):
   - `roboracer_estimation/config/nav2_params_real.yaml` — plain frames,
     `use_sim_time:false`, wheelbase 0.25, amcl + SMAC + RPP, **first-run speed
     capped to 0.5 m/s**.
   - `roboracer_estimation/launch/autonomous_real.launch.py` — one-command Nav2
     bringup + `cmd_vel_to_ackermann`.
   - `nav2-realcar-deploy/deploy.sh` — PC→car deploy.
   - `nav2-realcar-deploy/rr/`: `rr_healthcheck.sh` (B1–B7 PASS/FAIL),
     `rr_initpose.sh`, `rr_goal.sh`, `rr_record.sh`.
   - `guide.md` — full step-by-step runbook (deploy → build → pre-flight →
     re-SLAM → autonomous start→end), with logging + troubleshooting per
     bottleneck.
8. **Logging system** — everything lands in `~/rr_logs/`; `rr_healthcheck.sh`
   pinpoints which link (B1 sensors … B7 actuation) is broken before digging into
   `nav2.log`; `rr_record.sh` bags the full chain.
9. **Pushed everything to the `Hardware` branch** of
   `farhadvaseghi/RoboRacer-Shiran` (force-update `24890038` → `b48d838`).
   Added the two package files plus a `hardware/` folder (guide + deploy + rr
   scripts).
10. **Cleaned the car** — removed this session's temp artifacts
    (`nav2_install.*`). Kept Nav2, `f1tenth_ws`, and the group's persistent work
    (`roboracer_ws`, `rr`, `rr_maps`, `rr_logs`).

## Current state at end of session

- **Nav2:** installed and complete on the car (38 packages).
- **Real-car autonomous config:** prepared and on GitHub (`Hardware` branch).
  **Not yet deployed to the car, not built, not test-driven.**
- **Car services:** nothing left running by us (no t_stack/slam started this
  session). Clock was advanced for apt; it resets to 1970 on reboot.
- **First autonomous run is capped at 0.5 m/s** in the prepared config.

## Gotchas added this session

- `apt` over the hotspot needs the clock advanced **past** the repo Release-file
  date (car boots to 1970).
- The team's Pure Pursuit does **no TF transform** → needs a map-frame pose; not
  drop-in with Nav2/amcl. Phase 2 = a TF→Odometry (map frame) bridge node.
- `path_relay_node` default route is a **sim oval** — turn it off for real maps.
- The PC's `RoboRacer-Shiran/` working copy **disappeared mid-session** (the
  `Documents/Shiran-Hozuri` folder is OneDrive-synced and dehydrated/removed it).
  The push was done from a fresh clone in local temp instead. **Don't keep the
  git working copy under OneDrive** — clone it somewhere local (e.g. `~/Github`).

## How to resume (next session)

1. SSH in; fix clock if offline; bring up `~/t_stack.sh`.
2. Deploy + build the prepared files:
   ```
   bash nav2-realcar-deploy/deploy.sh          # PC (Git Bash)
   # on car:
   cd ~/roboracer_ws && colcon build --packages-select roboracer_estimation && source install/setup.bash
   ```
   (Or pull the `Hardware` branch on the car.)
3. Follow `guide.md` §3→§5: pre-flight health check → (re-SLAM if needed) →
   launch `autonomous_real.launch.py` → set start pose → send goal → 0.5 m/s run.
4. Use `rr_healthcheck.sh` at each stage; raise speed only after a clean lap.
5. **Phase 2 (optional):** build the map-pose bridge + real-car Pure Pursuit
   launch to drive with the team's controller (see `guide.md` §9).
