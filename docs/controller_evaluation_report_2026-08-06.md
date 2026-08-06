# Real-System Controller Evaluation — 2026-08-06

## 1. Purpose

This report compares three ROS 2 bag recordings from real RoboRacer Pure Pursuit tests:

- `~/rr_logs/controller_eval_20260806_144736`
- `~/rr_logs/controller_eval_20260806_175134`
- `~/rr_logs/controller_eval_20260806_175253`

The purpose is to measure path-tracking accuracy, heading accuracy, timing, speed response, steering demand, final goal accuracy, and localization integrity. The results are suitable for comparing future controller configurations only when the same route, initial pose, goal, speed, and physical conditions are used.

## 2. Test context

- ROS 2 domain: `7`
- Controller: custom Pure Pursuit controller
- Nominal controller frequency: `20 Hz`
- Nominal maximum drive command during these runs: `1.0 m/s`
- Configured Pure Pursuit goal tolerance: `0.25 m`
- Configured Nav2 XY goal tolerance: `0.25 m`
- Approximate planned route length: `13.4–13.5 m`
- Start region: approximately `(-0.38, 0.10)` in the `map` frame
- Goal region: approximately `(11.74–11.81, 2.72–2.86)` in the `map` frame

The tests used onboard localization. Therefore, cross-track error and final position error are errors relative to the robot's estimated pose, not independent physical ground truth.

## 3. Recording quality and selection

| Recording | Bag duration | Messages | Assessment | Recommended use |
|---|---:|---:|---|---|
| `144736` | 41.881 s | 5,379 | Complete normal run, but no TF was recorded and the final samples contain a path-direction anomaly | Supporting normal run |
| `175134` | 61.922 s | 17,683 | Valid recording of a failed/stalled run | Failure-case analysis only |
| `175253` | 100.796 s | 27,847 | Most complete normal recording, including TF and TF-static data | Primary benchmark run |

The recording itself is not faulty in any of the three cases. `175134` accurately captured abnormal system behavior and must not be averaged into normal controller-performance results.

## 4. Numerical comparison

Absolute values are used for the cross-track and heading-error statistics.

| Metric | `144736` | `175134` | `175253` |
|---|---:|---:|---:|
| Run classification | Normal, with anomalies | Failed/stalled | Normal, primary |
| Initial planned path length | 13.435 m | 13.496 m | 13.538 m |
| Active controller duration | 14.898 s | 49.001 s | 14.453 s |
| Controller update rate | 19.197 Hz | 19.999 Hz | 19.996 Hz |
| Tracking samples | 287 | 981 | 290 |
| Median cross-track error | 0.025 m | 0.021 m* | 0.018 m |
| RMS cross-track error | 0.047 m | 0.614 m | 0.060 m |
| 95th-percentile cross-track error | 0.115 m | 2.053 m | 0.159 m |
| Maximum cross-track error | 0.149 m | 2.058 m | 0.264 m |
| Median heading error | 2.427° | 165.055° | 2.523° |
| 95th-percentile heading error | 9.958° | 177.634° | 10.707° |
| Maximum heading error | 179.395° | 177.634° | 30.044° |
| Heading samples above 30° | 3 | 690 | 1 |
| Median commanded speed | 1.000 m/s | 0.800 m/s | 1.000 m/s |
| Median measured speed | 1.001 m/s | 0.000 m/s | 1.000 m/s |
| 95th-percentile measured speed | 1.252 m/s | 1.021 m/s | 1.199 m/s |
| Maximum measured speed | 1.441 m/s | 1.216 m/s | 1.352 m/s |
| Steering saturation fraction | 7.67% | 61.12% | 2.42% |
| Minimum recorded distance to goal | 0.394 m | 1.296 m | 0.511 m |
| Final recorded position error | 0.425 m | 2.046 m | 0.570 m |
| Nav2 reported success | Yes | No | Yes |
| Maximum `/odometry/map` step | 0.498 m | 1.498 m | 0.410 m |
| Map-pose steps above 0.10 m | 7 | 25 | 16 |
| Map-pose steps above 0.25 m | 6 | 2 | 13 |

\* The median cross-track error in `175134` is misleading because the controller spent a long period producing nearly constant values while the run was stalled. Its RMS and 95th-percentile values correctly show the failure.

## 5. Run-by-run findings

### 5.1 Recording `144736`

The active controller interval was 14.898 seconds. Lateral tracking was strong:

- Median absolute cross-track error: `0.025 m`
- 95th-percentile absolute cross-track error: `0.115 m`
- Maximum absolute cross-track error: `0.149 m`
- Median absolute heading error: `2.427°`
- 95th-percentile absolute heading error: `9.958°`

Nav2 reported `Reached the goal` and `Goal succeeded`. However, `/odometry/map` never entered the configured `0.25 m` goal tolerance; its minimum recorded distance was `0.394 m`. The final recorded error was `0.425 m`.

The final three tracking samples reported heading errors of approximately `177–179°`. These samples occurred after Nav2 reported success and indicate an end-of-path direction/progress anomaly rather than ordinary path tracking.

The largest map-pose step was `0.498 m`, and six steps exceeded `0.25 m`. Message header times and receipt times were aligned, so these jumps were not caused by rosbag delivery backlog.

The measured speed matched the `1.0 m/s` command at the median, but its maximum reached `1.441 m/s`, which is a 44% peak overshoot relative to the command.

### 5.2 Recording `175134`

This was a failed or stalled run, not a normal benchmark. The controller remained active for 49.001 seconds without reaching the goal.

Important evidence:

- Median commanded speed: `0.8 m/s`
- Median measured speed: `0.0 m/s`
- Steering saturated at `±0.32 rad` for 61.12% of controller samples
- 95th-percentile cross-track error: `2.053 m`
- Median heading error: `165.055°`
- Final position error: `2.046 m`
- No `Goal succeeded` event was recorded

The planner repeatedly ran far below its requested 5 Hz frequency, reaching observed loop rates of approximately `0.29–0.91 Hz`, and later reported that no valid path could be found.

The largest map-pose discontinuity was `1.498 m` in approximately `0.021 s`. This cannot represent physical vehicle motion. The run should be retained as a failure case because it demonstrates that the controller continued requesting motion and maximum steering after the system was no longer making valid progress.

### 5.3 Recording `175253`

This is the best primary recording for normal numerical comparison because it contains a complete goal run and the most complete topic coverage.

The active controller interval was 14.453 seconds, and the controller maintained 19.996 Hz. Tracking performance was good:

- Median absolute cross-track error: `0.018 m`
- RMS cross-track error: `0.060 m`
- 95th-percentile absolute cross-track error: `0.159 m`
- Maximum absolute cross-track error: `0.264 m`
- Median absolute heading error: `2.523°`
- 95th-percentile absolute heading error: `10.707°`
- Steering saturation: `2.42%`

The controller-rate result is excellent, and steering saturation was much lower than in the other recordings. The 95th-percentile heading error is slightly higher than the suggested 10° target, and the maximum cross-track error is slightly higher than the `0.25 m` comparison threshold.

Nav2 reported `Reached the goal` and `Goal succeeded`, but `/odometry/map` never came closer than `0.511 m` to the published goal. The final recorded error was `0.570 m`. Nav2 also reported that the starting point was in lethal space during one replanning attempt.

The measured median speed was `1.000 m/s`, but the maximum reached `1.352 m/s`. The largest map-pose step was `0.410 m`; 13 steps exceeded `0.25 m`. As in `144736`, these discontinuities make internal tracking metrics less trustworthy as measurements of physical motion.

## 6. Comparison of the two normal runs

The two normal runs, `144736` and `175253`, show consistent internal path-tracking behavior:

- Active duration: `14.45–14.90 s`
- Median absolute cross-track error: `0.018–0.025 m`
- 95th-percentile absolute cross-track error: `0.115–0.159 m`
- Median absolute heading error: `2.43–2.52°`
- 95th-percentile absolute heading error: `9.96–10.71°`
- Median measured speed: approximately `1.00 m/s`

These values show that Pure Pursuit follows the path generated from the estimated pose accurately during ordinary operation. However, neither normal run entered the `0.25 m` goal tolerance according to `/odometry/map`, even though Nav2 declared both goals successful.

## 7. Main conclusions

1. **Pure Pursuit lateral tracking is good when evaluated against its own localized pose.** Median cross-track error was approximately 2 cm in both normal runs.
2. **The 20 Hz controller timing is stable.** The best run maintained 19.996 Hz.
3. **Localization is the dominant limitation.** All runs contained physically impossible map-pose jumps between consecutive samples.
4. **Goal completion is inconsistent.** Nav2 declared success in both normal runs even though `/odometry/map` remained outside the configured 0.25 m tolerance.
5. **Speed overshoot requires investigation.** Peak measured speeds were 35–44% above the 1.0 m/s command in the normal runs.
6. **The controller needs a failure response.** During `175134`, it continued commanding speed and maximum steering without valid progress.
7. **Internal tracking error is not physical ground truth.** Low cross-track error can coexist with incorrect localization and incorrect physical position.

## 8. Recommended benchmark use

- Use `controller_eval_20260806_175253` as the primary baseline.
- Use `controller_eval_20260806_144736` as a secondary normal-run comparison.
- Keep `controller_eval_20260806_175134` as a separate failure-case regression test.
- Do not average the failed run together with normal runs.
- Do not increase speed until localization discontinuities and goal-completion disagreement are understood.

## 9. Required follow-up

1. Compare historical `map -> base_link` TF against `/odometry/map` using `175253`, which contains TF data.
2. Inspect `map_odom_relay` and SLAM correction behavior around the recorded 0.3–1.5 m pose jumps.
3. Determine why Nav2 clears the path and reports success while the pose used by Pure Pursuit remains outside 0.25 m.
4. Add a controller watchdog that stops the drive command when measured progress is absent, localization jumps, the path is stale, or steering remains saturated for too long.
5. Check speed calibration and closed-loop speed control to reduce overshoot.
6. Repeat the same start-to-goal route at least three times after each change.
7. For true real-world accuracy, compare onboard localization with an external reference such as an overhead camera, AprilTags, or motion capture.

## 10. Recording command for future comparisons

Run the following in a separate SSH terminal after bringup:

```bash
source /opt/ros/humble/setup.bash
source ~/roboracer_ws/install/setup.bash

ROS_DOMAIN_ID=7 ros2 bag record --include-unpublished-topics --include-hidden-topics \
  -o ~/rr_logs/controller_eval_$(date +%Y%m%d_%H%M%S) \
  /goal_pose /plan /control/plan /tracking_error \
  /odometry/map /odom /odom_gyro \
  /drive /navigate_to_pose/_action/status \
  /tf /tf_static /rosout
```

Before publishing a goal, confirm that the recorder subscribes to `/tracking_error`, `/odometry/map`, `/drive`, `/control/plan`, `/tf`, and `/odom`. After the robot stops, end the recorder gracefully with `Ctrl+C` and wait for `Recording stopped`.

Keep the test course clear and keep the gamepad emergency stop available throughout every physical test.
