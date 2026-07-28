# RoboRacer Odometry Calibration Log — 2026-07-23

Straight-line distance calibration of `speed_to_erpm_gain` (VESC wheel odometry).

**Method:** `cal_drive.py <dist>` drives the car straight (steering = 0) at 0.3 m/s until the
wheel **odometry** reads `<dist>` metres, then stops. The **REAL** distance is tape-measured on
the floor. Goal: make the real distance equal the commanded distance.

## Fixed setup during these tests
- Controller: custom `pure_pursuit`, `drive_topic: /drive`, speed cap 0.5 (the cal drives run at 0.3)
- Steering: `steering_angle_to_servo_offset: 0.499` — **unchanged** (car drives straight, no lateral drift)
- slam localization: `minimum_travel_distance: 0.3`, `minimum_time_interval: 0.2`
- Map: `corridor_despeck` (0.05 m/cell)
- Config: `~/f1tenth_ws/.../config/vesc.yaml` (install + src) + enforced by `~/rr/rr_bringup.sh` (`ERPM_GAIN`)
- **Note:** `vesc.yaml` uses a `/**:` wildcard node, so `speed_to_erpm_gain` is shared by
  `ackermann_to_vesc` (commanding) **and** `vesc_to_odom` (odometry).

## Test results

| # | speed_to_erpm_gain | target (odom) | odom actual | REAL (tape) | error | real/target | map/slam |
|---|--------------------|---------------|-------------|-------------|-------|-------------|----------|
| A | 4093 | 2.00 m | 2.002 m | 1.77 m | −0.23 m (short) | 0.885 | 1.64 m |
| C | 4629 | 1.00 m | 1.000 m | 0.88 m | −0.12 m (short) | 0.880 | 1.057 m |
| D | 5260 | 1.00 m | 1.008 m | 1.21 m | **+0.21 m (over)** | 1.210 | 1.118 m |
| E | 4860 | 1.00 m | 1.006 m | ~0.965 m | **−0.035 m (short)** | ~0.965 | — |
| F | 4860 | 2.00 m | 2.003 m | ~2.09 m | **+0.09 m (over)** | ~1.045 | — |
| G | 4770 | 3.00 m | 3.002 m | 3.25 m | **+0.25 m (over)** | 1.083 | — |
| H | 4400 | 3.00 m | 3.001 m | 3.14 m | **+0.14 m (over)** | 1.047 | — |
| **I** | **4100** | 3.00 m | 3.003 m | **3.00 m** | **0 (EXACT)** ✅ | **1.000** | — |

**MEASUREMENT-REFERENCE FIX (from row G on):** measure the SAME point (**front bumper**) at start AND
end — front-to-front = true translation = what wheel odom should read. Earlier rows (A–F) had an
inconsistent/back-bumper reference → that was most of the scatter. Rows G–I are all clean front-bumper
3 m tests and behave consistently.

(4093 = pre-session value. Rows A–F superseded by the front-bumper G–I sweep. Test B — gain 4629, 2 m,
map read 2.09 — no tape, omitted.)

## Analysis
- Gains 4093 and 4629 both gave ~0.88 (short) — the +13 % barely moved real.
- 4629 → 5260 (+13.6 %) overshot to 1.21 (over) — a large, non-linear jump.
- The gain sits on **both** the command and odom sides (shared via `/**:`), so it partially
  self-cancels in the drive-until-odom=target test; tape/start-point precision and wheel slip
  add noise → the true target gain is not perfectly stable.

## Next gain estimate (superseded — see Conclusion)
Linear interpolation between the two most recent points (4629 → 0.88, 5260 → 1.21) for real/target = 1.0:

```
gain ≈ 4629 + (1.00 − 0.88)/(1.21 − 0.88) × (5260 − 4629)
     ≈ 4629 + 0.364 × 631
     ≈ 4858
```

**4860 was a false convergence** — it only looked converged because the 1 m/2 m tape used an
inconsistent reference point. Once we fixed the reference to **front bumper** and used the longer,
more-precise 3 m baseline, 4860-era gains showed a clear systematic **over-run**, and a proper sweep
converged much lower.

## FINAL CONCLUSION — gain 4100 is EXACT (2026-07-23)
Clean front-bumper 3 m sweep:

| gain | real @ 3 m target | offset |
|------|-------------------|--------|
| 4770 | 3.25 m | +25 cm (over) |
| 4400 | 3.14 m | +14 cm (over) |
| **4100** | **3.00 m** | **0 (exact)** ✅ |

Because the `/**:` wildcard puts the gain on both command and odom sides, odom speed self-cancels to
≈ the commanded 0.3 m/s, so **real distance ∝ gain** — a clean, monotone knob once the measurement
reference was consistent. Elapsed time also rose monotonically (10.9 → 11.7 → 13.3 s) as gain dropped,
confirming we were moving the real operating point, not chasing noise. 4100 lands the front-bumper
distance dead on 3.00 m.

**DECISION: `speed_to_erpm_gain = 4100.0` is the calibrated value. HARDCODED** in all three vesc.yaml
copies (`install/`, `src/`, `build/`) **and** `~/rr/rr_bringup.sh` (`ERPM_GAIN="4100.0"`, re-enforced
on every bring-up).

### Method note (learned this session)
- **Always measure the same reference point (front bumper) at start and end.** That was the single
  biggest error source in rows A–F.
- Applying a new gain needs a **base-stack restart** (live `ros2 param set` doesn't take). The
  reliable way is the **name-based** kill `~/rr/kill_base.sh` then re-run `rr_bringup.sh` — NOT the
  PGID kill (`kill -TERM -PGID`), which left the launch half-alive so bringup saw "base already up" and
  skipped the relaunch, and the crippled launch then collapsed.

## Notes
- Car drives physically straight (no lateral) → steering calibration is fine.
- For real path/goal following, the controller uses slam's map-frame pose (`/odometry/map`,
  scan-matched to the walls), so slam accuracy matters more than raw odom for driving; this
  calibration mainly tightens the dead-reckoning between slam updates.
