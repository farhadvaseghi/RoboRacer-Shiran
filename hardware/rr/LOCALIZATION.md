# RoboRacer — Localization runbook (read this before you run anything)

This is the **localization** step: making the car know where it is on the saved
map, *before* any autonomous driving. You place the car on the floor cross, run
one script, and confirm it is localized. Nothing here moves the car.

> **TL;DR**
> 1. Put the car on the **cross** on the floor, nose pointing **down the corridor**
>    (same way it faced when the map was recorded).
> 2. SSH in, then run: `~/rr/rr_localize_run.sh`
> 3. Wait for the check. You want **`RESULT: LOCALIZATION READY`** and, in
>    Foxglove, the red laser scan sitting **on the map walls**.
> 4. If it fails, find the `FAIL` line and jump to **Troubleshooting** below.

---

## 1. Place the car first (this is what makes it work)

Localization here is **slam_toolbox in localization mode** on the saved
`corridor_clean` map. That map was built with its **origin (0,0,0) exactly on the
painted cross**, and the config (`localize_slam_real.yaml`, `map_start_at_dock:
true`) makes the car come up believing it starts at the origin.

Use the **placement picture provided separately** for the exact cross location
and the direction the nose must point. In short:

- **Center** the car over the cross (roughly — within ~20–30 cm is fine, the
  scan-matcher pulls it in).
- **Orientation matters more than position.** The nose must point the **same
  direction** the car faced when the map was made (down the corridor, along the
  map's +x). Being ~90° off will not converge — the walls won't match.
- Flat ground, wheels straight, nothing leaning on the car.
- If someone re-mapped and the cross moved, the *new* map's origin defines the
  new cross — always place on the cross that matches the map you are loading.

Why so strict: with a good placement the car is localized **the instant the
stack comes up**, with no manual pose entry. A bad placement is the #1 reason
localization fails (see Troubleshooting).

---

## 2. Connect and pre-checks

1. Laptop on the **`roboracer`** Wi-Fi. Then:
   ```bash
   ssh roboracer@192.168.50.10
   ```
   (Ping is filtered — judge reachability by SSH succeeding, not by ping.)

2. **Clock.** The car has no RTC and boots to **1970**. A wrong clock breaks TF
   and SLAM. Fix it **now, before starting the stack** (UTC):
   ```bash
   sudo date -u -s "2026-07-14 12:00:00"   # use the real current UTC time
   date
   ```
   Never `date -s` once the stack is running — the time jump breaks TF. The
   localize script refuses to run if it sees 1970, precisely so you fix this first.

3. You do **not** need to start the base stack yourself — the localize script
   brings it up. If a stack from a previous run is already up, that's fine; the
   script is idempotent and reuses what's running (it never `pkill -f`).

---

## 3. Run localization

One command:

```bash
~/rr/rr_localize_run.sh
```

It will, in order:

1. Bring up the **base stack** (LiDAR `/scan`, VESC `/odom`, joystick, mux),
   **Foxglove** bridge (`ws://192.168.50.10:8765`), and **slam_toolbox
   localization** on `corridor_clean` — via `rr_up_slam.sh`.
2. Start the **odom keepalive** (`rr_keep.sh`): a zero-speed `/drive` at 20 Hz.
   The VESC only publishes `/odom` + `odom→base_link` **while a drive command is
   flowing**. Parked with no command, the TF chain is broken and slam drops every
   scan. This keepalive holds the chain alive **without moving the car**.
3. Arm the joystick **deadman / e-stop** (`rr_fix_joy.sh`, LB = button 4).
4. Wait up to ~35 s for `map→odom`, then print a **PASS/FAIL check**.

Leave it running. To watch visually, connect the **Foxglove desktop app** to
`ws://192.168.50.10:8765` and set the 3D panel frame to **`map`**.

> The keepalive publishes to `/drive`. That is correct **for localization**, but
> it fights Nav2's controller. **Stop it before sending a navigation goal:**
> `~/rr/rr_keep_stop.sh` (see **Done / hand-off** below).

---

## 4. What "localized" looks like

At the end the script prints one of:

```
 RESULT:  LOCALIZATION READY  (0 warning(s))
```
or
```
 RESULT:  NOT LOCALIZED  (N critical, M warning(s))
```

For **READY** you want these lines all `PASS`:

| Check | Line | Meaning |
|---|---|---|
| L1 | `/scan @ N Hz` | LiDAR is publishing |
| L1 | `/odom @ N Hz` | VESC odom is flowing (keepalive working) |
| L2 | `odom -> base_link` | wheel-odom TF present |
| L3 | `/map has data` | saved map loaded |
| **L4** | **`map -> odom present`** | **the car is localized — the one that matters** |
| L4 | `map -> odom stable while parked` | pose is not jittering |

**`map→odom` is the whole point.** Without it the car has no idea where it is,
and no goal will ever plan. `base_link→laser` is only a `WARN` if missing.

**Also eyeball it in Foxglove:** the red **laser scan must lie on the map
walls**. If the scan is rotated or shifted off the walls, the pose is wrong even
if `map→odom` exists — treat that as not localized and see Troubleshooting.

---

## 5. Troubleshooting

Find your `FAIL`/`WARN` line, then:

### No `map→odom` (main failure — "NOT LOCALIZED")
The car isn't localizing. In likelihood order:
1. **`/odom` is also FAIL** → fix that first (below). No odom → slam can't
   transform scans → no `map→odom`. This is the most common cause.
2. **Placement.** Car is off the cross or facing the wrong way. Re-place it on
   the cross, nose down the corridor, and re-run. Confirm in Foxglove the scan
   roughly matches the walls.
3. **Not converged yet.** Give it more time: `CONVERGE_WAIT=60
   ~/rr/rr_localize_run.sh`, or nudge the car ~0.5 m forward by joystick (hold
   LB) so slam gets motion to scan-match, then let it settle.
4. **slam failed to load the map** → see *Map won't load*.

### No `/odom` (keepalive)
The VESC only emits `/odom` while a `/drive` command flows. The script starts a
zero-speed keepalive; if `/odom` is still FAIL:
- Is the keepalive alive? `pgrep -f 'topic pub /drive'` should list a PID. If
  not: `~/rr/rr_keep.sh`, then re-check.
- Is the base stack actually up (VESC)? `ls -l /dev/sensors/vesc` should exist;
  `pgrep -f vesc_driver` should list a PID. If not, the base stack didn't start
  — check `/tmp/t_stack.log`, replug the VESC USB if needed, re-run the script.
- Two keepalives fighting? Stop all with `~/rr/rr_keep_stop.sh`, then
  `~/rr/rr_keep.sh` once.

### No `/scan`
LiDAR is on the wired net. Check `eno1` is up and the Hokuyo at `192.168.0.10`
is reachable; read `/tmp/t_stack.log`. Restart the base stack if the interface
came up late.

### Map won't load / `/map` empty
- slam localization log: `cat /tmp/slamloc.log` (look for a map-path or yaml
  error). The map must exist: `ls ~/rr_maps/corridor_clean.*` (`.yaml`, `.pgm`,
  `.posegraph`, `.data`).
- The map is chosen inside `localize_slam_real.yaml`. To use a different saved
  map, point that file at it (and remember its origin defines a *different*
  cross).

### Pose drifts / "deflects" (map→odom WARN, or a second map in Foxglove)
Seen before: slam localization can jitter or render an offset "second map" over
the loaded one.
- Make sure the car is truly **still** (keepalive is zero-speed; nobody leaning
  on it). A stable parked pose should move < a few cm.
- Confirm the scan overlaps the walls in Foxglove. A small steady offset is
  usually a scan-match settle; large or growing offset = bad placement or a poor
  map — re-place, or re-map if the environment changed.
- Known open item (localization-quality follow-up); it did not stop a short run
  before, but resolve it before longer/faster driving.

### `slam_toolbox not running`
The localization node died. Read `/tmp/slamloc.log`. Common causes: bad params
file path, or another slam/amcl instance already publishing `/map`/`map→odom`
(they collide). Only **one** localization source may run. Check:
`ros2 node list | grep -Ei 'slam|amcl|map_server'`.

### Alternative: AMCL instead of slam localization
There is also an AMCL-based path on the car (`~/rr/rr_up.sh`, `amcl_global.yaml`).
The slam flow above is the one verified to come up localized on the cross with no
manual pose. Only switch to AMCL if slam localization is unusable — and never run
both at once (both publish `/map` + `map→odom`).

---

## 6. Done / hand-off to navigation

When you have `LOCALIZATION READY` and the scan overlaps the walls, the car is
localized and staying put. Before you send a **navigation goal**:

```bash
~/rr/rr_keep_stop.sh     # STOP the keepalive first — it fights Nav2 on /drive
```

During active navigation Nav2's own commands keep `/odom` alive. When the car is
back at rest and you want to hold localization again, restart `~/rr/rr_keep.sh`.

Full autonomous flow (bring up Nav2, send a goal, health-check B5–B7) is in the
top-level `guide.md`. This runbook covers localization only.

---

## 7. Under the hood (what the script uses)

`rr_localize_run.sh` is a thin orchestrator + verifier over the car-side scripts:

| Script | Role |
|---|---|
| `rr_up_slam.sh` | base stack + Foxglove + slam_toolbox localization on `corridor_clean` |
| `rr_keep.sh` / `rr_keep_stop.sh` | zero-speed `/drive` odom keepalive (start / stop) |
| `rr_fix_joy.sh` | remap joystick deadman to LB (button 4); arms the e-stop |
| `rr_localize.sh` | low-level: raw slam localization node only (advanced; the runner above is the front door) |
| `localize_slam_real.yaml` | slam localization params (map, `map_start_at_dock: true`) |

Logs: `/tmp/t_stack.log`, `/tmp/foxglove.log`, `/tmp/slamloc.log`,
`/tmp/zerodrive.log`. Override defaults via env, e.g.
`ROS_DOMAIN_ID=7 MAP_NAME=corridor_clean CONVERGE_WAIT=60 ~/rr/rr_localize_run.sh`.

**Deploy from the PC** (updates the car copy of these files):
```bash
bash /c/Users/Student/Documents/Shiran-Hozuri/nav2-realcar-deploy/deploy.sh
```
