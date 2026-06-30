# RoboRacer — Session Log (2026-06-30)

What we did this session, the current state, and exactly how to resume next time.

## Summary

Took our simulation stack (`farhadvaseghi/RoboRacer-Shiran`, branch
`dynamic-overtaking+slam`) onto the real RoboRacer car for the first time.
Backed up the other group's work, deployed + built our packages on the car,
validated the sensor/actuator pipeline, got the car **moving** under joystick
control, and completed a **SLAM mapping run** of the track. Autonomous Nav2
driving is set up but blocked on finishing the Nav2 package install.

## Accomplished

1. **SSH access** — passwordless key login to `roboracer@192.168.50.10` over the
   `roboracer` Wi-Fi. (Car drops ICMP/ping by design; judge by SSH.)
2. **Backup of the other group's work** → `car-backup-2026-06-30/`
   (`custom-work.tar.gz` 37 MB + `autoware-src.tar.gz` 817 MB).
3. **Our repo** → cloned to PC (`RoboRacer-Shiran/`) and pushed to the car
   (`~/roboracer_ws/src/RoboRacer-Shiran`); built `roboracer_estimation` +
   `roboracer_control` with colcon (sim deps skipped).
4. **Hardware validated (motion-free):** LiDAR `/scan` 40 Hz, VESC `/odom`
   50 Hz, `odom→base_link` + `base_link→laser` TF all good.
5. **Car clock fixed** — it boots to ~1970 (no RTC battery); connecting to the
   internet auto-synced it via NTP. (If offline at boot:
   `sudo date -u -s "YYYY-MM-DD HH:MM:SS"`.)
6. **Internet on the car** — connected its internal Wi-Fi (`wlP1p1s0`) to the
   `Milad` iPhone hotspot (5 GHz — the car is deaf on 2.4 GHz). Needed a low
   route-metric so the hotspot becomes the default route.
7. **First motion** — confirmed the full chain gamepad → `/teleop` → mux →
   `/commands/motor/speed` → VESC → wheels. The car drives under joystick
   (deadman = **LB**; release = instant stop).
8. **SLAM mapping run** — drove a slow loop with slam_toolbox; saved
   **`my_track`** (745×899 px ≈ 37×45 m) to `~/rr_maps/`, the repo `maps/`, and
   `Documents/Shiran-Hozuri/maps/`.

## Current state at end of session

- **slam_toolbox:** stopped.
- **Map refresh loop / map snapshots:** stopped.
- **Nav2 install:** **incomplete** — stopped while still downloading (the
  hotspot link was too flaky for the big download). No packages were unpacked,
  so the system is clean (no broken dpkg state). `nav2_bringup`,
  `nav2_smac_planner`, `nav2_controller` are still **missing**.
- **Drivers (`~/t_stack.sh`):** may still be running in your terminal — Ctrl+C
  it.
- **Power:** the traction LiPo reads 12.0 V — **disconnect/turn it off** so it
  doesn't drain.

## Key facts about THIS car (gotchas)

- **Hardware:** Jetson Orin Nano Super, Ubuntu 22.04, ROS 2 Humble. Hokuyo
  LiDAR on wired Ethernet `192.168.0.10` (`eno1` static IP via NetworkManager
  profile "Hokoyu"). VESC on USB (`/dev/sensors/vesc`). **ZED 2i camera present
  but its ROS wrapper/SDK is NOT installed.**
- **Real frames are plain** (`base_link`, `odom`, `laser`) — the sim used
  namespaced `ego_racecar/*`.
- **Real wheelbase = 0.25 m** (sim config says 0.3302).
- `/odom` only publishes while a drive command flows (vesc_to_odom waits for a
  servo command). At idle there is no `/odom` and no `odom→base_link` TF — this
  is expected, not a bug.
- The car only sees **5 GHz** Wi-Fi; its 2.4 GHz reception is dead.

## Artifacts created

**On the PC (`C:\Users\Student\Documents\Shiran-Hozuri\`):**
- `car-backup-2026-06-30/` — other group's work backup
- `RoboRacer-Shiran/` — our repo (reference copy)
- `maps/my_track.{pgm,yaml,png}` — the SLAM map
- `roboracer-architecture.md` — full system architecture
- `roboracer-hardware-bringup.md` — bring-up guide + config-adaptation checklist
- `live_map.html` + `live_map.png` — live map viewer (for mapping runs)
- `ssh-connection-guide.md`, `session.md` (this file)

**On the car:**
- `~/roboracer_ws/` — our built workspace
- `~/rr/` — helper scripts: `rr_slam.sh`, `rr_record.sh`, `rr_savemap.sh`,
  `rr_snap.sh`, `rr_doctor.sh`, `slam_params_real.yaml`
- `~/rr_maps/my_track.*`, `~/rr_logs/` (rosbags)

## Blockers / pending (next session)

1. **Finish Nav2 install** (needs a stable connection — ideally real Wi-Fi or a
   less laggy hotspot):
   ```bash
   sudo apt-get -o Acquire::ForceIPv4=true -o Acquire::Retries=20 install -y \
       ros-humble-nav2-bringup ros-humble-nav2-smac-planner ros-humble-nav2-controller
   ```
2. **Adapt configs sim → hardware** (see `roboracer-hardware-bringup.md` §
   "Config adaptation"): frames `ego_racecar/*` → plain; wheelbase 0.25;
   `odom_topic` → `/odom`; cap speeds to ~1 m/s for first autonomous run;
   `enable_mpc_overtaking: false`.
3. **Estimation without the ZED wrapper:** run on VESC `/odom` only (the EKF's
   `ekf_real.yaml` expects ZED visual odom/IMU that isn't available). Simplest
   first pass: feed the controller `/odom` directly, skip the EKF.
4. **Hardware localization for Nav2:** add a `map_server` (publishing
   `my_track`) + slam_toolbox in **localization** mode for the `map→odom` TF —
   `navigation.launch.py` does not start these (the sim provided them).

## How to resume

1. SSH in: `ssh roboracer@192.168.50.10` (over `roboracer` Wi-Fi).
2. If offline at boot, fix the clock and reconnect the hotspot (5 GHz):
   `sudo nmcli dev wifi connect "Milad" password "12345678" ifname wlP1p1s0`
   then `sudo nmcli con mod Milad ipv4.route-metric 50 && sudo nmcli con up Milad`.
3. Drivers: `~/t_stack.sh`. Health check: `~/rr/rr_doctor.sh`.
4. To map again: `~/rr/rr_slam.sh`, drive a loop, `~/rr/rr_savemap.sh <name>`.
5. To progress autonomy: finish Nav2 install (blocker 1), then work blockers
   2–4 toward `navigation.launch.py` + `controller.launch.py` → `/drive`.

