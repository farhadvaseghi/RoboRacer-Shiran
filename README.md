<div align="center">

# 🏁 RoboRacer-Shiran

**An autonomous F1TENTH-style race car — taken from simulator to physical track.**

ROS 2 Humble · Nav2 · Pure Pursuit · EKF Fusion · SLAM · ZED Vision

[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/Platform-Jetson%20Orin%20Nano%20Super-76B900?logo=nvidia&logoColor=white)](docs/roboracer-architecture.md)
[![Simulator](https://img.shields.io/badge/Sim-f1tenth__gym__ros-blue)](deps/f1tenth_gym_ros)
[![Status](https://img.shields.io/badge/status-active-success.svg)](changes.md)

[Quick Start](#-quick-start-simulation) · [Architecture](#%EF%B8%8F-architecture) · [Packages](#-packages) · [Real Car](#-real-hardware) · [Field Tools](#-field-tools) · [Docs](#-documentation)

</div>

---

## 📑 Table of Contents

| | Section | | Section |
|:---:|---|:---:|---|
| 📖 | [Overview](#-overview) | 🎮 | [Simulation Scenarios](#-simulation-scenarios) |
| ✨ | [Feature Highlights](#-feature-highlights) | 🚗 | [Real Hardware](#-real-hardware) |
| ⚡ | [Quick Start (Simulation)](#-quick-start-simulation) | 🛠️ | [Field Tools](#-field-tools) |
| 🏗️ | [Architecture](#%EF%B8%8F-architecture) | 📡 | [Key Topics](#-key-topics) |
| 📦 | [Packages](#-packages) | 🧰 | [Troubleshooting](#-troubleshooting) |
| 🗺️ | [SLAM Mapping](#%EF%B8%8F-slam-mapping) | 🌿 | [Branch Strategy](#-branch-strategy) |
| | | 📚 | [Documentation](#-documentation) |

---

## 📖 Overview

RoboRacer-Shiran is a five-member university project building a **1/10th-scale autonomous race car**. The same codebase drives a patched `f1tenth_gym_ros` simulator **and** the physical car (Jetson Orin Nano Super + Hokuyo LiDAR + VESC + ZED 2i camera):

```text
        SIMULATION                          REAL CAR
   f1tenth_gym_ros sim              Jetson Orin Nano Super
     LiDAR + odom scan            Hokuyo LiDAR + VESC + ZED 2i
            │                                │
            └──────────┬─────────────────────┘
                       ▼
     EKF fusion → Nav2 planning → Pure Pursuit → Ackermann /drive
```

- **Estimation** — adaptive-covariance odometry + `robot_localization` EKF, `slam_toolbox` mapping and on-car localization (AMCL is also supported).
- **Planning** — Nav2 with SMAC Hybrid-A\* planner; the route is driven as one continuous `NavigateThroughPoses` goal.
- **Control** — custom C++ **Pure Pursuit** controller (the Stanley controller was removed); reverse handling, goal-stop, RViz debug markers.
- **Vision** — ZED depth converted to a `/camera_scan` LaserScan, intended as a **second obstacle source** for the Nav2 costmap alongside LiDAR (implemented; not yet integrated/demonstrated on the car).

<div align="right"><a href="#-table-of-contents">back to top ⬆️</a></div>

---

## ✨ Feature Highlights

| ✅ | Subsystem | What it does |
|:---:|---|---|
| 🏎️ | **Pure Pursuit controller** | C++ path tracker with speed control, reverse mode, cross-track/heading error telemetry, and RViz debug markers |
| 🧭 | **Nav2 planning** | SMAC Hybrid-A\* planner + behavior trees, tuned `nav2_params.yaml` for sim and real car |
| 📊 | **EKF state estimation** | Adaptive covariance on simulator odometry; real-car fusion of ZED IMU/odom + VESC odometry (`ekf_real.yaml`) |
| 🗺️ | **SLAM mapping** | `slam_toolbox` async mapping — no prior map needed, drive the track once and save |
| 📷 | **ZED camera costmap** | Depth points → `/camera_scan` → Nav2 obstacle layer, to catch obstacles the 2-D LiDAR plane misses (implemented; not yet integrated/demonstrated on the car) |
| 🚧 | **Dynamic scenarios** | Patched two-agent simulator with autonomous moving-obstacle vehicle, static obstacle maps, cylinder-wall visualization |
| 🛑 | **Field-tested tooling** | One-command bring-up, chain healthcheck with per-link reports, rosbag recording, zero-speed `/drive` keepalive for parked localization |
| 🏟️ | **Real-car runbooks** | Verified SLAM → save map → AMCL → autonomous drive workflow, plus one-script deploy to the car |

<div align="right"><a href="#-table-of-contents">back to top ⬆️</a></div>

---

## ⚡ Quick Start (Simulation)

**Prerequisites** — Ubuntu 22.04 · ROS 2 Humble · Python 3.10+

```bash
sudo apt install python3-colcon-common-extensions python3-rosdep \
    ros-humble-nav2-map-server ros-humble-nav2-lifecycle-manager \
    ros-humble-rviz2 ros-humble-robot-state-publisher \
    ros-humble-xacro ros-humble-rqt-image-view
python3 -m pip install transforms3d numpy opencv-python-headless scikit-learn scipy
```

**1. Clone and build** the workspace:

```bash
mkdir -p ~/ros2_ws/src
git clone -b Hardware https://github.com/farhadvaseghi/RoboRacer-Shiran.git ~/ros2_ws/src/RoboRacer-Shiran
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

**2. Launch the stack** — three terminals, source the workspace in each (`source ~/ros2_ws/install/setup.bash`):

| Terminal | Command | Starts |
|:---:|---|---|
| 1 | `ros2 launch roboracer_estimation sim.launch.py` | Simulator, RViz, adaptive covariance, EKF |
| 2 | `ros2 launch roboracer_estimation navigation.launch.py` | Nav2 + SMAC planner + path relay |
| 3 | `ros2 launch roboracer_control controller.launch.py` | Pure Pursuit controller |

**3. Drive** — use RViz's **Nav2 Goal** tool to pick a target. Recommended order: simulator → navigation → controller → goal. Because `/control/plan` is retained, sending the goal before the controller starts also works.

> ⚠️ The stack uses **wall time** (`use_sim_time:=false`) — this simulator configuration does not publish `/clock`.

<div align="right"><a href="#-table-of-contents">back to top ⬆️</a></div>

---

## 🏗️ Architecture

```text
/ego_racecar/odom
        │
        ▼
adaptive_covariance_node ──► /estimation/sim_odom_adaptive
        │
        ▼
ekf_filter_node ──► /odometry/filtered ─────────────┐
                                                     │
Nav2 (SMAC Hybrid-A*) ──► /plan ──► path_relay_node  │
                              │                       │
                              ▼                       ▼
                       /control/plan ──► pure_pursuit_controller
                                                     │
                                                     ▼
                                            /drive (AckermannDriveStamped)
                                                     │
                                                     ▼
                                             simulator bridge
```

- Nav2's built-in `controller_server` stays part of the navigation action, but its velocity output is **isolated** on `/navigation/cmd_vel` — only the custom Pure Pursuit commands the vehicle.
- `path_relay_node` republishes `/plan` as a retained `/control/plan`, so the controller may start before or after a goal is sent.

**Real-car hardware** (full details in [`docs/roboracer-architecture.md`](docs/roboracer-architecture.md)):

| Part | Detail |
|---|---|
| Compute | NVIDIA Jetson Orin Nano (Super) Dev Kit — 6-core ARM, 7.4 GB RAM, JetPack 6 |
| LiDAR | Hokuyo URG, networked at `192.168.0.10:10940` over Ethernet, ~40 Hz `/scan` |
| Motor controller | VESC over USB serial (`/dev/sensors/vesc`), ~50 Hz wheel odometry |
| Camera | ZED 2i — depth → `/camera_scan`, visual odometry + IMU for the EKF |
| Drivetrain | Brushless motor + steering servo, wheelbase 0.324 m |
| Input | USB gamepad (Logitech F710-style) for manual driving & SLAM mapping |

<div align="right"><a href="#-table-of-contents">back to top ⬆️</a></div>

---

## 📦 Packages

| Package | Language | Contents |
|---|:---:|---|
| [`roboracer_estimation`](roboracer_estimation/) | Python | Simulator integration, adaptive covariance, EKF launch & config, Nav2 config, maps, SLAM launches, path relay |
| [`roboracer_control`](roboracer_control/) | C++ | Custom Pure Pursuit path-tracking controller |
| [`roboracer_camera`](roboracer_camera/) | Python | ZED depth→scan node, opponent & person detectors, emergency brake, sim test launches |
| [`deps/f1tenth_gym_ros`](deps/f1tenth_gym_ros/) | Python/C++ | Vendored, **patched** simulator: collision recovery, wall slowdown, two-agent scenarios |
| [`deps/f1tenth_gym`](deps/f1tenth_gym/) | Python | Vendored dynamics backend (`f110_gym`) |
| [`hardware/`](hardware/) | Bash/Python | Real-car deploy scripts, `t_stack.sh` base bring-up, the `rr_*` field toolkit |
| [`docs/`](docs/) | Markdown | Architecture reference, sim→hardware bring-up log, session reports |

Key config files:

| File | Purpose |
|---|---|
| [`roboracer_estimation/config/ekf_sim.yaml`](roboracer_estimation/config/ekf_sim.yaml) | Simulator EKF (`ego_racecar/odom`, `ego_racecar/base_link`) |
| [`roboracer_estimation/config/ekf_real.yaml`](roboracer_estimation/config/ekf_real.yaml) | Real-hardware ZED + VESC sensor fusion |
| [`roboracer_estimation/config/nav2_params.yaml`](roboracer_estimation/config/nav2_params.yaml) | Nav2 planner, costmap, behavior tree (sim) |
| [`roboracer_estimation/config/nav2_params_real.yaml`](roboracer_estimation/config/nav2_params_real.yaml) | Nav2 tuned for the physical car |
| [`roboracer_control/config/controller_params.yaml`](roboracer_control/config/controller_params.yaml) | Vehicle geometry, speeds, lookahead, steering limits |

<div align="right"><a href="#-table-of-contents">back to top ⬆️</a></div>

---

## 🗺️ SLAM Mapping

Before navigating, you need a map of the track. `slam_mapping` runs the simulator with a single ego car + `slam_toolbox` in async mode — the map is built from scratch, no prior map published.

**Terminal 1 — mapping launch:**

```bash
ros2 launch roboracer_estimation slam_mapping.launch.py
```

**Terminal 2 — teleop** (drive the full track at least once so the loop closes):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Terminal 3 — save the map** once it looks complete in RViz:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_track
```

Copy the resulting `.pgm`/`.yaml` into `roboracer_estimation/maps/` and point `ROBORACER_MAP_NAME` at the new map.

<div align="right"><a href="#-table-of-contents">back to top ⬆️</a></div>

---

## 🎮 Simulation Scenarios

The vendored simulator ships several scenario families (see [`HOWTO.md`](HOWTO.md) for the full list — flat-wall and cylinder-wall visualization variants of each):

| Launch (from `f1tenth_gym_ros`) | Scenario |
|---|---|
| `gym_bridge_launch.py` | Original `oval_track` |
| `gym_bridge_solid.launch.py` | Custom `solid_oval_track` — 2.85 m straights / 2.90 m curves |
| `gym_bridge_solid_obstacles.launch.py` | + two static circular obstacles (r = 0.40 m) |
| `gym_bridge_solid_moving_obstacle.launch.py` | + second vehicle driving the oval autonomously (you control ego only) |
| `gym_bridge_solid_static_moving_obstacles.launch.py` | Static obstacles **and** the dynamic opponent together |
| `*_cylinder.launch.py` variants | Same collision maps + raised cylinder-wall markers and wall-hit LiDAR highlighting in RViz |

Teleop for scenario testing (`W`/`S`/`A`/`D`, `Space` = stop, `Q` = quit):

```bash
ros2 run roboracer_perception teleop_key
```

<div align="right"><a href="#-table-of-contents">back to top ⬆️</a></div>

---

## 🚗 Real Hardware

The sim stack runs on the physical car with the same packages — captured live in [`docs/roboracer-hardware-bringup.md`](docs/roboracer-hardware-bringup.md) and [`hardware/guide.md`](hardware/guide.md):

```text
~/t_stack.sh ── /scan, /odom, TF odom→base_link→laser, ackermann_mux
      │
autonomous_real.launch.py:
   map_server ── /map ────────────────┐
   amcl ── TF map→odom, /amcl_pose ───┤  localization
      │                               │
   Nav2 planner ── /plan ─► path relay ─┤  planning
      │                                  │
   pure_pursuit_controller ── /drive ───┘  control
```

**Workflow:** SLAM the track (drive manually) → save the map → AMCL localization → Nav2 plans → Pure Pursuit drives. Nav2's default Humble behavior trees (`NavigateToPose` / `NavigateThroughPoses`) are used; a fresh pose estimate re-initializes localization.

**Deploy config to the car** (from the PC, passwordless SSH already set up):

```bash
cd hardware && bash deploy.sh
```

**Standalone real-hardware estimation** (ZED IMU/odometry + VESC odom):

```bash
ros2 launch roboracer_estimation estimation.launch.py is_sim:=false use_sim_time:=false
```

**Camera as a second obstacle source** ([`roboracer_camera/DEPLOY.md`](roboracer_camera/DEPLOY.md)):

```text
ZED wrapper ─ /zed/.../depth ─► depth_to_scan ─► /camera_scan ─► Nav2 obstacle layer
```

<div align="right"><a href="#-table-of-contents">back to top ⬆️</a></div>

---

## 🛠️ Field Tools

Single-purpose bring-up and diagnosis scripts (in [`hardware/rr/`](hardware/rr)), built and verified during on-car sessions:

| Tool | Purpose |
|---|---|
| `rr_up.sh` | Bring up the verified-working stack (each node in its own process group) |
| `rr_up_slam.sh` | Same, but with `slam_toolbox` localization on the saved map instead of AMCL |
| `rr_localize_run.sh` | One command to localize the car on the saved map + self-check (read `LOCALIZATION.md` first) |
| `rr_localize.sh` | Localize with slam_toolbox — run after `~/t_stack.sh` |
| `rr_initpose.sh X Y YAW` | Tell AMCL where the car starts (= RViz "2D Pose Estimate") |
| `rr_amcl_global.sh` | Global place-anywhere localization; drive toward a corner to converge |
| `rr_amcl_restart.sh` | Restart only the AMCL group (map_server + amcl + lifecycle) without touching the base stack |
| `rr_goal.sh X Y YAW` | Send a Nav2 goal from the CLI (= RViz "2D Goal Pose") |
| `rr_keep.sh` / `rr_keep_stop.sh` | Zero-speed `/drive` keepalive so VESC keeps publishing odom + TF at rest — stop it before sending a goal |
| `rr_healthcheck.sh` | Verify every link of the autonomous chain, one bottleneck at a time → report in `~/rr_logs/` |
| `rr_record.sh` | Rosbag the full autonomous chain (sensors, TF, plan, commands) — one bag per run |
| `rr_fix_joy.sh` | Re-apply the joy_teleop fix after any base-stack relaunch or rebuild |

<div align="right"><a href="#-table-of-contents">back to top ⬆️</a></div>

---

## 📡 Key Topics

| Topic | Type | Direction | Purpose |
|---|---|:---:|---|
| `/ego_racecar/odom` | `nav_msgs/Odometry` | In | Raw simulator odometry |
| `/odometry/filtered` | `nav_msgs/Odometry` | In | EKF vehicle state (controller input) |
| `/plan` → `/control/plan` | `nav_msgs/Path` | In | Nav2 path, retained by the relay |
| `/drive` | `ackermann_msgs/AckermannDriveStamped` | **Out** | Vehicle speed & steering command |
| `/tracking_error` | `geometry_msgs/Vector3Stamped` | Out | Cross-track and heading errors |
| `/control_debug_markers` | `visualization_msgs/MarkerArray` | Out | RViz controller visualization |
| `/navigation/cmd_vel` | `geometry_msgs/Twist` | — | Nav2's own controller output (isolated, unused) |
| `/camera_scan` | `sensor_msgs/LaserScan` | In (real car) | ZED-depth obstacles for the costmap |

**Quick diagnostics:**

```bash
ros2 topic echo /odometry/filtered --once
ros2 topic echo /control/plan --once
ros2 topic info /drive -v      # expect publisher: /pure_pursuit_controller, subscriber: /bridge
```

<div align="right"><a href="#-table-of-contents">back to top ⬆️</a></div>

---

## 🧰 Troubleshooting

| Symptom | Fix |
|---|---|
| Controller runs but no `/drive` | Verify both `/control/plan` and `/odometry/filtered` carry messages |
| Second robot invisible in moving-obstacle scenario | Re-run the setup so the patched RViz config is refreshed |
| Cylinder walls don't collide in 3-D | Expected — collision is 2-D from the occupancy grid; cylinders are an RViz overlay |
| `ModuleNotFoundError: f110_gym` | `python3 -m pip install -e ~/ros2_ws/src/f1tenth_gym/` |
| RViz blank / TF errors | Rebuild with `colcon build --symlink-install --packages-ignore f110_gym` and re-source |

Full sim troubleshooting: [`HOWTO.md`](HOWTO.md)

The Pure Pursuit terminal logs `mode`, pose (`x`, `y`, `yaw`), `progress`, `cte`/`heading_err`, `steer`, and `v_ref`/`v_meas`/`v_cmd` — if it prints `Received path with ...` but no `Goal reached`, check the two input topics first.

<div align="right"><a href="#-table-of-contents">back to top ⬆️</a></div>

---

## 🌿 Branch Strategy

Each subsystem is developed on its own branch so five people can work in parallel; `Hardware` is the integration branch that runs on the physical car.

| Branch | Focus |
|---|---|
| `Hardware` ⭐ | **Integration branch — real-car bring-up, Nav2 tuning, field tooling** (this is the branch to clone) |
| `perception` / `sensor-processing` | PCA LIDAR wall detection, ZED color detection, simulator scenario tooling |
| `Control` / `Control-V2` | Controller development |
| `Estimation` / `Estimation-V2` | EKF and state estimation |
| `Planning-V2` / `planner-setup` | Nav2 planning work |
| `dynamic-overtaking` / `dynamic-overtaking+slam` | Overtaking research + SLAM integration |
| `camera-costmap` | Camera→costmap obstacle injection |
| `hw-freeze` | Frozen snapshot of the confirmed-working hardware state (2026-08-04) |
| `dev` / `master` | Scratch and default branch |

<div align="right"><a href="#-table-of-contents">back to top ⬆️</a></div>

---

## 📚 Documentation

| Doc | Contents |
|---|---|
| [`HOWTO.md`](HOWTO.md) | Running the simulator: setup, all scenario launches, teleop, test scripts, troubleshooting |
| [`changes.md`](changes.md) | Precise record of modifications vs. upstream (who changed what) |
| [`docs/roboracer-architecture.md`](docs/roboracer-architecture.md) | Live-car reference: hardware, network topology, node map |
| [`docs/roboracer-hardware-bringup.md`](docs/roboracer-hardware-bringup.md) | Sim → hardware bring-up log: SLAM → map → localization → control |
| [`hardware/guide.md`](hardware/guide.md) | Runbook: re-map with SLAM, then drive start→end autonomously via Nav2 |
| [`roboracer_camera/DEPLOY.md`](roboracer_camera/DEPLOY.md) | Deploying the ZED camera-scan node to the car |
| [`docs/ssh-connection-guide.md`](docs/ssh-connection-guide.md) | Connecting to the car over the `roboracer` Wi-Fi |
| [`docs/session_report_*.md`](docs/) | Dated session reports from on-car testing |

<div align="right"><a href="#-table-of-contents">back to top ⬆️</a></div>

---

## 🤝 Team

Five-member project team — package ownership stays separated so each subsystem can be developed and tested independently.

<div align="center">

| Contributor | GitHub | Subsystem |
|:---:|:---:|---|
| <img src="https://github.com/sadeghshoushtari.png?size=80" width="60" height="60" alt="M. Shoushtaridehshal"/> | [**@sadeghshoushtari**](https://github.com/sadeghshoushtari) | Perception (LiDAR/camera, AEB) |
| <img src="https://github.com/farhadvaseghi.png?size=80" width="60" height="60" alt="F. Vaseghi"/> | [**@farhadvaseghi**](https://github.com/farhadvaseghi) | Environmental modelling & dynamic overtaking |
| <img src="https://github.com/MiladBahariQaragoz.png?size=80" width="60" height="60" alt="M. Bahari Qaragoz"/> | [**@MiladBahariQaragoz**](https://github.com/MiladBahariQaragoz) | Estimation & localization |
| <img src="https://github.com/kazhalshirvani.png?size=80" width="60" height="60" alt="K. Shirvani"/> | [**@kazhalshirvani**](https://github.com/kazhalshirvani) | Planning & SLAM |
| <img src="https://github.com/mohammadbrd.png?size=80" width="60" height="60" alt="M. Barabadi"/> | [**@mohammadbrd**](https://github.com/mohammadbrd) | Control |

</div>

## 📜 License

Distributed under the MIT License — see [`LICENSE`](LICENSE).

<div align="center">

---

**RoboRacer-Shiran** · built with ROS 2 Humble, Nav2, and a lot of track time 🏁

[↑ Back to top](#-table-of-contents)

</div>
