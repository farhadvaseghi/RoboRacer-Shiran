# RoboRacer (F1TENTH) — System Architecture

A reference map of what runs on the car, how it reads its sensors (inputs), how
it decides, and how it drives the motor and steering (outputs). Captured by
inspecting the live car on 2026-06-30.

> TL;DR: It's a standard **F1TENTH** autonomous race car running **ROS 2 Humble**
> on an **NVIDIA Jetson Orin Nano Super**. A LiDAR + wheel odometry feed a
> particle-filter localizer; a command **multiplexer** arbitrates between
> joystick (manual) and autonomy, and the chosen drive command is converted to
> **VESC** motor + servo signals.

---

## 1. Hardware

| Part | Detail |
| --- | --- |
| Compute | NVIDIA Jetson Orin Nano (Super) Dev Kit, 6-core ARM, 7.4 GB RAM |
| OS | Ubuntu 22.04.5 LTS, kernel 5.15.148-tegra (JetPack 6 / L4T R36.4) |
| Middleware | ROS 2 **Humble** (`/opt/ros/humble`) |
| LiDAR | Hokuyo URG (networked) at `192.168.0.10:10940` over Ethernet |
| Motor controller | **VESC** over USB serial (`/dev/sensors/vesc` → `/dev/ttyACM0`) |
| Drivetrain | Brushless motor (throttle) + steering servo, wheelbase **0.25 m** |
| Input device | USB gamepad/joystick (Logitech F710-style) |
| GPU use | CUDA 12.6 available; `range_libc` for fast LiDAR raycasting |

---

## 2. Network topology

The Jetson is **multi-homed** on three networks at once:

| Interface | Address | Network | Purpose |
| --- | --- | --- | --- |
| `wlx7419f816d21d` (USB Wi-Fi) | `192.168.50.10/24` | `roboracer` Wi-Fi | **SSH / control** — how you connect |
| `eno1` (onboard Ethernet) | `192.168.0.15/24` | wired LiDAR net | Talks to the Hokuyo LiDAR at `.10` |
| `wlP1p1s0` (internal Wi-Fi) | `100.87.x` (DHCP) | upstream Wi-Fi | Internet access |

- **The "static IP" box + LAN cable** you described is the **`eno1` / `192.168.0.x`
  segment**. A NetworkManager profile literally named **`Hokoyu`** pins `eno1`
  to a fixed address so the car can always find the LiDAR at `192.168.0.10`.
- SSH in over the `roboracer` Wi-Fi: `ssh roboracer@192.168.50.10`.
- Ping to the car fails by design (ICMP filtered) — judge reachability by SSH /
  port 22, not ping.

---

## 3. Software workspaces (in `~`)

| Workspace / dir | What it is |
| --- | --- |
| `f1tenth_ws` | **The real-car driver stack.** Packages: `f1tenth_system` (drivers + bringup), `particle_filter` (localization) |
| `sim_ws` | Simulation — `f1tenth_gym_ros` bridge to the F1TENTH Gym |
| `autoware` | Autoware autonomy stack (sourced in `.bashrc`; experimental) |
| `test_ws` | Scratch / experiments |
| `~/build`, `~/install`, `~/log` | A second colcon build (mux + teleop tools + `vesc_msgs`) |
| `*.pgm / *.yaml / *.posegraph` | Saved SLAM maps (`mymap`, `reglungstechnik_corridor`, `map_1779107126`) |

**Launcher:** `~/t_stack.sh` sources ROS 2 + `f1tenth_ws` and runs
`ros2 launch f1tenth_stack bringup_launch.py` — the real-car stack
(VESC + LiDAR + joystick + mux).

---

## 4. The core driver stack (`bringup_launch.py`)

Nodes started, grouped by role:

**Sensing (inputs)**
- `urg_node` — Hokuyo LiDAR driver → publishes **`/scan`** (`LaserScan`)
- `joy` (joy_node) — reads the USB gamepad → **`/joy`** (`Joy`)
- `vesc_driver_node` — talks to the VESC over USB; reads motor/servo feedback

**Command handling (decision / arbitration)**
- `joy_teleop` — turns `/joy` into a drive command on **`/teleop`**
- `ackermann_mux` — multiplexes drive sources by priority → forwards chosen command
- `throttle_interpolator` — smoother, **disabled** (commented out in launch)

**Actuation (outputs)**
- `ackermann_to_vesc_node` — drive command → VESC motor + servo signals
- `vesc_to_odom_node` — VESC feedback → **`/odom`** + `odom`→`base_link` TF

**Transforms**
- `static_transform_publisher` — fixed `base_link`→`laser` at **[0.27, 0.0, 0.11] m**
  (LiDAR sits 27 cm forward, 11 cm up from the rear axle)

---

## 5. Data flow: inputs → processing → outputs

```
                 SENSORS (inputs)                       ACTUATORS (outputs)
   Hokuyo LiDAR ──/scan──┐                      ┌─ /commands/motor/speed ─→ motor
   (192.168.0.10)        │                      │
   USB gamepad ──/joy─→ joy_teleop ─/teleop─┐   │  /commands/servo/position ─→ steering
                                            ▼   │            ▲
                              ┌──────────────────────┐       │
   autonomy/planner ─/drive─→│    ackermann_mux     │       │
                             │  picks highest prio   │   ackermann_to_vesc
                             └──────────┬────────────┘       ▲
                                        └── ackermann_cmd ────┘
   VESC feedback ─→ vesc_to_odom ─/odom─┐
                                        ▼
   /scan + /odom + map ─→ particle_filter ─→ pose estimate + map→odom TF
```

**Step by step**

1. **Inputs.** The LiDAR publishes `/scan`; the VESC's wheel feedback becomes
   `/odom` (via `vesc_to_odom`, using wheelbase 0.25 m); the gamepad publishes
   `/joy`.
2. **Localization.** `particle_filter` (Monte-Carlo localization, 4000 particles)
   fuses `/scan` + `/odom` against a saved map to estimate where the car is, and
   publishes the `map`→`odom` transform.
3. **Decision.** Autonomy (an Autoware or custom planner) consumes the pose +
   `/scan` and publishes drive commands on **`/drive`**. The joystick path
   publishes on **`/teleop`**.
4. **Arbitration.** `ackermann_mux` listens to both and forwards the
   **highest-priority active** source as `ackermann_cmd`:
   - `joystick` → topic `teleop`, **priority 100** (wins)
   - `navigation` → topic `drive`, **priority 10**
   - each with a 0.2 s timeout (a stale source is dropped).
   So **the joystick always overrides autonomy** — that's the safety/override.
5. **Outputs.** `ackermann_to_vesc` converts the chosen `AckermannDriveStamped`
   into VESC signals: `/commands/motor/speed` (eRPM) and
   `/commands/servo/position` (0–1). `vesc_driver` sends them over USB to the
   hardware → wheels spin and steer.

---

## 6. Key topics

| Topic | Type | Produced by → Consumed by |
| --- | --- | --- |
| `/scan` | `sensor_msgs/LaserScan` | urg_node → particle_filter, planner |
| `/joy` | `sensor_msgs/Joy` | joy_node → joy_teleop |
| `/teleop` | `ackermann_msgs/AckermannDriveStamped` | joy_teleop → mux (prio 100) |
| `/drive` | `ackermann_msgs/AckermannDriveStamped` | **autonomy → mux (prio 10)** |
| `ackermann_cmd` | `ackermann_msgs/AckermannDriveStamped` | mux → ackermann_to_vesc |
| `/commands/motor/speed` | `std_msgs/Float64` | ackermann_to_vesc → vesc_driver |
| `/commands/servo/position` | `std_msgs/Float64` | ackermann_to_vesc → vesc_driver |
| `/odom` | `nav_msgs/Odometry` | vesc_to_odom → particle_filter, planner |

---

## 7. Key parameters & conversions

**VESC** (`config/vesc.yaml`)
- Speed → motor eRPM: `eRPM = 4532 × speed(m/s)` (offset 0)
- Steering angle → servo: `servo = −0.8825 × angle(rad) + 0.4715`
- Servo limits: `0.188 … 0.78`; speed limit ±9250 eRPM (≈ ±2.0 m/s)

**Joystick teleop** (`config/joy_teleop.yaml`)
- `human_control`: **deadman = button 4** (hold to drive), speed scale 2.0 m/s,
  steering scale 0.34 rad. (Default profile has zero scale = no motion.)

**Localization** (`particle_filter/config/localize.yaml`)
- Inputs `/scan` + `/odom`; 4000 particles; range method `rm`; map
  `map_1779107126`; publishes odometry/pose.

---

## 8. How to use it

- **Launch the real-car stack:** `~/t_stack.sh`
  (= `ros2 launch f1tenth_stack bringup_launch.py`).
- **Drive manually:** hold the deadman button (4) on the gamepad and use the
  sticks.
- **Drive autonomously:** publish `ackermann_msgs/AckermannDriveStamped` to
  **`/drive`** — the mux forwards it whenever the joystick isn't overriding.
- **Localize / start autonomy:** also launch
  `ros2 launch particle_filter localize_launch.py` (needs a map) plus the
  planner.
- **Inspect at runtime** (after launch): `ros2 node list`, `ros2 topic list`,
  `ros2 topic echo /scan`, `ros2 topic hz /odom`.

---

## 9. Notes & open questions

- **Multi-homing:** three default routes exist (LiDAR Ethernet, roboracer
  Wi-Fi, internet Wi-Fi). Lowest metric currently wins for general traffic;
  on-link traffic to each subnet is unaffected.
- The **"Raspberry-looking box" on the LAN cable** maps to the `192.168.0.x`
  LiDAR segment (`eno1`, NM profile `Hokoyu`). Worth confirming physically
  whether it's the LiDAR itself, a PoE injector, or a small switch/router.
- `autoware` is installed and auto-sourced — likely the intended autonomy
  brain, but not started by `t_stack.sh`. Confirm which planner publishes
  `/drive`.
- `throttle_interpolator` (accel smoothing) is **disabled**; raw commands go
  straight to the VESC.

