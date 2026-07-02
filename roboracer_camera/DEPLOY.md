# roboracer_camera — deploy to the car

Adds the ZED camera as a **second obstacle source for the Nav2 costmap**. The
camera publishes `/camera_scan` (a LaserScan of obstacles standing above/below
the LiDAR slice); Nav2 already consumes it via the one `camera` observation
source added to `nav2_params_real.yaml`. Nothing else in the stack changes.

Runtime chain (only the ZED wrapper + our node are new):

```
~/t_stack.sh ─ /scan, /odom, TF ─┐
ZED wrapper ─ /zed/.../depth ─► depth_to_scan ─ /camera_scan ─┤
                                                              ├─► obstacle_layer
autonomous_real.launch.py ─ amcl + planner + controller ──────┘   (scan + camera)
```

> This is additive on top of the LiDAR-only autonomous flow in
> `hardware/guide.md`. Get that driving a clean slow lap **first**; then add the
> camera. If anything misbehaves, delete `camera` from `observation_sources` in
> `nav2_params_real.yaml` and you are back to the LiDAR-only stack.

---

## The internet constraint

Only **one** step needs the internet: installing the ZED SDK + wrapper **on the
car** (Phase A). Everything else is offline file-copy over the `roboracer` Wi-Fi
(no uplink needed). To avoid ever needing the car online *and* SSH at once, do
the downloads on a machine that has internet, then carry the files to the car on
a USB stick or `scp` them over the local Wi-Fi.

---

## Phase A — install the ZED SDK + wrapper on the car (one time)

The ZED wrapper is what publishes the depth topic; without it our node has no
input. Two ways:

### A1. Offline (recommended given the constraint)

On any internet machine:

1. Download the **ZED SDK for JetPack 6 (L4T 36.x, CUDA 12)** `.run` installer
   from stereolabs.com/developers. (Match the car: Ubuntu 22.04, JetPack 6 /
   L4T R36.4 — see `docs/roboracer-architecture.md`.)
2. Clone the wrapper matching ROS 2 Humble:
   ```bash
   git clone --recurse-submodules https://github.com/stereolabs/zed-ros2-wrapper.git
   ```
3. Carry both to the car (USB stick, or over the local Wi-Fi):
   ```bash
   scp ZED_SDK_Tegra_*.run zed-ros2-wrapper -r roboracer@192.168.50.10:~/zed_install/
   ```

On the car (SSH; no internet needed for the SDK `.run` itself — it bundles what
it needs on top of the JetPack CUDA already present):
```bash
cd ~/zed_install
chmod +x ZED_SDK_Tegra_*.run && ./ZED_SDK_Tegra_*.run       # accept defaults
mkdir -p ~/zed_ws/src && cp -r zed-ros2-wrapper ~/zed_ws/src/
cd ~/zed_ws
source /opt/ros/humble/setup.bash
# rosdep needs internet ONCE; if offline, most deps (image_transport, etc.) are
# already on the car from f1tenth_stack — try the build and apt-install only
# what it complains is missing:
colcon build --symlink-install
source install/setup.bash
```

### A2. Online (if the car's hotspot is stable enough)

SSH in with the car on its internet Wi-Fi (`wlP1p1s0` → hotspot, see
`docs/session.md`), then run the SDK installer and `rosdep install` normally.
This is the path that was flaky last session — A1 is safer.

**Verify the camera publishes** (after either path):
```bash
source ~/zed_ws/install/setup.bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
# in another terminal:
ros2 topic list | grep -i zed         # note the EXACT depth + camera_info names
ros2 topic hz /zed/zed_node/depth/depth_registered
```
Write down the exact topic names — you pass them to our node in Phase C if they
differ from the defaults.

---

## Phase B — deploy our package + config (offline, over local Wi-Fi)

From this repo (branch `camera-costmap`) copy two things to the car. No internet
needed — this is the `roboracer` LAN.

```bash
# 1) the new package -> the car workspace src
scp -r roboracer_camera roboracer@192.168.50.10:~/roboracer_ws/src/

# 2) the edited Nav2 params (adds the 'camera' obstacle source)
scp roboracer_estimation/config/nav2_params_real.yaml \
    roboracer@192.168.50.10:~/roboracer_ws/src/RoboRacer-Shiran/roboracer_estimation/config/
```
> Adjust the destination path if the repo on the car is laid out differently —
> confirm with `ls ~/roboracer_ws/src` first. If the car deploys via the PC
> `deploy.sh`, also drop the new `nav2_params_real.yaml` into that deploy folder
> so it isn't overwritten next deploy.

Build on the car (SSH, offline):
```bash
source /opt/ros/humble/setup.bash
cd ~/roboracer_ws
colcon build --packages-select roboracer_camera roboracer_estimation
source install/setup.bash
ros2 pkg executables roboracer_camera        # -> roboracer_camera depth_to_scan
```

---

## Phase C — run order (each session)

Everything below is on the car (SSH is fine; no internet needed to *run*).

```bash
# T1 — sensors + drive stack (as today)
~/t_stack.sh

# T2 — ZED driver
source ~/zed_ws/install/setup.bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i

# T3 — our camera -> /camera_scan  (+ the base_link->camera_scan static TF)
source ~/roboracer_ws/install/setup.bash
ros2 launch roboracer_camera camera.launch.py
#   if Phase A showed different topic names, pass them:
#   ros2 launch roboracer_camera camera.launch.py \
#       depth_topic:=/zed2i/zed_node/depth/depth_registered \
#       info_topic:=/zed2i/zed_node/rgb/camera_info

# T4 — the autonomous stack (unchanged command; now reads scan + camera)
source ~/roboracer_ws/install/setup.bash
ros2 launch roboracer_estimation autonomous_real.launch.py \
    map:=/home/roboracer/rr_maps/track2.yaml
# then init pose + goal exactly as in hardware/guide.md §5
```

### Verify the camera is actually feeding the costmap
```bash
ros2 topic hz /camera_scan                       # should publish steadily
ros2 topic echo /camera_scan --once              # finite ranges where obstacles are
# put a box on a chair (above LiDAR height) in front of the car -> it should
# appear as lethal cells in the local costmap in RViz, and the planner routes
# around it. Remove it -> cells clear within a second or two.
```

---

## On-site tuning (expect to touch these once, on the real floor)

Params are on `depth_to_scan` (set via `--ros-args -p name:=value` or edit
`camera.launch.py`):

- **Dark corridor floor leaking in as obstacles** → raise `ground_z_min`
  (0.08 → 0.12) and/or `range_min`.
- **Textureless white walls give sparse depth** → expected and fine; the LiDAR
  owns the walls. If you *want* more fill, set the ZED depth mode to `NEURAL`
  in the wrapper config.
- **Camera mounted with a downward pitch** → the flat-ground assumption biases
  height; symptom is the floor appearing at range or real obstacles vanishing.
  Lower `ceiling_z_max` / raise `ground_z_min` as a quick fix, or add a pitch
  term later.
- **Too jumpy / false positives** → raise `min_pts` (2 → 4).
- **CPU too high on the Jetson** → raise `stride` (2 → 3) to subsample more.
- **Wrong FOV** → set `angle_min`/`angle_max` to the real ZED horizontal FOV.

## Rollback

Camera off, LiDAR-only stack back instantly: in `nav2_params_real.yaml` change
both `observation_sources: scan camera` back to `observation_sources: scan`
(or just don't launch T2/T3). No rebuild of other packages needed.
