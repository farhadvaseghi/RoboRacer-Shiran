# F1TENTH Simulator ROS2 Migration Log

This file records what was migrated and debugged from the start of this process to make the project run on **Ubuntu 22.04 + ROS2 Humble**.

---

## 1) Initial Problem

The build failed with:

- `fatal error: ros/ros.h: No such file or directory`

This showed that the source code was still ROS1 (`roscpp`) while the workspace/build flow was ROS2 (`colcon`, Humble).

---

## 2) Core Migration Work

### 2.1 Node API migration (ROS1 -> ROS2)

Migrated all runtime nodes from ROS1 APIs to ROS2 `rclcpp`:

- `node/simulator.cpp`
- `node/mux.cpp`
- `node/behavior_controller.cpp`
- `node/random_walk.cpp`
- `node/keyboard.cpp`

Main conversions applied:

- `#include <ros/ros.h>` -> `#include <rclcpp/rclcpp.hpp>`
- ROS1 message headers -> ROS2 message headers (`.../msg/...`)
- `ros::NodeHandle` -> `rclcpp::Node` usage
- `getParam` -> `declare_parameter` + `get_parameter`
- `advertise` -> `create_publisher`
- `subscribe` -> `create_subscription`
- `createTimer` -> `create_wall_timer`
- `ros::Time::now()` -> `this->now()`
- `ros::init/spin` -> `rclcpp::init/spin/shutdown`

### 2.2 Parameter file migration

Converted `params.yaml` from ROS1 flat format to ROS2 parameter-file format:

- Added top-level `/**:`
- Added `ros__parameters:` block

### 2.3 Build and package updates

Updated build/dependency metadata for ROS2:

- `CMakeLists.txt` already ROS2-oriented and used for `ament_cmake` + `colcon`
- `package.xml` adjusted to include required ROS2 runtime dependencies

### 2.4 Launch system updates

Updated `launch/simulator.launch.py`:

- Use `xacro` command expansion for robot description
- Added map server lifecycle transitions using timed `ros2 lifecycle set` commands (`configure`, `activate`) so `/map` becomes available under ROS2
- Set node outputs to `screen` for easier debugging

---

## 3) Build Validation and Fixes

### 3.1 First successful ROS2 build

Ran `colcon build --symlink-install` in WSL/Humble and confirmed successful package build.

### 3.2 C++17 compatibility cleanup

Replaced C++20-style designated initializers in migrated code (warnings under C++17):

- `node/simulator.cpp`
- `node/behavior_controller.cpp`

After this cleanup, build completed cleanly again on Humble.

---

## 4) RViz and Runtime Debugging

### 4.1 RViz plugin class mismatch fix

Initial RViz error:

- `rviz/Orbit ... does not exist`

Cause: old ROS1 RViz config class names.

Fix in `launch/simulator.rviz`:

- Converted `rviz/...` plugin IDs to ROS2 equivalents (`rviz_common/...`, `rviz_default_plugins/...`)

### 4.2 Robot model path fix

Robot model was not visible although map/environment showed.

Fixes in `launch/simulator.rviz`:

- `Robot Description` corrected to ROS2 path/name
- Set RobotModel to topic source explicitly:
  - `Description Source: Topic`
  - `Description Topic: /robot_description`

### 4.3 OpenGL/WSL shader issue mitigation

From logs, RViz had GLSL shader link errors (graphics stack issue, common on WSL/Mesa).

Mitigation added in `launch/simulator.launch.py`:

- RViz launched with `LIBGL_ALWAYS_SOFTWARE=1`

This forces software rendering for compatibility.

---

## 5) Logging Improvements

To make debugging easier:

- Enabled `output='screen'` for launched nodes in `launch/simulator.launch.py`
- Recommended launch with debug + log capture:

```bash
ros2 launch f1tenth_simulator simulator.launch.py --debug 2>&1 | tee ~/rviz_debug.log
```

---

## 6) Documentation Updates

Added/updated migration docs:

- `README.md` (ROS2 Humble note for this workspace)
- `ROS2_MIGRATION.md`
- `ROS2_QUICK_START.md`
- `migration.md` (this file)

---

## 7) Current Status

- Package builds with `colcon` on ROS2 Humble.
- ROS2 nodes launch and run.
- Map server lifecycle handled in launch.
- RViz config converted to ROS2 plugin IDs.
- Robot model source settings corrected.
- Additional WSL graphics compatibility setting added.

If the racecar model is still not visible on a specific machine, remaining issue is likely local graphics/driver behavior in RViz rather than ROS1/ROS2 API mismatch.

---

## 8) Additional Debug/Fix Session (This Chat)

This section records the follow-up debugging and migration hardening done after the above baseline worked.

### 8.1 Revert of temporary troubleshooting changes

Some temporary changes made during live debugging were reverted on request to return to baseline behavior:

- Removed aggressive RViz GL/QT env overrides from `launch/simulator.launch.py`
- Restored RViz `Map` display to enabled state in `launch/simulator.rviz`
- Removed temporary global `--log-level error` launch arguments from all nodes

### 8.2 Migration hardening fixes applied

After a full ROS1->ROS2 scan, additional migration-sensitive fixes were applied.

#### a) ROS2 map QoS + topic consistency in simulator node

File: `node/simulator.cpp`

- Changed map publisher/subscriber QoS to ROS2 map-server-compatible QoS:
  - `reliable`
  - `transient_local`
  - depth `1`
- Stopped hardcoding `"/map"` in publisher; now uses parameterized `map_topic`
- Ensured map subscription uses the same QoS profile for robust map reception

Why: ROS2 map data is often latched (`transient_local`), and mismatched durability can cause missing/unstable map behavior in RViz or downstream consumers.

#### b) TF frame ID normalization

File: `node/simulator.cpp`

- Replaced ROS1-style `"/map"` frame assignment with `map_frame_`

Why: Leading slash frame IDs are a common ROS1 carry-over and can cause TF inconsistencies in ROS2 tools.

#### c) Legacy launch compatibility cleanup

Files:

- `launch/simulator.launch`
- `launch/racecar_model.launch`

Updates:

- Converted/remapped legacy ROS1 XML launch usage to ROS2-compatible launch behavior
- `simulator.launch` now includes `simulator.launch.py` so accidental use of the XML launch entry no longer breaks in ROS2 environments

---

### 8.3 RViz diagnostics instrumentation added

File: `launch/simulator.launch.py`

Added runtime diagnostics to expose why RViz displays are red:

- RViz now starts with debug logger level:
  - `--ros-args --log-level rviz2:=debug`
- Added timed diagnostics block that prints:
  - all configured RViz display topics
  - `ros2 topic info -v` for each topic (or missing-topic message)
  - TF check for `map -> base_link`

Purpose: Make it explicit which display topics are unavailable and whether TF exists at runtime.

---

### 8.4 Findings from diagnostics run

From collected diagnostics output:

- Healthy topics:
  - `/map` (published)
  - `/scan` (published)
  - `/robot_description` (published)
- Missing/unavailable topics (red RViz items expected):
  - `/racecar_sim/update`
  - `/dynamic_viz`
  - `/env_viz`
  - `/static_viz`
  - `/tree_lines`
  - `/tree_nodes`
  - `/waypoint_vis`
  - `/path_lines`
  - `/converted_scan`
- `/smoothed_path` exists by type but had no active publisher during test

Interpretation: Most red displays are currently pointing to optional visualization/planner topics that are not being published by active nodes in this run configuration.

TF note:

- TF check initially reported `map` frame unavailable, then showed transform output shortly after startup.
- This indicates timing/startup ordering effects rather than a permanent TF graph failure.

---

### 8.5 Current practical status after this session

- Core simulator ROS2 stack launches and runs.
- Migration-sensitive QoS/frame/launch compatibility issues were further reduced.
- RViz red display causes are now observable in logs instead of opaque.
- Remaining red displays correspond primarily to currently-unpublished optional topics rather than ROS1 API leftovers.
