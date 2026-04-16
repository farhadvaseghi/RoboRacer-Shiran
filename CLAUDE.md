# CLAUDE.md — RoboRacer Perception Module

## Project Overview
Perception module for the FAU RoboRacer autonomous racing project.
The perception module has TWO sub-responsibilities, owned by TWO different people:
1. **Sensor Data Processing** (this repo owner) — raw sensor input → detected objects
2. **Environment Modeling** (teammate) — detected objects → structured world model (track map, boundaries, occupancy)

These two sub-modules live in the same package but MUST remain cleanly separated in code.
Part of a 4-person team: Perception (2 people), Estimation, Planning, Control.

## Hardware
- **Computer:** Nvidia Jetson Orin Nano Super Developer Kit (67 TOPS, 8GB unified RAM)
- **OS:** Ubuntu 22.04, ROS 2 Humble Hawksbill
- **LiDAR:** HOKUYO UST-10LX — 2D, 270° FOV, 10m range, 40 Hz, 1081 rays, ±30mm accuracy
- **Camera:** ZED 2i Stereo Camera — RGB + depth + IMU, mounted on top of LiDAR
- **Motor Controller:** TRAMPA VESC 6 MkVI (used by Control teammate, not us)

## Car Dimensions
- Total length: 520 mm (front bumper to rear bumper)
- Wheelbase (front axle to rear axle): 324 mm
- Total width including wheels: 232 mm
- Front bumper width: 198 mm
- Wheel diameter: 100 mm
- Front overhang (front bumper to front axle): 101 mm
- Rear overhang (rear axle to rear bumper): 95 mm
- Front bumper to LiDAR: 144 mm
- Front axle to LiDAR: 43 mm (LiDAR is 43 mm AHEAD of front axle)

## Coordinate Frame
Origin (`base_link`) = **rear axle**, ground level. This is the ROS REP-105 standard for
ground vehicles and matches the f1tenth_system bringup convention (confirmed on real hardware).
Convention: x = forward, y = left, z = up (ROS REP-103).

The geometric center of the car is 165 mm ahead of the rear axle (= 260 mm from front bumper − 95 mm rear overhang).

### Sensor Transforms (relative to base_link = rear axle)
| Frame             | X (m)   | Y (m)   | Z (m)  | Notes                                                     |
|-------------------|---------|---------|--------|-----------------------------------------------------------|
| base_link         |  0.000  |  0.000  | 0.000  | **Rear axle**, ground level                               |
| rear_axle         |  0.000  |  0.000  | 0.000  | Same as base_link (identity — published for teammates)    |
| front_axle        | +0.324  |  0.000  | 0.000  | Wheelbase = 324 mm                                        |
| laser             | +0.270  |  0.000  | +0.110 | HOKUYO UST-10LX — **confirmed from working hardware**     |
| zed_camera_link   | +0.270  | -0.005  | +0.155 | ZED 2i — same x as laser, 45 mm above it, 5 mm right     |

> **ZED note:** x and z are derived (laser + physical offset). Verify physically when mounting.

### Static Transform Publishers (for launch file)
```bash
# LiDAR — confirmed from f1tenth hardware (bringup_launch.py / tf_publisher.py)
ros2 run tf2_ros static_transform_publisher \
  --x 0.270 --y 0.0 --z 0.110 \
  --roll 0 --pitch 0 --yaw 0 \
  --frame-id base_link --child-frame-id laser

# ZED 2i camera
ros2 run tf2_ros static_transform_publisher \
  --x 0.270 --y -0.005 --z 0.155 \
  --roll 0 --pitch 0 --yaw 0 \
  --frame-id base_link --child-frame-id zed_camera_link

# Rear axle = base_link (identity — for teammate convenience)
ros2 run tf2_ros static_transform_publisher \
  --x 0.0 --y 0.0 --z 0.0 \
  --roll 0 --pitch 0 --yaw 0 \
  --frame-id base_link --child-frame-id rear_axle

# Front axle
ros2 run tf2_ros static_transform_publisher \
  --x 0.324 --y 0.0 --z 0.0 \
  --roll 0 --pitch 0 --yaw 0 \
  --frame-id base_link --child-frame-id front_axle
```

### TF Tree
```
base_link  (= rear axle, ground level)
├── rear_axle          (0.000m — identity, same as base_link)
├── front_axle         (+0.324m ahead — wheelbase)
├── laser              (+0.270m ahead, +0.110m above ground — HOKUYO UST-10LX)
└── zed_camera_link    (+0.270m ahead, +0.155m above ground, 5mm right — ZED 2i)
```

## Simulator
Using **f1tenth_gym_ros** for development. It provides `/scan` (LaserScan) but NO camera.
Sim config must match our car:
```yaml
# f1tenth_gym_ros/config/sim.yaml
bridge:
  ros__parameters:
    scan_fov: 4.7124                    # 270° in radians (UST-10LX)
    scan_beams: 1081                     # UST-10LX ray count
    scan_distance_to_base_link: 0.270    # LiDAR x-offset from base_link (rear axle)
    wheelbase: 0.324                     # 324 mm
    width: 0.232                         # 232 mm track width
```

### Sim-to-Real Strategy
- Topic names are identical between sim and real hardware (`/scan` stays `/scan`)
- Launch file has `sim:=true` (LiDAR only) and `sim:=false` (full stack with camera)
- Only changes on hardware: enable camera/fusion nodes, tune parameters, verify TF tree

## Tech Stack
- **Language:** Python 3.10+
- **Framework:** ROS 2 Humble
- **Key libraries:** numpy, opencv-python-headless, scikit-learn (DBSCAN), scipy
- **Camera SDK:** ZED SDK + zed-ros2-wrapper (real hardware only)
- **LiDAR driver:** urg_node (real hardware only)
- **Inference (future):** YOLOv8-nano exported to TensorRT FP16

## Architecture — Sub-Module Separation

**CRITICAL: Sensor Data Processing and Environment Modeling are separate sub-modules.**
They are developed by different people and MUST be kept in separate directories and files.
The interface between them is clearly defined: sensor data processing outputs detected cones,
environment modeling consumes them and builds a world model.

### Directory Structure
```
src/roboracer_perception/
├── __init__.py
├── utils.py                              ← shared math helpers (both sub-modules may use)
│
├── sensor_processing/                    ← SENSOR DATA PROCESSING (your code)
│   ├── __init__.py
│   ├── lidar_processor.py                ← LiDAR filtering, clustering, cone extraction
│   ├── camera_processor.py               ← camera color detection, bounding boxes
│   ├── depth_estimator.py                ← ZED depth lookup for camera detections
│   └── fusion.py                         ← fuse LiDAR + camera into colored 3D cones
│
└── environment_modeling/                 ← ENVIRONMENT MODELING (teammate's code)
    ├── __init__.py
    ├── track_mapper.py                   ← builds track boundary model from cone observations
    ├── cone_tracker.py                   ← temporal tracking / data association across frames
    └── occupancy_grid.py                 ← local occupancy or free-space representation
```

### ROS 2 Nodes (in scripts/)
```
scripts/
├── lidar_processor_node.py               ← sensor processing
├── camera_processor_node.py              ← sensor processing
├── perception_fusion_node.py             ← sensor processing
├── track_mapper_node.py                  ← environment modeling (teammate)
└── cone_tracker_node.py                  ← environment modeling (teammate)
```

### Data Flow Between Sub-Modules
```
═══════════════════════════════════════════════════════════════
  SENSOR DATA PROCESSING (you)
═══════════════════════════════════════════════════════════════

/scan (LaserScan)                /zed/.../image + /zed/.../depth
       │                                    │
       ▼                                    ▼
┌──────────────────┐           ┌─────────────────────────┐
│ LidarProcessor   │           │ CameraProcessor + Depth │
│ filter → DBSCAN  │           │ HSV/YOLO → depth lookup │
│ → circle fit     │           │ → colored 3D cones      │
└────────┬─────────┘           └────────────┬────────────┘
         │ ConeCandidate[]                  │ FusedCone3D[]
         └──────────┬───────────────────────┘
                    ▼
           PerceptionFusion (3D world coord association)
                    │
                    ▼
           /perception/detected_cones  ← INTERFACE between sub-modules

═══════════════════════════════════════════════════════════════
  ENVIRONMENT MODELING (teammate)
═══════════════════════════════════════════════════════════════

           /perception/detected_cones
                    │
         ┌──────────┴──────────┐
         ▼                     ▼
  ┌─────────────┐     ┌────────────────┐
  │ ConeTracker │     │ TrackMapper    │
  │ temporal    │     │ boundary fit   │
  │ association │     │ track model    │
  └──────┬──────┘     └───────┬────────┘
         ▼                    ▼
  /perception/tracked_cones   /perception/track_boundaries
         │                    │
         └────────────────────┘
                    ▼
            To Estimation / Planning modules
```

### Interface Between Sub-Modules
The boundary is the `/perception/detected_cones` topic.
- **Sensor Data Processing publishes** single-frame cone detections (no history, no tracking)
- **Environment Modeling subscribes** and adds temporal tracking, mapping, boundary fitting

DO NOT let sensor processing code import from environment_modeling or vice versa.
The only coupling is through ROS topics (or the future ConeArray message type).

## Full File Structure
```
roboracer_perception/
├── package.xml
├── CMakeLists.txt
├── CLAUDE.md                              ← you are here
├── README.md
├── config/
│   └── perception_params.yaml             ← ALL tunable parameters
├── launch/
│   └── perception.launch.py               ← sim:=true / sim:=false
├── msg/                                   ← future custom messages
│   ├── Cone.msg
│   └── ConeArray.msg
├── scripts/
│   ├── lidar_processor_node.py            ← sensor processing node
│   ├── camera_processor_node.py           ← sensor processing node
│   ├── perception_fusion_node.py          ← sensor processing node
│   ├── track_mapper_node.py               ← environment modeling node (teammate)
│   └── cone_tracker_node.py               ← environment modeling node (teammate)
├── src/roboracer_perception/
│   ├── __init__.py
│   ├── utils.py                           ← shared math helpers
│   ├── sensor_processing/                 ← YOUR CODE (sensor data processing)
│   │   ├── __init__.py
│   │   ├── lidar_processor.py
│   │   ├── camera_processor.py
│   │   ├── depth_estimator.py
│   │   └── fusion.py
│   └── environment_modeling/              ← TEAMMATE'S CODE (environment modeling)
│       ├── __init__.py
│       ├── track_mapper.py
│       ├── cone_tracker.py
│       └── occupancy_grid.py
├── test/
│   ├── test_lidar_processing.py           ← sensor processing tests
│   ├── test_cone_detection.py             ← sensor processing tests
│   ├── test_track_mapper.py               ← environment modeling tests (teammate)
│   └── test_cone_tracker.py               ← environment modeling tests (teammate)
└── rviz/
    └── perception.rviz
```

## How to Run Tests
```bash
# All tests
pytest test/ -v

# Only sensor processing tests (your code)
pytest test/test_lidar_processing.py test/test_cone_detection.py -v

# Specific test
pytest test/test_lidar_processing.py -v -k "test_single_cone"
```

## Important Conventions
- All distances in **meters**, angles in **radians**
- Coordinate frame: x-forward, y-left, z-up (ROS REP-103)
- Cone colors: blue (left boundary), yellow (right boundary), orange (special)
- Config values go in `config/perception_params.yaml`, NEVER hardcoded
- **Sensor processing code goes ONLY in `src/roboracer_perception/sensor_processing/`**
- **Environment modeling code goes ONLY in `src/roboracer_perception/environment_modeling/`**
- **No cross-imports between sensor_processing/ and environment_modeling/**
- Shared utilities (math, geometry) go in `utils.py` at the package root
- Every new module needs corresponding tests in `test/`
- Use DBSCAN for clustering (not the legacy naive BFS)

## Interface Contract with Teammates
Sensor data processing publishes to `/perception/detected_cones`.
Environment modeling subscribes to that and publishes to `/perception/tracked_cones`
and `/perception/track_boundaries` for Estimation and Planning.

Each detected cone contains: position (x,y) in base_link frame, color, confidence, radius.

Future custom message:
```
# msg/Cone.msg
float64 x
float64 y
uint8 color        # 0=unknown, 1=blue, 2=yellow, 3=orange
float64 confidence
float64 radius

# msg/ConeArray.msg
std_msgs/Header header
Cone[] cones
```

## ROS 2 Topics
| Topic | Type | Owner | Direction |
|-------|------|-------|-----------|
| /scan | LaserScan | Hardware/Sim | IN → sensor processing |
| /zed/.../image_rect_color | Image | ZED 2i | IN → sensor processing |
| /zed/.../depth_registered | Image | ZED 2i | IN → sensor processing |
| /perception/detected_cones | MarkerArray | Sensor processing | OUT → environment modeling |
| /perception/tracked_cones | MarkerArray | Environment modeling | OUT → Estimation |
| /perception/track_boundaries | MarkerArray | Environment modeling | OUT → Planning |

## Common Tasks
1. **Tune parameters** → edit `config/perception_params.yaml`
2. **Add detection method** → new file in `sensor_processing/`, new node in `scripts/`, update CMakeLists
3. **Fix a bug** → run `pytest test/ -v` to reproduce, then fix
4. **Improve clustering** → modify `sensor_processing/lidar_processor.py`, use DBSCAN
5. **Add camera calibration** → update FusionConfig in `sensor_processing/fusion.py`
6. **Create custom message** → add .msg in `msg/`, update package.xml and CMakeLists
7. **Switch sim↔real** → change `sim:=true` to `sim:=false` in launch

## Files NOT to Modify Without Team Discussion
- `package.xml` (affects build for everyone)
- Topic names (other modules and environment modeling subscribe to them)
- The interface topic `/perception/detected_cones` (boundary between sub-modules)
- Anything in `environment_modeling/` (that's your teammate's code)

## Performance Targets
- Full sensor processing pipeline: ≥20 Hz (50ms per cycle) on Jetson Orin Nano
- LiDAR processing: <2ms
- Camera detection: <8ms (TensorRT) or <5ms (HSV)
- Fusion: <2ms
