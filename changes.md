Here's the complete list of every change, organized by what's Farhad's code (modified) vs ours (new). One thing first: teleop_key.py was not
  modified at all — it's identical to origin/perception. You can verify with git diff origin/perception -- roboracer_perception/teleop_key.py
  (empty).

  Farhad's track files (modified)

  tools/generate_solid_oval_map.py

  - STRAIGHT_X_MAX: 20.0 → 60.0
  - RIGHT_CENTER: (20.0, 2.5) → (60.0, 2.5)
  - WIDTH: 2560 → 3040

  roboracer_perception/solid_wall_geometry.py

  - STRAIGHT_X_MAX: 20.0 → 60.0 (same as generator)
  - RIGHT_CENTER: (20.0, 2.5) → (60.0, 2.5) (same as generator)

  maps/solid_oval_track.pgm

  - Fully regenerated — new file, 3040×1040 (was 2560×1040), ~3.16 MB. Same oval design, just 3× longer straights.

  roboracer_perception/solid_wall_visualizer.py

  - One line: enumerate(segments) → enumerate(segments, start=1) to fix the "Multiple Markers in same MarkerArray" warning (was colliding with the
  DELETEALL marker at id=0).

  roboracer_perception/moving_obstacle_controller.py

  - if x < 20.0 and y < 2.5: → if x < 60.0 and y < 2.5:
  - tx = min(20.0, x + STRAIGHT_LOOKAHEAD) → tx = min(60.0, ...)
  - elif x >= 20.0 and y <= 5.0: → elif x >= 60.0 ...
  - cx, cy, radius = 20.0, 2.5, 2.5 → cx, cy, radius = 60.0, 2.5, 2.5

  Farhad's gym_bridge files (modified)

  deps/f1tenth_gym_ros/f1tenth_gym_ros/gym_bridge.py

  - Line 527: if recovery_cmd is None and self.ego_requested_speed != 0: → if False:  # Wall slowdown disabled for demo — was hijacking commanded
  speed
  - Effect: the entire LIDAR-based forward-clearance speed-capping block is bypassed. The driver command passes through unmodified.
  Collision-recovery (post-collision reset) still works.

  deps/f1tenth_gym_ros/config/sim_moving_obstacle.yaml

  - sy: -0.6 → -0.3 (ego spawn closer to lane center)
  - sy1: 0.6 → 0.5 (opp spawn closer to lane center)

  Farhad's other files (NOT modified)

  - teleop_key.py — unchanged
  - roboracer_perception/solid_wall_scan_highlighter.py — unchanged
  - roboracer_perception/cone_tracker.py, cone_map_builder.py, perception_fusion.py, etc. — unchanged
  - All Farhad's launch files in deps/f1tenth_gym_ros/launch/ — unchanged (we don't use them; we use our own)

  NEW files we created

  roboracer_perception/sim_camera_walls.py (~200 lines)

  Synthetic ZED 2i feed for the solid_wall scenario. Subscribes to /ego_racecar/odom and /opp_racecar/odom, projects wall segments and the opp
  footprint onto a 1280×720 BGR image, publishes on the real ZED topics.

  roboracer_perception/wall_opponent_detector.py (~180 lines)

  Replaces lidar_processor_pca_node in this scenario. DBSCAN clustering + PCA classifies clusters as WALL (long, elongated) or OPPONENT (small,
  rectangular). Publishes to /perception/walls (red LINE_STRIPs) and /perception/opponent (green CUBE).

  roboracer_perception/pycache/*.pyc

  Deleted stale ones (Python's auto-rebuild handles fresh ones).

  Our own files (modified by us, owned by us)

  launch/solid_wall_perception.launch.py

  - Added tf_base_to_zed static transform
  - Added sim_camera_walls node
  - Added camera_processor_node node (with perception_params.yaml)
  - Replaced lidar_processor_pca_node with wall_opponent_detector
  - Header docstring updated

  setup.py

  - Added entry points: sim_camera_walls, wall_opponent_detector

  CMakeLists.txt

  - Added install(PROGRAMS ...) entries for sim_camera_walls.py and wall_opponent_detector.py

  config/roboracer_sim.rviz

  Many small tweaks across the long debug session:
  - Background Color: 30;30;30 → 200;200;200 (light grey)
  - Fixed Frame: ego_racecar/base_link → map ← this was the critical bug fix
  - LaserScan.Color: 0;255;0 → 255;30;30 (red)
  - LaserScan.Enabled: true (kept on)
  - SolidWalls.Enabled: true (kept on)
  - WallHits.Enabled: false (still off)
  - ObjectsPCA.Enabled: false (disabled — replaced by Walls + Opponent below)
  - Added Walls MarkerArray display → /perception/walls (red lines)
  - Added Opponent MarkerArray display → /perception/opponent (green box)
  - Added OppRobotModel RobotModel display → /opp_robot_description (renders the opp car body)
  - Odometry.Enabled: false (was showing red arrow in front of car)
  - TF.Show Arrows: false, Show Axes: false, Show Names: false
  - View went through 5 iterations, final: ThirdPersonFollower, Target Frame ego_racecar/base_link, Distance 10, Pitch 0.5354, Yaw π — copied from
  Milad's gym_bridge.rviz but with ThirdPersonFollower instead of Orbit to fix the "view shows car from front after bend" bug

  Environment changes (not Farhad's, but system-level)

  - pip install --user opencv-python (4.13.0) — to give a numpy-2-compatible cv2
  - pip install --user 'numpy<2' → numpy 1.26.4 — this was the critical fix for cv_bridge (ROS Humble's apt-installed cv_bridge_boost is built
  against numpy 1.x)

  Files NOT regenerated (potential follow-up)

  - maps/solid_oval_track_obstacles.pgm — still has the old 20m straight geometry. If you switch to a launch that uses this map, the layout will
  mismatch the new 60m logic in solid_wall_geometry.py and moving_obstacle_controller.py. Need to also edit
  tools/generate_solid_oval_obstacles_map.py if you want the obstacles scenario at the new length.

  Run git status from inside ~/roboracer_ws/src/roboracer_perception/ for the authoritative diff against origin/perception.