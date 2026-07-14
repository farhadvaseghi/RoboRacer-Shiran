#!/usr/bin/env bash
# rr_initpose.sh X Y YAW — tell amcl where the car starts on the map (map frame).
# Example: rr_initpose.sh 0 0 0   (car at the SLAM origin, facing +x)
# Equivalent to RViz "2D Pose Estimate". Coordinates are in METERS, yaw in RAD.
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-7}
source /opt/ros/humble/setup.bash 2>/dev/null
source ~/roboracer_ws/install/setup.bash 2>/dev/null

X=${1:-0.0}; Y=${2:-0.0}; YAW=${3:-0.0}
read QZ QW < <(python3 -c "import math;print(math.sin($YAW/2.0), math.cos($YAW/2.0))")

echo "[rr_initpose] x=$X y=$Y yaw=$YAW  (qz=$QZ qw=$QW)"
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
"{header: {frame_id: 'map'},
  pose: {pose: {position: {x: $X, y: $Y, z: 0.0},
                orientation: {x: 0.0, y: 0.0, z: $QZ, w: $QW}},
         covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0,
                      0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.0685]}}"
