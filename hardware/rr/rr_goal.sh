#!/usr/bin/env bash
# rr_goal.sh X Y YAW — send the END point (navigation goal) in the map frame.
# Example: rr_goal.sh 5.0 1.0 0
# Equivalent to RViz "2D Goal Pose" / "Nav2 Goal". Meters + radians.
# The car will plan a path from its current (amcl) pose to here and drive it.
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-7}
source /opt/ros/humble/setup.bash 2>/dev/null
source ~/roboracer_ws/install/setup.bash 2>/dev/null

if [ "$#" -lt 2 ]; then echo "usage: rr_goal.sh X Y [YAW]"; exit 1; fi
X=$1; Y=$2; YAW=${3:-0.0}
read QZ QW < <(python3 -c "import math;print(math.sin($YAW/2.0), math.cos($YAW/2.0))")

echo "[rr_goal] goal x=$X y=$Y yaw=$YAW"
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'map'},
  pose: {position: {x: $X, y: $Y, z: 0.0},
         orientation: {x: 0.0, y: 0.0, z: $QZ, w: $QW}}}"
