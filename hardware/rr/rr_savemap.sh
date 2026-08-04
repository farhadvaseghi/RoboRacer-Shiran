#!/bin/bash
# rr_savemap.sh [NAME] — save the current slam_toolbox map to ~/rr_maps/NAME.*
# Saves BOTH the nav2 occupancy grid (.pgm + .yaml, for map_server/AMCL) and the
# posegraph (.data + .posegraph, to resume SLAM / localization mode).
# Default NAME=corridor_clean. Origin (0,0,0) = where SLAM was started (the cross).
export ROS_DOMAIN_ID=7
source /opt/ros/humble/setup.bash 2>/dev/null
NAME=${1:-corridor_clean}
DIR=/home/roboracer/rr_maps
mkdir -p "$DIR"

echo "[rr_savemap] occupancy grid -> $DIR/$NAME.pgm / .yaml"
timeout -s KILL 30 ros2 service call /slam_toolbox/save_map \
  slam_toolbox/srv/SaveMap "{name: {data: '$DIR/$NAME'}}"

echo "[rr_savemap] posegraph -> $DIR/$NAME.data / .posegraph"
timeout -s KILL 40 ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph "{filename: '$DIR/$NAME'}"

echo "[rr_savemap] files:"; ls -la "$DIR/$NAME".* 2>/dev/null
