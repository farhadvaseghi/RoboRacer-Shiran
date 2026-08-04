#!/bin/bash
# rr_slam.sh — (re)start async slam_toolbox MAPPING on the real car.
# Frames: odom / base_link / /scan (via ~/rr/slam_params_real.yaml).
# Kills any existing instance first, so the car's pose at launch becomes the
# FRESH map origin (0,0,0). Park on the cross before running, then drive the
# full loop slowly and END with overlap back at the start to close the loop.
export ROS_DOMAIN_ID=7
source /opt/ros/humble/setup.bash 2>/dev/null
PARAMS=/home/roboracer/rr/slam_params_real.yaml

# stop any running instance (fresh map + new origin)
P=$(pgrep -f async_slam_toolbox_node)
if [ -n "$P" ]; then
  echo "[rr_slam] stopping existing instance: $P"
  kill -TERM $P 2>/dev/null; sleep 2
  kill -KILL $(pgrep -f async_slam_toolbox_node) 2>/dev/null
fi

setsid bash -c "source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=7; \
  ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file $PARAMS" \
  >/tmp/slam.log 2>&1 </dev/null &
sleep 6
echo "[rr_slam] started fresh. node: $(pgrep -af async_slam_toolbox_node | head -1)"
