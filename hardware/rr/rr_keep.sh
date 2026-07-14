#!/bin/bash
# rr_keep.sh — start a zero-speed /drive keepalive (20 Hz) so the VESC keeps
# emitting /odom + odom->base_link TF at rest (needed for slam localization
# while parked). Runs from a file so pgrep -f 'topic pub /drive' can't self-match.
# STOP IT before sending a Nav2 goal (rr_keep_stop.sh) — else it fights Nav2 on /drive.
export ROS_DOMAIN_ID=7
source /opt/ros/humble/setup.bash 2>/dev/null
if pgrep -f 'topic pub /drive' >/dev/null; then
  echo "keeper already running: $(pgrep -f 'topic pub /drive')"; exit 0
fi
setsid bash -c 'source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=7; \
  exec ros2 topic pub /drive ackermann_msgs/msg/AckermannDriveStamped "{drive: {speed: 0.0}}" -r 20' \
  >/tmp/zerodrive.log 2>&1 </dev/null &
sleep 1
echo "keeper started: $(pgrep -f 'topic pub /drive')"
