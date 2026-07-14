#!/usr/bin/env bash
# rr_record.sh [name] — rosbag the full autonomous chain for post-run debugging.
# Records sensors, TF, localization, plan, commands, diagnostics. One bag per run.
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-7}
source /opt/ros/humble/setup.bash 2>/dev/null
source ~/roboracer_ws/install/setup.bash 2>/dev/null

NAME="${1:-auto}"
TS=$(date +%Y%m%d_%H%M%S)
OUT="$HOME/rr_logs/bag_${NAME}_${TS}"
echo "[rr_record] -> $OUT   (Ctrl-C to stop)"
exec ros2 bag record -o "$OUT" \
  /scan /odom /tf /tf_static /map \
  /amcl_pose /particle_cloud /initialpose /goal_pose \
  /plan /local_plan /cmd_vel /cmd_vel_nav \
  /drive /teleop /commands/motor/speed /commands/servo/position \
  /diagnostics /rosout
