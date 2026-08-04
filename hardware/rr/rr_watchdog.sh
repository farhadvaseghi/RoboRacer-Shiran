#!/bin/bash
# rr_watchdog.sh - keep foxglove_bridge alive + log lap-recording health.
# Every few seconds: if the foxglove bridge (ws :8765) is down, relaunch it;
# log foxglove/controller state and how many points the lap recorder has captured.
# Stop with:  kill -INT $(cat ~/rr_logs/watchdog.pid)
set +u
export ROS_DOMAIN_ID=7
source /opt/ros/humble/setup.bash
[ -f ~/f1tenth_ws/install/setup.bash ] && source ~/f1tenth_ws/install/setup.bash 2>/dev/null
[ -f ~/roboracer_ws/install/setup.bash ] && source ~/roboracer_ws/install/setup.bash 2>/dev/null

LOG=~/rr_logs/watchdog.log
CSV=${1:-~/rr_maps/lap_line.csv}
echo "[watchdog] start $(date) watching foxglove + $CSV" >> "$LOG"

while true; do
  ts=$(date +%H:%M:%S)
  if ss -ltn 2>/dev/null | grep -q ':8765'; then
    fg=UP
  else
    setsid bash -c "source /opt/ros/humble/setup.bash; [ -f ~/f1tenth_ws/install/setup.bash ] && source ~/f1tenth_ws/install/setup.bash 2>/dev/null; export ROS_DOMAIN_ID=7; exec ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765" >~/rr_logs/foxglove_wd_restart.log 2>&1 </dev/null &
    fg="DOWN->RESTARTED"
  fi
  ctrl=$(pgrep -f pure_pursuit_controller >/dev/null && echo UP || echo DOWN)
  rec=$(pgrep -f rr_record_path >/dev/null && echo UP || echo DOWN)
  pts=$(( $(wc -l < "$CSV" 2>/dev/null || echo 1) - 1 ))
  echo "$ts foxglove=$fg controller=$ctrl recorder=$rec lap_points=$pts" >> "$LOG"
  sleep 3
done
