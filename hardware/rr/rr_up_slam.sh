#!/bin/bash
# =====================================================================
# rr_up_slam.sh — bring up the RoboRacer stack with slam_toolbox
# LOCALIZATION on the saved corridor_clean map (instead of AMCL).
# corridor_clean origin = the cross (0,0,0), so map_start_at_dock places
# the car at 0,0,0 on boot with no manual initial-pose needed.
#
# Starts (each its own detached process group):
#   1. Base sensor/drive stack (f1tenth_stack: /scan /odom joystick mux)
#   2. Foxglove bridge  ws://<car-ip>:8765
#   3. slam_toolbox localization -> /map + map->odom TF
# Idempotent; no pkill -f; clears stale shm ONLY at true cold start.
# =====================================================================
export ROS_DOMAIN_ID=7
source /opt/ros/humble/setup.bash

PARAMS=/home/roboracer/rr/localize_slam_real.yaml
OVERLAY=/home/roboracer/f1tenth_ws/install/setup.bash
log(){ echo "[rr_up_slam] $*"; }

if [ "$(date +%Y)" = "1970" ]; then
  log "WARNING: clock is 1970 — fix it BEFORE this launch, never after (TF jump)."
fi

if [ -z "$(pgrep -f 'vesc_driver|foxglove_bridge|slam_toolbox')" ]; then
  log "cold start -> clearing stale FastRTPS shm"
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
else
  log "existing nodes detected -> NOT touching shm"
fi

# 1. base stack
if pgrep -f vesc_driver >/dev/null; then
  log "1/3 base stack: already running"
else
  log "1/3 base stack: starting"
  setsid bash -c "source /opt/ros/humble/setup.bash; source $OVERLAY; \
    export ROS_DOMAIN_ID=7; ros2 launch f1tenth_stack bringup_launch.py" \
    >/tmp/t_stack.log 2>&1 </dev/null &
  sleep 9
fi

# 2. foxglove
if ss -tlnp 2>/dev/null | grep -q :8765; then
  log "2/3 foxglove: already listening on 8765"
else
  log "2/3 foxglove: starting on ws://<car-ip>:8765"
  setsid bash -c "source /opt/ros/humble/setup.bash; source $OVERLAY; \
    export ROS_DOMAIN_ID=7; ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765" \
    >/tmp/foxglove.log 2>&1 </dev/null &
  sleep 6
fi

# 3. slam_toolbox localization on saved map
if pgrep -f slam_toolbox >/dev/null; then
  log "3/3 slam localization: already running"
else
  log "3/3 slam localization: starting (map=corridor_clean, start_at_dock=origin)"
  setsid bash -c "source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=7; \
    ros2 launch slam_toolbox localization_launch.py \
    slam_params_file:=$PARAMS use_sim_time:=false" \
    >/tmp/slamloc.log 2>&1 </dev/null &
  sleep 10
fi

echo
log "================= STATUS ================="
log "base   : joy=$(pgrep -f joy_node|wc -l) vesc=$(pgrep -f vesc_driver|wc -l) urg=$(pgrep -f urg_node|wc -l)"
log "foxglove 8765: $(ss -tlnp 2>/dev/null | grep -q :8765 && echo UP || echo DOWN)"
log "slam_loc: $(pgrep -f slam_toolbox|wc -l)"
log "logs   : /tmp/t_stack.log  /tmp/foxglove.log  /tmp/slamloc.log"
log "=========================================="
