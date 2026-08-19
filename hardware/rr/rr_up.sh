#!/bin/bash
# =====================================================================
# rr_up.sh — bring up ONLY the verified-working RoboRacer stack.
#
# What it starts (each = its own detached process group, so restarting
# one never kills the others):
#   1. Base sensor/drive stack  (f1tenth_stack: /scan, /odom, joystick, mux)
#   2. Foxglove bridge on ws://<car-ip>:8765  (with f1tenth_ws overlay so
#      vesc_msgs schemas resolve)
#   3. AMCL GLOBAL localization  (nav2 map_server -> dense /map  +  amcl,
#      then global-init the particle cloud over the whole map)
#      Params: ~/rr/amcl_global.yaml (TUNED — update_min_d 0.05,
#      max_beams 180, tighter likelihood). Retune: edit that file then
#      run ~/rr/rr_amcl_restart.sh (restarts ONLY the amcl group).
#
# SAFETY RULES baked in (learned the hard way):
#   * Idempotent: checks what is already running and starts ONLY what is
#     missing -> never a duplicate base stack (duplicate VESC kills /odom).
#   * Clears stale /dev/shm/fastrtps_* ONLY at a true cold start (nothing
#     running) -> never yanks shm out from under a live stack.
#   * No `pkill -f` anywhere (it self-matches the running command).
#   * ROS_DOMAIN_ID=7, use_composition:=False (composition hangs stand-alone).
# =====================================================================
# (set -u removed: ROS setup.bash references unbound vars)
export ROS_DOMAIN_ID=7
source /opt/ros/humble/setup.bash

MAP=/home/roboracer/rr_maps/corridor_clean.yaml
AMCL_PARAMS=/home/roboracer/rr/amcl_global.yaml
OVERLAY=/home/roboracer/f1tenth_ws/install/setup.bash

log(){ echo "[rr_up] $*"; }

# --- clock sanity (car has no RTC/NTP; reads 1970 after every boot) -----
if [ "$(date +%Y)" = "1970" ]; then
  log "WARNING: clock is 1970 (dead RTC). Stable, so the stack still works,"
  log "         but do NOT date -s fix it while the base stack runs (the jump"
  log "         breaks its TF). Fix only at a true cold boot, before launch."
fi

# --- cold-start shm clear: ONLY when nothing relevant is running --------
if [ -z "$(pgrep -f 'vesc_driver|foxglove_bridge|nav2_amcl|nav2_map_server')" ]; then
  log "cold start (no nodes running) -> clearing stale FastRTPS shm"
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
else
  log "existing nodes detected -> NOT touching shm"
fi

# --- 1. base sensor/drive stack ----------------------------------------
if pgrep -f vesc_driver >/dev/null; then
  log "1/3 base stack: already running (leaving it — never duplicate)"
else
  log "1/3 base stack: starting"
  setsid bash -c "source /opt/ros/humble/setup.bash; source $OVERLAY; \
    export ROS_DOMAIN_ID=7; ros2 launch f1tenth_stack bringup_launch.py" \
    >/tmp/t_stack.log 2>&1 </dev/null &
  sleep 9
fi

# --- 2. foxglove bridge -------------------------------------------------
if ss -tlnp 2>/dev/null | grep -q :8765; then
  log "2/3 foxglove: already listening on 8765"
else
  log "2/3 foxglove: starting on ws://<car-ip>:8765"
  setsid bash -c "source /opt/ros/humble/setup.bash; source $OVERLAY; \
    export ROS_DOMAIN_ID=7; ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765" \
    >/tmp/foxglove.log 2>&1 </dev/null &
  sleep 6
fi

# --- 3. AMCL global localization (map_server + amcl) -------------------
if pgrep -f nav2_amcl >/dev/null; then
  log "3/3 amcl localization: already running"
else
  log "3/3 amcl localization: starting (map_server dense /map + amcl)"
  setsid bash -c "source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=7; \
    ros2 launch nav2_bringup localization_launch.py map:=$MAP \
    params_file:=$AMCL_PARAMS use_composition:=False use_sim_time:=false" \
    >/tmp/amcl.log 2>&1 </dev/null &
  sleep 15
  log "3/3 amcl: requesting GLOBAL initialization (spread particles over map)"
  ros2 service call /reinitialize_global_localization std_srvs/srv/Empty "{}" >/dev/null 2>&1
fi

# --- status summary -----------------------------------------------------
echo
log "================= STATUS ================="
log "base   : joy=$(pgrep -f joy_node|wc -l) vesc=$(pgrep -f vesc_driver|wc -l) urg=$(pgrep -f urg_node|wc -l)"
log "foxglove 8765: $(ss -tlnp 2>/dev/null | grep -q :8765 && echo UP || echo DOWN)"
log "localize: amcl=$(pgrep -f nav2_amcl|wc -l) map_server=$(pgrep -f nav2_map_server|wc -l)"
log "logs   : /tmp/t_stack.log  /tmp/foxglove.log  /tmp/amcl.log"
log "=========================================="
log "Next: connect Foxglove desktop -> ws://192.168.50.10:8765, then drive"
log "      the car toward a corridor feature and watch the particle cloud."
