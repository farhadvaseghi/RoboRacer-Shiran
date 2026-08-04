#!/usr/bin/env bash
# Restart ONLY nav2 (planner + costmaps + bt_navigator), leaving slam, the map
# server, the controller and the base stack up.
#
# Needed after editing nav2_params_real.yaml: these servers read their
# parameters once, in configure(), so a live 'ros2 param set' is accepted and
# then silently ignored.
#
# Detached, so an ssh drop cannot kill it mid-restart.
#
# NOTE on the kill patterns: every name is bracketed so pkill cannot match its
# own command line, and the lifecycle manager is matched by its FULL node name
# -- a bare 'lifecycle_manager' would also kill map_clean_lifecycle and take the
# map down with it.
#
# Usage:  bash ~/rr/rr_restart_nav2.sh

set +u
RR=/home/roboracer/rr
LOGS=/home/roboracer/rr_logs
WS=/home/roboracer/roboracer_ws/install/setup.bash
NAV_PARAMS=/home/roboracer/roboracer_ws/src/RoboRacer-Shiran/roboracer_estimation/config/nav2_params_real.yaml

echo "[nav2-restart] stopping nav2"
for pat in "navigation_launc[h]" "controller_serve[r]" "planner_serve[r]" \
           "behavior_serve[r]" "bt_navigato[r]" "smoother_serve[r]" \
           "velocity_smoothe[r]" "waypoint_followe[r]" \
           "lifecycle_manager_navigatio[n]"; do
  pkill -9 -f "$pat" 2>/dev/null
done
sleep 3

still=$(pgrep -cf "bt_navigato[r]")
if [ "$still" != "0" ]; then
  echo "[nav2-restart] ERROR: nav2 did not die (bt_navigator x$still); aborting"
  exit 1
fi

echo "[nav2-restart] starting nav2 with $NAV_PARAMS"
setsid bash -c "source /opt/ros/humble/setup.bash; [ -f '$WS' ] && source '$WS'; export ROS_DOMAIN_ID=7; export FASTRTPS_DEFAULT_PROFILES_FILE=$RR/fastdds_udp_only.xml; exec stdbuf -oL -eL ros2 launch nav2_bringup navigation_launch.py use_composition:=False use_sim_time:=false params_file:=$NAV_PARAMS" \
  >"$LOGS/nav.log" 2>&1 </dev/null &

# Wait for the servers themselves rather than a log marker: ROS block-buffers
# stdout to a file, so readiness markers can arrive long after nav2 is healthy.
waited=0
while [ "$waited" -lt 90 ]; do
  sleep 2; waited=$((waited + 2))
  if [ "$(pgrep -cf 'bt_navigato[r]')" != "0" ] && \
     [ "$(pgrep -cf 'planner_serve[r]')" != "0" ]; then
    echo "[nav2-restart] nav2 processes up after ${waited}s"
    break
  fi
done

sleep 5
echo "[nav2-restart] failure_tolerance now in use:"
grep -E "^\s*failure_tolerance:" "$NAV_PARAMS" | sed 's/^/    /'
echo "[nav2-restart] planner: $(pgrep -cf 'planner_serve[r]')  bt: $(pgrep -cf 'bt_navigato[r]')  controller_server: $(pgrep -cf 'controller_serve[r]')"
echo "[nav2-restart] NOTE: costmaps start empty; give a 2D Pose Estimate before the first goal."
