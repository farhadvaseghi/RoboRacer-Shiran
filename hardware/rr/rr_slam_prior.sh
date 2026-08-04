#!/usr/bin/env bash
# Switch how much slam_toolbox leans on odometry, so the two settings can be
# measured against each other with rr_loc_monitor.py.
#
#   scan  - match often, so odometry only has to bridge 5 cm / 3 deg gaps
#   odom  - the stock setting: match every 30 cm / 17 deg, dead-reckon between
#   show  - print the live values and slam's CPU, change nothing
#
# WHY these three parameters:
#   minimum_travel_distance  how far the car moves before slam re-matches
#   minimum_travel_heading   how far it TURNS before slam re-matches. At the
#                            stock 0.3 rad this is 17 deg -- and heading is the
#                            channel vesc_to_odom fabricates from the steering
#                            COMMAND, so it is the one that drifts worst.
#   minimum_time_interval    the rate ceiling, so a fast car cannot force
#                            matching faster than the CPU can serve.
#
# correlation_search_space_dimension is deliberately NOT touched: a wider
# search only helps when the prior is badly wrong, and matching more often
# makes the prior error smaller, not bigger. Widening it would cost CPU
# quadratically for nothing.
#
# Restarting slam DISCARDS the pose, so this re-seeds via rr_seed_start.py.
#
# Usage:  bash ~/rr/rr_slam_prior.sh {scan|odom|show}

set -u

RR=/home/roboracer/rr
LOGS=/home/roboracer/rr_logs
YAML="$RR/localize_slam_real.yaml"
OVERLAY=/home/roboracer/f1tenth_ws/install/setup.bash
WS=/home/roboracer/roboracer_ws/install/setup.bash
BACKUP="$YAML.bak_prior"

# The bracketed letter keeps this pattern from matching the pkill/pgrep
# process's own command line.
NODEPAT='localization_slam_toolbox_nod[e]'

log(){ echo "[slam-prior] $*"; }

show(){
  log "live values in $(basename "$YAML"):"
  grep -E "minimum_travel_distance|minimum_travel_heading|minimum_time_interval|correlation_search_space_dimension" "$YAML" \
    | sed 's/^/    /'
  local pid
  pid=$(pgrep -f "$NODEPAT" | head -1)
  if [ -n "$pid" ]; then
    log "slam pid $pid  cpu=$(ps -o pcpu= -p "$pid" | tr -d ' ')%  up=$(ps -o etime= -p "$pid" | tr -d ' ')"
  else
    log "slam is NOT running"
  fi
  log "system load:$(cut -d' ' -f1-3 /proc/loadavg | sed 's/^/ /') on $(nproc) cores"
}

apply(){  # $1 travel_distance  $2 travel_heading  $3 time_interval
  [ -f "$BACKUP" ] || { cp -p "$YAML" "$BACKUP"; log "backed up -> $(basename "$BACKUP")"; }
  cp -p "$YAML" "$YAML.bak_$(date +%Y%m%d_%H%M%S)"
  sed -i \
    -e "s/^\( *minimum_travel_distance:\).*/\1 $1/" \
    -e "s/^\( *minimum_travel_heading:\).*/\1 $2/" \
    -e "s/^\( *minimum_time_interval:\).*/\1 $3/" \
    "$YAML"
  log "set travel_distance=$1 travel_heading=$2 time_interval=$3"
}

restart_slam(){
  # Capture the pose BEFORE slam dies. rr_seed_start.py always seeds the map
  # ORIGIN, which would teleport the estimate there if the car is parked
  # anywhere else -- that silently ruins any measurement taken mid-corridor.
  CAPTURED_POSE=$( set +u; source /opt/ros/humble/setup.bash
                   ROS_DOMAIN_ID=7 python3 "$RR/rr_pose_capture.py" 2>/dev/null )
  if [ -n "$CAPTURED_POSE" ]; then
    log "captured live pose: $CAPTURED_POSE  (x y yaw_deg)"
  else
    log "WARNING: could not read the live pose; will fall back to the origin seed"
  fi

  log "stopping slam"
  pkill -9 -f "$NODEPAT" 2>/dev/null
  sleep 2
  if pgrep -f "$NODEPAT" >/dev/null; then
    log "ERROR: slam did not die; aborting before it can be double-started"
    return 1
  fi

  log "starting slam with the new parameters"
  setsid bash -c "source /opt/ros/humble/setup.bash; [ -f '$OVERLAY' ] && source '$OVERLAY'; [ -f '$WS' ] && source '$WS'; export ROS_DOMAIN_ID=7; export FASTRTPS_DEFAULT_PROFILES_FILE=$RR/fastdds_udp_only.xml; exec ros2 run slam_toolbox localization_slam_toolbox_node --ros-args --params-file $YAML -r /map:=/map_posegraph -r /map_metadata:=/map_posegraph_metadata" \
    >"$LOGS/slam_toolbox.log" 2>&1 </dev/null &

  # Wait for the node to actually exist rather than sleeping a fixed guess.
  local waited=0
  while [ "$waited" -lt 40 ]; do
    sleep 1; waited=$((waited + 1))
    pgrep -f "$NODEPAT" >/dev/null && break
  done
  if ! pgrep -f "$NODEPAT" >/dev/null; then
    log "ERROR: slam did not come up in ${waited}s -- see $LOGS/slam_toolbox.log"
    return 1
  fi
  log "slam up after ${waited}s"
  sleep 3   # let it finish loading the posegraph before the seed lands
  return 0
}

reseed(){
  # ROS's setup.bash reads unbound variables, so 'set -u' must be off while it
  # is sourced or the seed dies before it publishes anything.
  if [ -n "${CAPTURED_POSE:-}" ]; then
    log "re-seeding at the captured pose, NOT the origin"
    ( set +u; source /opt/ros/humble/setup.bash
      ROS_DOMAIN_ID=7 python3 "$RR/seed_pose.py" $CAPTURED_POSE 2>&1 ) | sed 's/^/    /'
  else
    log "re-seeding at the map ORIGIN -- only correct if the car is parked at the start"
    ( set +u; source /opt/ros/humble/setup.bash
      ROS_DOMAIN_ID=7 python3 "$RR/rr_seed_start.py" 2>&1 ) | sed 's/^/    /'
  fi
  sleep 2
  if grep -aq "Localizing to" "$LOGS/slam_toolbox.log"; then
    log "seed accepted: $(grep -a 'Localizing to' "$LOGS/slam_toolbox.log" | tail -1)"
  else
    log "WARNING: no 'Localizing to' in the slam log -- the seed may not have landed"
  fi
}

case "${1:-show}" in
  scan) apply 0.05 0.05 0.1 && restart_slam && reseed && show ;;
  odom) apply 0.3  0.3  0.2 && restart_slam && reseed && show ;;
  show) show ;;
  *)    echo "usage: $0 {scan|odom|show}"; exit 2 ;;
esac
