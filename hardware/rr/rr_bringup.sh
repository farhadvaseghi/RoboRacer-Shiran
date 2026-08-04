#!/bin/bash
# =============================================================================
# rr_bringup.sh - ONE-SHOT bring-up of the RoboRacer autonomy stack on the car,
#                 using the TEAM'S CUSTOM pure_pursuit controller (not Nav2's).
#
# Idempotent: starts only what is missing. No `pkill -f`. Clears FastRTPS shm
# ONLY at a true cold start. Everything runs on ROS_DOMAIN_ID=7.
#
# What it sets up (all the per-session fixes we used to do by hand):
#   1.  Clock         - fixes the 1970 RTC (sudo; asks for the time if needed).
#   2.  Base stack    - f1tenth_stack: LiDAR /scan, VESC /odom, joystick, mux.
#   3.  Joystick fix  - rr_fix_joy.sh: LB(4)=deadman/e-stop, /teleop silent hands-off.
#   4.  Foxglove      - bridge on ws://<car-ip>:8765 (desktop app connects here).
#   5.  Clean map     - despeckles corridor_clean -> corridor_despeck (if missing)
#                       and serves it on /map_clean (view it in Foxglove).
#   6.  Localization  - slam_toolbox localization on corridor_clean -> /map + map->odom.
#   7.  Nav2 PLANNER  - navigation_launch.py (planner + costmaps). Its controller runs
#                       but is ISOLATED (no cmd_vel_to_ackermann) - custom ctrl drives.
#                       Global costmap static layer already points at /map_clean.
#   8.  plan bridge   - plan_qos_relay: /plan -> /control/plan (QoS the controller wants).
#   9.  odom bridge   - map_odom_relay: map->base_link + /odom -> /odometry/map (map-frame pose).
#   10. CUSTOM CTRL   - roboracer_control/pure_pursuit_controller -> /drive (slow real params).
#   11. auto-keeper   - rr_autokeep.py: zero /drive at rest, auto-pauses while driving
#                       (no more manual keeper handoff before each goal).
#   12. reset-on-pose - rr_costmap_reset.py: on every 2D Pose Estimate (/initialpose)
#                       clears global+local costmaps, cancels the active nav goal, and
#                       empties /plan + /control/plan (controller drops the stale path).
#
# Usage:   RR_UTC="2026-07-16 15:30:00" ~/rr/rr_bringup.sh      (non-interactive clock)
#     or   ~/rr/rr_bringup.sh            (prompts for the time only if clock is 1970)
# =============================================================================
set +u
export ROS_DOMAIN_ID=7
# UDP-only DDS: the shared-memory transport on this machine poisons itself
# and leaves later-started nodes deaf. See rr/fastdds_udp_only.xml.
export FASTRTPS_DEFAULT_PROFILES_FILE="/home/roboracer/rr/fastdds_udp_only.xml"

RR="$HOME/rr"
MAPS="$HOME/rr_maps"
LOGS="$HOME/rr_logs"
OVERLAY="$HOME/f1tenth_ws/install/setup.bash"
WS="$HOME/roboracer_ws/install/setup.bash"
NAV_PARAMS="$HOME/roboracer_ws/src/RoboRacer-Shiran/roboracer_estimation/config/nav2_params_real.yaml"
CTRL_PARAMS="$RR/controller_params_real.yaml"
mkdir -p "$LOGS"

source /opt/ros/humble/setup.bash
[ -f "$OVERLAY" ] && source "$OVERLAY"
[ -f "$WS" ] && source "$WS"

log(){ echo "[bringup] $*"; }
up(){ pgrep -f "$1" >/dev/null 2>&1; }

# start a detached, fully-sourced background node if its pattern is not running
spawn(){  # $1 = match pattern (for idempotency + log name), $2 = command
  local pat="$1"; local cmd="$2"
  if up "$pat"; then log "already up: $pat"; return; fi
  log "start: $pat"
  local logf="$LOGS/$(echo "$pat" | tr '/ ' '__').log"
  setsid bash -c "source /opt/ros/humble/setup.bash; [ -f '$OVERLAY' ] && source '$OVERLAY'; [ -f '$WS' ] && source '$WS'; export ROS_DOMAIN_ID=7; export FASTRTPS_DEFAULT_PROFILES_FILE=/home/roboracer/rr/fastdds_udp_only.xml; exec $cmd" >"$logf" 2>&1 </dev/null &
}

# ---------------------------------------------------------------- 1. CLOCK ----
if [ "$(date +%Y)" = "1970" ]; then
  if [ -z "$RR_UTC" ]; then
    read -r -p "[bringup] Clock is 1970. Enter current UTC (YYYY-MM-DD HH:MM:SS): " RR_UTC
  fi
  log "setting clock -> UTC '$RR_UTC' (sudo may prompt)"
  sudo date -u -s "$RR_UTC" && log "clock now: $(date)"
else
  log "clock OK ($(date)) - not touching it"
fi

# ------------------------------------------------- 2. cold-start shm clear ----
if [ -z "$(pgrep -f 'vesc_driver|slam_toolbox|foxglove_bridge|pure_pursuit_controller')" ]; then
  log "cold start -> clearing stale FastRTPS shm"
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
else
  log "nodes already running -> NOT clearing shm"
fi

# ------------------------------------------- 2.5 wait for the gamepad ----
# joy_node grabs the controller only at startup, so the gamepad must be ON
# BEFORE the base stack launches. Wait up to ~30s (non-fatal).
if [ -e /dev/input/js0 ]; then
  log "gamepad already connected (js0)"
else
  log ">>> TURN ON THE GAMEPAD now (waiting up to 30s for /dev/input/js0) <<<"
  for _i in $(seq 1 15); do [ -e /dev/input/js0 ] && break; sleep 2; done
  [ -e /dev/input/js0 ] && log "gamepad connected" \
    || log "WARNING: no gamepad detected - LB e-stop will NOT work this session"
fi

# ---------------------------------------- 2.7 enforce VESC calibration ----
# Physical calibration (measured with a ruler, 2026-07-16): steering servo CENTER
# (car drifted ~30 cm left over 2.2 m at steering=0) and ODOM scale (odom read 2.005 m
# for a real 2.22 m). Forced into vesc.yaml BEFORE the base stack reads it, so a
# colcon rebuild / config revert can never silently lose the calibration.
STEER_OFFSET="0.499"      # was 0.4715 (uncalibrated)
ERPM_GAIN="3690.0"        # RECALIBRATED 2026-08-01: odom read 2.709/2.694 m for a TAPED
                          # 3.000 m (10% under), so 4100 x 2.7015/3.000. The 07-23 value of
                          # 4100 had drifted. Gain sits on BOTH sides via the /**: wildcard
                          # (v_odom = erpm/gain, erpm_cmd = gain*v_cmd), so this fixes the
                          # reported speed AND the real speed together.
for vy in "$HOME/f1tenth_ws/install/f1tenth_stack/share/f1tenth_stack/config/vesc.yaml" \
          "$HOME/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/vesc.yaml"; do
  [ -f "$vy" ] || continue
  sed -i -E "s/^([[:space:]]+)steering_angle_to_servo_offset:[[:space:]].*/\1steering_angle_to_servo_offset: $STEER_OFFSET/" "$vy"
  sed -i -E "s/^([[:space:]]+)speed_to_erpm_gain:[[:space:]].*/\1speed_to_erpm_gain: $ERPM_GAIN/" "$vy"
  # rr_gyro_odom owns odom->base_link now, so vesc_to_odom must NOT publish it too, and its
  # servo-command heading must stay off: measured 2026-08-01 at ~250 deg of error per lap.
  sed -i -E "s/^([[:space:]]+)publish_tf:[[:space:]].*/\1publish_tf: false/" "$vy"
  sed -i -E "s/^([[:space:]]+)use_servo_cmd_to_calc_angular_velocity:[[:space:]].*/\1use_servo_cmd_to_calc_angular_velocity: false/" "$vy"
done
log "VESC calibration enforced (steering_offset=$STEER_OFFSET, speed_to_erpm_gain=$ERPM_GAIN, vesc odom TF OFF)"

# ------------------------------------------------------------ 3. base stack ----
if up vesc_driver; then
  log "base stack already up"
else
  log "start: base stack (f1tenth_stack bringup)"
  setsid bash -c "source /opt/ros/humble/setup.bash; [ -f '$OVERLAY' ] && source '$OVERLAY'; export ROS_DOMAIN_ID=7; export FASTRTPS_DEFAULT_PROFILES_FILE=/home/roboracer/rr/fastdds_udp_only.xml; exec ros2 launch f1tenth_stack bringup_launch.py" >"$LOGS/base.log" 2>&1 </dev/null &
  sleep 10
fi

# ------------------------------------- 3.5 ensure joy_node grabbed the gamepad ----
# Handles re-runs / a controller connected after joy_node already started: if js0
# exists but joy_node holds no /dev/input/event* device, restart joy_node so it grabs it.
if [ -e /dev/input/js0 ]; then
  jn=$(pgrep -f 'lib/joy/joy_node' | head -1)
  if [ -n "$jn" ] && ! ls -l /proc/"$jn"/fd 2>/dev/null | grep -q '/dev/input/event'; then
    log "joy_node has no input device -> restarting it so it grabs the gamepad"
    kill "$jn" 2>/dev/null; sleep 2
    setsid bash -c "source /opt/ros/humble/setup.bash; [ -f '$OVERLAY' ] && source '$OVERLAY'; export ROS_DOMAIN_ID=7; export FASTRTPS_DEFAULT_PROFILES_FILE=/home/roboracer/rr/fastdds_udp_only.xml; exec ros2 run joy joy_node --ros-args -r __node:=joy --params-file $HOME/f1tenth_ws/install/f1tenth_stack/share/f1tenth_stack/config/joy_teleop.yaml" >"$LOGS/joy.log" 2>&1 </dev/null &
    sleep 3
  else
    log "joy_node holding a gamepad device (ok)"
  fi
fi

# ------------------------------------------------- 3.6 gyro odometry (odom TF) ----
# vesc_to_odom fabricated heading from the steering COMMAND; a return-to-marks loop on
# 2026-08-01 measured 12.7 m of position error and ~250 deg of heading error over a 39 m
# lap. This publishes odom->base_link using the real VESC gyro instead (vesc.yaml above
# turns the old publisher off). Must be up BEFORE slam, which needs that transform.
spawn rr_gyro_odom "python3 $RR/rr_gyro_odom.py"
sleep 6   # it measures the gyro bias at standstill before it starts publishing

# ------------------------------------------------------------ 4. joystick fix ----
log "joystick deadman fix (rr_fix_joy.sh)"
bash "$RR/rr_fix_joy.sh" >/dev/null 2>&1 || log "  rr_fix_joy.sh reported an issue (continuing)"

# -------------------------------------------------------------- 5. foxglove ----
if ss -ltn 2>/dev/null | grep -q ':8765'; then
  log "foxglove already listening on 8765"
else
  spawn foxglove_bridge "ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765"
  sleep 5
fi

# ------------------------------------------------------ 6. clean map (serve) ----
if [ ! -f "$MAPS/corridor_despeck.pgm" ]; then
  log "despeckling corridor_clean -> corridor_despeck"
  python3 "$RR/rr_despeckle.py" || log "  despeckle FAILED (continuing without clean map)"
else
  log "clean map corridor_despeck present"
fi

# --------------------------------------- 6.5 reload the maps if they changed ----
# map_server and slam_toolbox read their map files ONCE, at startup. If a map is
# re-made or edited while they are running, the spawns below just report
# "already up" and the stack keeps serving the STALE map -- which looks exactly
# like the edit having no effect. (2026-07-31: cost an hour; map_server had
# cached a map written two minutes before it started.)
# So: if a map file is newer than the process currently holding it, stop that
# process here and let the spawn below start it fresh.
newer_than_proc(){   # $1 = pgrep pattern, $2.. = files
  local pat="$1"; shift
  local pid; pid=$(pgrep -f "$pat" 2>/dev/null | head -1)
  [ -z "$pid" ] && return 1
  local et; et=$(ps -o etimes= -p "$pid" 2>/dev/null | tr -d ' ')
  [ -z "$et" ] && return 1
  local start=$(( $(date +%s) - et ))
  local f m
  for f in "$@"; do
    [ -f "$f" ] || continue
    m=$(stat -c %Y "$f" 2>/dev/null || echo 0)
    if [ "$m" -gt "$start" ]; then return 0; fi
  done
  return 1
}

if newer_than_proc map_clean_server "$MAPS/corridor_despeck.pgm" "$MAPS/corridor_despeck.yaml"; then
  log "corridor_despeck is NEWER than the running map_server -> reloading it"
  pkill -9 -f map_clean_server 2>/dev/null
  pkill -9 -f map_clean_lifecycle 2>/dev/null
  sleep 2
fi

# Restarting slam DISCARDS the current pose estimate, so re-seed after this.
if newer_than_proc localization_slam_toolbox_node "$MAPS/corridor_clean.posegraph" "$MAPS/corridor_clean.data"; then
  log "corridor_clean posegraph is NEWER than the running slam -> reloading it"
  log "  (localization was reset: seed the pose before sending a goal)"
  pkill -9 -f localization_slam_toolbox_node 2>/dev/null
  sleep 3
fi

# ------------------------------------------------------- 7. slam localization ----
# slam does localization (map->odom TF) but we push its NOISY occupancy grid off
# /map (to /map_posegraph) so the clean map_server can own /map instead.
spawn slam_toolbox "ros2 run slam_toolbox localization_slam_toolbox_node --ros-args --params-file $RR/localize_slam_real.yaml -r /map:=/map_posegraph -r /map_metadata:=/map_posegraph_metadata"
sleep 6

# ------------------------------------------------- 8. serve /map_clean map_server ----
if up map_clean_server; then
  log "map_clean_server already up"
else
  log "start: map_server -> /map (despeckled clean map; slam's noisy grid is on /map_posegraph)"
  setsid bash -c "source /opt/ros/humble/setup.bash; [ -f '$WS' ] && source '$WS'; export ROS_DOMAIN_ID=7; export FASTRTPS_DEFAULT_PROFILES_FILE=/home/roboracer/rr/fastdds_udp_only.xml; exec ros2 run nav2_map_server map_server --ros-args -r __node:=map_clean_server -p yaml_filename:=$MAPS/corridor_despeck.yaml -p topic_name:=/map" >"$LOGS/map_clean.log" 2>&1 </dev/null &
  sleep 4
fi
# activate it via a lifecycle_manager (CLI `lifecycle set` is unreliable for the
# remapped node on a fresh-boot daemon; the manager uses bonds and always works)
if up map_clean_lifecycle; then
  log "map_clean lifecycle_manager already up"
else
  log "start: lifecycle_manager -> activate map_clean_server"
  setsid bash -c "source /opt/ros/humble/setup.bash; [ -f '$WS' ] && source '$WS'; export ROS_DOMAIN_ID=7; export FASTRTPS_DEFAULT_PROFILES_FILE=/home/roboracer/rr/fastdds_udp_only.xml; exec ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -r __node:=map_clean_lifecycle -p node_names:=['map_clean_server'] -p autostart:=true -p bond_timeout:=0.0 -p attempt_respawn_reconnection:=true" >"$LOGS/map_clean_lifecycle.log" 2>&1 </dev/null &
  sleep 5
fi

# ------------------------------------------------ 9. Nav2 planner (RPP isolated) ----
if up bt_navigator; then
  log "nav2 already up"
else
  log "start: nav2 (planner + costmaps; its controller runs but is NOT bridged to /drive)"
  setsid bash -c "source /opt/ros/humble/setup.bash; [ -f '$WS' ] && source '$WS'; export ROS_DOMAIN_ID=7; export FASTRTPS_DEFAULT_PROFILES_FILE=/home/roboracer/rr/fastdds_udp_only.xml; exec ros2 launch nav2_bringup navigation_launch.py use_composition:=False use_sim_time:=false params_file:=$NAV_PARAMS" >"$LOGS/nav.log" 2>&1 </dev/null &
  sleep 14
fi

# ---------------------------------------------- 10. plan QoS bridge -> /control/plan ----
spawn plan_qos_relay "python3 $RR/plan_qos_relay.py"

# ----------------------------------------------- 11. map-frame odom bridge ----
spawn map_odom_relay "ros2 run roboracer_camera map_odom_relay --ros-args -p map_frame:=map -p base_frame:=base_link -p odom_topic:=/odom -p output_topic:=/odometry/map"

# --------------------------------------------------- 12. CUSTOM controller ----
spawn pure_pursuit_controller "ros2 run roboracer_control pure_pursuit_controller --ros-args --params-file $CTRL_PARAMS"

# ------------------------------------------------ 10b. LiDAR emergency brake ----
# If the controller is wired to /drive_nav then the AEB owns the real /drive and
# MUST run, or nothing forwards commands and the car is dead. Start it only in
# that case, so turning the AEB off (rr_aeb_off.sh) stays off across a bring-up.
if grep -qE "^\s*drive_topic:\s*/drive_nav" "$CTRL_PARAMS"; then
  spawn rr_wall_aeb "python3 $RR/rr_wall_aeb.py"
else
  log "AEB not wired (controller drives /drive directly) - skipping"
fi
# NOTE: opponent_detector intentionally NOT started — on this map its LiDAR classifier
# emits phantom opponents that made the controller stop ~2 m short of goals. Re-add + tune
# only when actually racing a real opponent.

# ------------------------------------------------ 13. gated auto-keeper ----
spawn rr_autokeep "python3 $RR/rr_autokeep.py"

# ----------------------------------- 14. costmap reset on /initialpose ----
spawn rr_costmap_reset "python3 $RR/rr_costmap_reset.py"

sleep 3
# ------------------------------------------------------------- roll-call ----
echo
log "================= STATUS ================="
for pair in "base/VESC:vesc_driver" "joystick:joy_node" "foxglove:foxglove_bridge" \
            "slam-loc:slam_toolbox" "map_clean:map_clean_server" "nav2:bt_navigator" \
            "planner:planner_server" "plan_relay:plan_qos_relay" "odom_bridge:map_odom_relay" \
            "CUSTOM_CTRL:pure_pursuit_controller" \
            "auto_keeper:rr_autokeep" "costmap_reset:rr_costmap_reset" \
            "gyro_odom:rr_gyro_odom"; do
  name="${pair%%:*}"; pat="${pair##*:}"
  if up "$pat"; then echo "  [ UP ] $name"; else echo "  [DOWN] $name  (see $LOGS)"; fi
done
echo
log "Foxglove: connect desktop app to  ws://192.168.50.10:8765   (enable /map + /scan; /map is the CLEAN map)"
log "Drive:    publish a goal with the Foxglove 'Publish -> Pose' tool on /goal_pose"
log "Relocalize: 'Publish -> 2D Pose Estimate' (/initialpose) -> clears costmaps + cancels goal + empties paths"
log "E-STOP:   hold LB on the gamepad (overrides everything), or Ctrl+C the nav log tail"
log "=========================================="
