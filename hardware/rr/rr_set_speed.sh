#!/usr/bin/env bash
# Set the custom pure_pursuit controller's drive speed and restart it.
#
#   bash ~/rr/rr_set_speed.sh 1.5     set 1.5 m/s
#   bash ~/rr/rr_set_speed.sh         just show the current values
#
# Four keys have to move together or the change does nothing:
#   speed / straight_speed   the nominal and straight-line targets
#   turn_speed               kept at 80% of the target, as it was at 1.0/0.8
#   max_speed_command        the hard cap -- leave it behind and it silently
#                            clamps everything else back down
#
# The controller reads this file once at startup, so it is restarted here. A
# restart does NOT touch slam or the pose; it only drops the current path,
# which means the car stops. Send a fresh goal afterwards.
#
# Lookahead is velocity-scaled (lookahead_time 0.8 s) but capped by
# max_lookahead_distance, currently 0.80 m -- so above ~1.0 m/s the lookahead
# stops growing with speed and the car starts cutting corners. Raise that cap
# alongside speed if you go much past 1.5 m/s.

set +u
export ROS_DOMAIN_ID=7
RR="$HOME/rr"
CTRL="$RR/controller_params_real.yaml"
LOGS="$HOME/rr_logs"
WS="$HOME/roboracer_ws/install/setup.bash"

show() {
  echo "[speed] current values in $(basename "$CTRL"):"
  grep -E "^\s*(speed|straight_speed|turn_speed|max_speed_command|max_lookahead_distance|max_acceleration):" \
    "$CTRL" | sed 's/^/    /'
  # Count the NODE only. 'ros2 run' leaves a python wrapper plus the real
  # executable, so pgrep -c always reports 2 for a single healthy controller.
  echo "[speed] controller nodes running: $(pgrep -cf 'lib/roboracer_control/pure_pursuit_controlle[r]')"
}

if [ -z "$1" ]; then
  show
  echo "[speed] usage: bash $0 <m/s>"
  exit 0
fi

TARGET="$1"
if ! echo "$TARGET" | grep -qE '^[0-9]+(\.[0-9]+)?$'; then
  echo "[speed] ERROR: '$TARGET' is not a number"
  exit 2
fi

TURN=$(awk "BEGIN{printf \"%.3f\", $TARGET * 0.8}")

cp -p "$CTRL" "$CTRL.bak_speed_$(date +%Y%m%d_%H%M%S)"
sed -i -E \
  -e "s/^(\s*speed:).*/\1 $TARGET/" \
  -e "s/^(\s*straight_speed:).*/\1 $TARGET/" \
  -e "s/^(\s*turn_speed:).*/\1 $TURN/" \
  -e "s/^(\s*max_speed_command:).*/\1 $TARGET/" \
  "$CTRL"

echo "[speed] set speed=$TARGET straight=$TARGET turn=$TURN cap=$TARGET"

source /opt/ros/humble/setup.bash
[ -f "$WS" ] && source "$WS"

pkill -f 'pure_pursuit_controlle[r]' 2>/dev/null
sleep 2
setsid bash -c "source /opt/ros/humble/setup.bash; [ -f '$WS' ] && source '$WS'; export ROS_DOMAIN_ID=7; export FASTRTPS_DEFAULT_PROFILES_FILE=$RR/fastdds_udp_only.xml; exec stdbuf -oL ros2 run roboracer_control pure_pursuit_controller --ros-args --params-file $CTRL" \
  >>"$LOGS/pure_pursuit_controller.log" 2>&1 </dev/null &
sleep 3

show
echo "[speed] the car is stopped (the restart dropped the path) - send a new goal."
