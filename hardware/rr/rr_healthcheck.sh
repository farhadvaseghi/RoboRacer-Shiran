#!/usr/bin/env bash
# rr_healthcheck.sh — verify every link of the autonomous chain, one bottleneck
# at a time, and write a timestamped report to ~/rr_logs/.
#
# Run it any time. Run it AFTER sending a goal to also check B6 (plan/cmd).
# Each line is PASS / FAIL / -- (not applicable yet) with a hint on failure.

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-7}
source /opt/ros/humble/setup.bash 2>/dev/null
source ~/roboracer_ws/install/setup.bash 2>/dev/null

LOGDIR="$HOME/rr_logs"
mkdir -p "$LOGDIR"
TS=$(date +%Y%m%d_%H%M%S)
REPORT="$LOGDIR/healthcheck_$TS.txt"

pass=0; fail=0
say()  { echo "$@" | tee -a "$REPORT"; }
ok()   { say "  PASS  $1"; pass=$((pass+1)); }
bad()  { say "  FAIL  $1"; say "        hint: $2"; fail=$((fail+1)); }
na()   { say "  --    $1 ($2)"; }

has_topic() { ros2 topic list 2>/dev/null | grep -qx "$1"; }
has_node()  { ros2 node list  2>/dev/null | grep -qx "$1"; }
msg_once()  { timeout --kill-after=2 "${2:-5}" ros2 topic echo "$1" --once >/dev/null 2>&1; }   # 0 if a msg arrives
rate_of()   { timeout --kill-after=2 8 bash -c 'ros2 topic hz "$1" 2>/dev/null | grep -m1 "average rate" | grep -oE "[0-9]+\.[0-9]+" | head -1' _ "$1"; }
tf_ok()     { timeout --kill-after=2 12 bash -c 'ros2 run tf2_ros tf2_echo "$1" "$2" 2>/dev/null | grep -m1 -qE "Translation|At time"' _ "$1" "$2"; }
lc_state()  { ros2 lifecycle get "$1" 2>/dev/null | head -1; }

say "===== RoboRacer autonomous health check  $TS ====="
say "date on car: $(date)"
say ""

# ---------- B1: SENSORS + DRIVE STACK (t_stack.sh) ----------
say "[B1] Sensors / drive stack  (source: ~/t_stack.sh)"
[ -e /dev/sensors/vesc ] && ok "VESC symlink /dev/sensors/vesc" \
  || bad "VESC symlink missing" "is the VESC USB plugged in? is t_stack running?"
if msg_once /scan 8; then
  ok "/scan publishing"
else
  bad "/scan not publishing" "LiDAR down: check eno1 link to 192.168.0.10 (Hokuyo)"
fi

if msg_once /odom 8; then
  ok "/odom publishing"
else
  bad "/odom not publishing" "VESC/vesc_to_odom down: check t_stack log"
fi
say ""

# ---------- B2: BASE TF ----------
say "[B2] Base TF chain"
tf_ok odom base_link && ok "odom -> base_link" \
  || bad "no odom->base_link TF" "vesc_to_odom not publishing TF; check t_stack"
tf_ok base_link laser && ok "base_link -> laser" \
  || bad "no base_link->laser TF" "static_transform_publisher missing in bringup"
say ""

# ---------- B3: MAP SERVER ----------
say "[B3] Map server  (source: autonomous_real.launch.py)"
has_node /map_server && ok "node /map_server up" \
  || bad "/map_server not running" "autonomous_real.launch.py not started?"
if has_node /map_server; then
  s=$(lc_state /map_server); echo "$s" | grep -qi active \
    && ok "/map_server lifecycle: $s" \
    || bad "/map_server not active ($s)" "lifecycle_manager_localization failed; check its log"
fi
msg_once /map 6 && ok "/map has data" \
  || bad "/map empty/absent" "wrong map path in map:= ? file missing in ~/rr_maps/"
say ""

# ---------- B4: LOCALIZATION (amcl) ----------
say "[B4] Localization (amcl)"
has_node /amcl && ok "node /amcl up" || bad "/amcl not running" "localization launch failed"
if has_node /amcl; then
  s=$(lc_state /amcl); echo "$s" | grep -qi active \
    && ok "/amcl lifecycle: $s" || bad "/amcl not active ($s)" "lifecycle_manager_localization failed"
fi
tf_ok map odom && ok "map -> odom (localized)" \
  || bad "no map->odom TF" "set an initial pose: rr_initpose.sh, or RViz '2D Pose Estimate'"
msg_once /amcl_pose 6 && ok "/amcl_pose publishing" \
  || bad "/amcl_pose absent" "amcl has no initial pose yet, or /scan not reaching amcl"
say ""

# ---------- B5: NAV SERVERS ----------
say "[B5] Navigation servers"
for n in /planner_server /controller_server /bt_navigator /behavior_server /velocity_smoother; do
  if has_node "$n"; then
    s=$(lc_state "$n"); echo "$s" | grep -qi active \
      && ok "$n: $s" || bad "$n not active ($s)" "lifecycle_manager_navigation failed; check its log"
  else
    bad "$n not running" "autonomous_real.launch.py navigation part failed"
  fi
done
say ""

# ---------- B6: PLAN + COMMAND (only meaningful AFTER a goal is sent) ----------
say "[B6] Plan + command  (send a goal first: rr_goal.sh X Y YAW)"
if msg_once /plan 4; then
  ok "/plan present (planner produced a path)"
else
  na "/plan" "no path yet — send a goal, or planner failed (check planner_server log / costmaps)"
fi
r=$(rate_of /cmd_vel); [ -n "$r" ] && ok "/cmd_vel @ ${r} Hz (Nav2 controlling)" \
  || na "/cmd_vel" "no command yet — only flows while a goal is active"
r=$(rate_of /drive);  [ -n "$r" ] && ok "/drive @ ${r} Hz (cmd_vel_to_ackermann)" \
  || na "/drive" "no drive cmd — is cmd_vel_to_ackermann running? is a goal active?"
say ""

# ---------- B7: ACTUATION (mux + VESC) ----------
say "[B7] Actuation (mux -> VESC)"
has_node /ackermann_mux && ok "node /ackermann_mux up" \
  || bad "/ackermann_mux missing" "t_stack not fully up"
r=$(rate_of /commands/motor/speed); [ -n "$r" ] && ok "/commands/motor/speed @ ${r} Hz" \
  || na "/commands/motor/speed" "no motor cmd yet — only while a goal is active & mux forwarding"
say ""

say "===== SUMMARY:  $pass PASS / $fail FAIL  ====="
say "report saved: $REPORT"
