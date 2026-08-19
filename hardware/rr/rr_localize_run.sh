#!/usr/bin/env bash
# rr_localize_run.sh — one command to LOCALIZE the real car on the saved map,
# then self-check whether it worked.
#
# READ LOCALIZATION.md FIRST. It tells you where to physically place the car
# (on the floor cross) before you run this — placement is what makes it work.
#
# What this does, in order:
#   1. Sanity: source ROS, set ROS_DOMAIN_ID, warn if the clock is stuck at 1970.
#   2. Bring up the localization stack from the proven building blocks:
#        ~/rr/rr_up_slam.sh  -> base sensor/drive stack + Foxglove + slam_toolbox
#                               LOCALIZATION on the saved corridor_clean map.
#                               The map origin IS the cross, so with the car on
#                               the cross it comes up localized at (0,0,0) — no
#                               manual initial pose needed.
#        ~/rr/rr_keep.sh     -> zero-speed /drive keepalive so the VESC keeps
#                               emitting /odom + odom->base_link at rest. Without
#                               it slam_toolbox drops every scan and never
#                               produces map->odom. Localization CANNOT work
#                               without this while the car is parked.
#        ~/rr/rr_fix_joy.sh  -> arms the LB (button 4) deadman / e-stop.
#   3. Wait for localization to converge, then VERIFY the whole TF chain up to
#      map->odom (the transform that means "the car knows where it is") and
#      print a clear PASS / FAIL verdict.
#
# Idempotent (safe to re-run). Never uses `pkill -f`. On any FAIL it points you
# to the matching section of LOCALIZATION.md.
#
# Config (override via env):  ROS_DOMAIN_ID (7)  RR (~/rr)  MAP_NAME (corridor_clean)
#                             CONVERGE_WAIT seconds to wait for map->odom (35)

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-7}
RR=${RR:-$HOME/rr}
MAP_NAME=${MAP_NAME:-corridor_clean}
CONVERGE_WAIT=${CONVERGE_WAIT:-35}

source /opt/ros/humble/setup.bash 2>/dev/null
source "$HOME/f1tenth_ws/install/setup.bash" 2>/dev/null
source "$HOME/roboracer_ws/install/setup.bash" 2>/dev/null

crit_fail=0; warn=0
say()  { echo "$@"; }
ok()   { say "  PASS  $1"; }
bad()  { say "  FAIL  $1"; say "        -> $2"; crit_fail=$((crit_fail+1)); }
warnl(){ say "  WARN  $1"; say "        -> $2"; warn=$((warn+1)); }

has_topic() { ros2 topic list 2>/dev/null | grep -qx "$1"; }
has_node()  { ros2 node list  2>/dev/null | grep -q  "$1"; }
rate_of()   { timeout 6 ros2 topic hz "$1" 2>/dev/null | grep -m1 'average rate' | grep -oE '[0-9]+\.[0-9]+' | head -1; }
msg_once()  { timeout "${2:-6}" ros2 topic echo "$1" --once >/dev/null 2>&1; }
tf_ok()     { timeout 5 ros2 run tf2_ros tf2_echo "$1" "$2" 2>/dev/null | grep -qE 'Translation|At time'; }
tf_xy()     { timeout 5 ros2 run tf2_ros tf2_echo "$1" "$2" 2>/dev/null | grep -m1 'Translation' | grep -oE '[-0-9.]+' | head -2 | tr '\n' ' '; }

say "=================================================================="
say " RoboRacer localization  —  map: $MAP_NAME   domain: $ROS_DOMAIN_ID"
say "=================================================================="

# --- clock sanity ---------------------------------------------------------
if [ "$(date +%Y)" = "1970" ]; then
  say "!! CLOCK IS 1970 (dead RTC). Fix it NOW, before the stack is running:"
  say "     sudo date -u -s \"$(date -u +%Y-%m-%d\ %H:%M:%S 2>/dev/null || echo 'YYYY-MM-DD HH:MM:SS')\""
  say "   Never date-fix once the base stack is up (the jump breaks its TF)."
  say "   Aborting so you can set the clock first. Re-run afterwards."
  exit 2
fi

# --- 1. bring up the localization stack ----------------------------------
if [ ! -x "$RR/rr_up_slam.sh" ]; then
  say "FATAL: $RR/rr_up_slam.sh not found."
  say "  This runner orchestrates the car-side helper scripts. Deploy them"
  say "  first (PC:  bash nav2-realcar-deploy/deploy.sh) or see LOCALIZATION.md."
  exit 3
fi

say ""
say ">> base stack + Foxglove + slam_toolbox localization  (rr_up_slam.sh)"
"$RR/rr_up_slam.sh"

say ""
say ">> odom keepalive  (rr_keep.sh) — required so slam sees /odom at rest"
if [ -x "$RR/rr_keep.sh" ]; then "$RR/rr_keep.sh"; else
  say "   rr_keep.sh missing — starting an inline zero-speed keepalive"
  if ! pgrep -f 'topic pub /drive' >/dev/null; then
    setsid bash -c "source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=$ROS_DOMAIN_ID; \
      exec ros2 topic pub /drive ackermann_msgs/msg/AckermannDriveStamped '{drive: {speed: 0.0}}' -r 20" \
      >/tmp/zerodrive.log 2>&1 </dev/null &
  fi
fi

say ""
say ">> joystick deadman / e-stop  (rr_fix_joy.sh)"
[ -x "$RR/rr_fix_joy.sh" ] && "$RR/rr_fix_joy.sh" || say "   (skipped — rr_fix_joy.sh not found; e-stop may not be armed)"

# --- 2. wait for convergence ---------------------------------------------
say ""
printf ">> waiting up to %ss for map->odom (localization to converge) " "$CONVERGE_WAIT"
waited=0
until tf_ok map odom || [ "$waited" -ge "$CONVERGE_WAIT" ]; do printf '.'; sleep 3; waited=$((waited+3)); done
say ""

# --- 3. verify ------------------------------------------------------------
say ""
say "================  LOCALIZATION CHECK  ================"

say "[L1] Sensors"
r=$(rate_of /scan); [ -n "$r" ] && ok "/scan @ ${r} Hz" \
  || bad "/scan not publishing" "LiDAR down. See LOCALIZATION.md > Troubleshooting > No /scan."
r=$(rate_of /odom); [ -n "$r" ] && ok "/odom @ ${r} Hz" \
  || bad "/odom not publishing" "Keepalive not reaching the VESC. See > No /odom (keepalive)."

say "[L2] Base TF"
tf_ok odom base_link && ok "odom -> base_link" \
  || bad "no odom -> base_link" "VESC odom TF missing (usually same as No /odom)."
tf_ok base_link laser && ok "base_link -> laser" \
  || warnl "no base_link -> laser" "static TF from bringup didn't start; restart the base stack."

say "[L3] Map"
has_node slam_toolbox && ok "slam_toolbox node up" \
  || bad "slam_toolbox not running" "localization node failed — see /tmp/slamloc.log."
msg_once /map 8 && ok "/map has data" \
  || bad "/map empty/absent" "wrong map or slam failed to load it. See > Map won't load."

say "[L4] Localization  (the part that matters)"
if tf_ok map odom; then
  ok "map -> odom present  (the car is localized)"
  a=$(tf_xy map odom); sleep 4; b=$(tf_xy map odom)
  d=$(awk -v a="$a" -v b="$b" 'BEGIN{split(a,p);split(b,q);dx=p[1]-q[1];dy=p[2]-q[2];printf "%.3f",sqrt(dx*dx+dy*dy)}' 2>/dev/null)
  if [ -n "$d" ] && awk -v d="$d" 'BEGIN{exit !(d<0.30)}' 2>/dev/null; then
    ok "map -> odom stable while parked (moved ${d} m in 4 s)"
  else
    warnl "map -> odom drifting while parked (moved ${d:-?} m in 4 s)" \
      "Localization is jittering. Check scan-vs-map overlap in Foxglove. See > Pose drifts / deflects."
  fi
else
  bad "no map -> odom (NOT localized)" \
    "Car may be off the cross, or /odom never came up. See > No map->odom (main failure)."
fi

# --- verdict --------------------------------------------------------------
say ""
say "====================================================="
if [ "$crit_fail" -eq 0 ]; then
  say " RESULT:  LOCALIZATION READY  ($warn warning(s))"
  say " Sanity-check in Foxglove: the red laser scan should sit on the map"
  say " walls. Then hand off to navigation (see LOCALIZATION.md > Done)."
else
  say " RESULT:  NOT LOCALIZED  ($crit_fail critical, $warn warning(s))"
  say " Fix the FAIL line above using LOCALIZATION.md > Troubleshooting,"
  say " then re-run:  $RR/rr_localize_run.sh"
fi
say "====================================================="
exit $crit_fail
