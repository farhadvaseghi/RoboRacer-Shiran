#!/usr/bin/env bash
# Turn LiDAR emergency braking ON.
#
# Re-wires the drive chain so the AEB owns /drive and can veto the controller:
#     controller -- /drive_nav --> rr_wall_aeb -- /drive --> mux --> VESC
#
# The controller's drive_topic is PERSISTED to /drive_nav, so rr_bringup.sh
# starts the AEB automatically from now on (it checks that setting). Without
# the AEB running, nothing would forward /drive_nav to /drive and the car would
# be dead -- which is why bringup was taught about it.
set +u
export ROS_DOMAIN_ID=7
RR="$HOME/rr"
CTRL="$RR/controller_params_real.yaml"
source /opt/ros/humble/setup.bash
[ -f "$HOME/f1tenth_ws/install/setup.bash" ] && source "$HOME/f1tenth_ws/install/setup.bash"
[ -f "$HOME/roboracer_ws/install/setup.bash" ] && source "$HOME/roboracer_ws/install/setup.bash"

cp -p "$CTRL" "$CTRL.bak_aeb_on"
sed -i 's|^\(\s*drive_topic:\s*\).*|\1/drive_nav|' "$CTRL"
echo "[aeb-on] controller drive_topic -> /drive_nav"

# Restart ONLY the controller so it picks up the new topic.
pkill -f 'pure_pursuit_controlle[r]' 2>/dev/null
sleep 2
setsid bash -c "source /opt/ros/humble/setup.bash; [ -f '$HOME/roboracer_ws/install/setup.bash' ] && source '$HOME/roboracer_ws/install/setup.bash'; export ROS_DOMAIN_ID=7; exec stdbuf -oL ros2 run roboracer_control pure_pursuit_controller --ros-args --params-file $CTRL" \
  >> "$HOME/rr_logs/pure_pursuit_controller.log" 2>&1 </dev/null &
sleep 3

if pgrep -f 'rr_wall_ae[b]' >/dev/null; then
  echo "[aeb-on] AEB already running"
else
  setsid bash -c "source /opt/ros/humble/setup.bash; [ -f '$HOME/roboracer_ws/install/setup.bash' ] && source '$HOME/roboracer_ws/install/setup.bash'; export ROS_DOMAIN_ID=7; exec stdbuf -oL python3 $RR/rr_wall_aeb.py" \
    > "$HOME/rr_logs/rr_wall_aeb.log" 2>&1 </dev/null &
  sleep 3
fi

echo "[aeb-on] controller: $(pgrep -cf 'pure_pursuit_controlle[r]')  aeb: $(pgrep -cf 'rr_wall_ae[b]')"
echo "[aeb-on] watch it:  ros2 topic echo /perception/aeb_state"
echo "[aeb-on] turn off:  bash ~/rr/rr_aeb_off.sh"
