#!/usr/bin/env bash
# Turn LiDAR emergency braking OFF and put the controller back on /drive.
set +u
export ROS_DOMAIN_ID=7
RR="$HOME/rr"
CTRL="$RR/controller_params_real.yaml"
source /opt/ros/humble/setup.bash
[ -f "$HOME/f1tenth_ws/install/setup.bash" ] && source "$HOME/f1tenth_ws/install/setup.bash"
[ -f "$HOME/roboracer_ws/install/setup.bash" ] && source "$HOME/roboracer_ws/install/setup.bash"

pkill -f 'rr_wall_ae[b]' 2>/dev/null
sed -i 's|^\(\s*drive_topic:\s*\).*|\1/drive|' "$CTRL"
echo "[aeb-off] controller drive_topic -> /drive"

pkill -f 'pure_pursuit_controlle[r]' 2>/dev/null
sleep 2
setsid bash -c "source /opt/ros/humble/setup.bash; [ -f '$HOME/roboracer_ws/install/setup.bash' ] && source '$HOME/roboracer_ws/install/setup.bash'; export ROS_DOMAIN_ID=7; exec stdbuf -oL ros2 run roboracer_control pure_pursuit_controller --ros-args --params-file $CTRL" \
  >> "$HOME/rr_logs/pure_pursuit_controller.log" 2>&1 </dev/null &
sleep 3
echo "[aeb-off] controller: $(pgrep -cf 'pure_pursuit_controlle[r]')  aeb: $(pgrep -cf 'rr_wall_ae[b]')"
