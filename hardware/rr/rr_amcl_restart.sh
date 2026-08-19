#!/bin/bash
# rr_amcl_restart.sh — restart ONLY the AMCL localization group (map_server +
# amcl + its lifecycle_manager) with the current ~/rr/amcl_global.yaml, then
# global-init. Base stack + foxglove are NOT touched. No shm clear (base live).
# Safe from pkill self-match: this script's cmdline is 'bash rr_amcl_restart.sh',
# which contains none of the kill patterns below.
export ROS_DOMAIN_ID=7
source /opt/ros/humble/setup.bash
MAP=/home/roboracer/rr_maps/corridor_clean.yaml
AMCL_PARAMS=/home/roboracer/rr/amcl_global.yaml
PAT='localization_launch|nav2_amcl|nav2_map_server|nav2_lifecycle_manager'

echo "[reloc] stopping current localization by PID..."
for p in $(pgrep -f "$PAT"); do kill -TERM $p 2>/dev/null; done
sleep 3
for p in $(pgrep -f "$PAT"); do kill -KILL $p 2>/dev/null; done
sleep 1

echo "[reloc] relaunching localization with tuned params..."
setsid bash -c "source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=7; \
  ros2 launch nav2_bringup localization_launch.py map:=$MAP \
  params_file:=$AMCL_PARAMS use_composition:=False use_sim_time:=false" \
  >/tmp/amcl.log 2>&1 </dev/null &
sleep 15
echo "[reloc] global init (place-anywhere)..."
ros2 service call /reinitialize_global_localization std_srvs/srv/Empty "{}" >/dev/null 2>&1
echo "[reloc] /map publisher: $(timeout 6 ros2 topic info /map 2>/dev/null | grep -i 'Publisher count')"
echo "[reloc] done."
