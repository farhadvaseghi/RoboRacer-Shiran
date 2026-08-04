#!/bin/bash
# =====================================================================
# rr_aeb.sh — start the Automatic Emergency Braking chain (real car).
#
# Chain (each is a pass-through /drive filter; ANY one braking stops the car):
#   controller ─/drive_nav─► [rr_wall_aeb] ─/drive_wall─► [emergency_brake] ─/drive─► mux
#                              LiDAR, any obstacle        camera YOLO, persons
#
# PREREQUISITE: the controller must publish to /drive_nav (not /drive). Set
#   drive_topic: /drive_nav   in ~/rr/controller_params_real.yaml   and restart the controller.
# Joystick (mux priority 100) still overrides everything.
#
# USE_PERSON=1  also start the camera person-AEB (person_detector YOLO + emergency_brake).
#               Needs the ZED SDK + ultralytics on the car. Default 0 = LiDAR wall-AEB only.
# =====================================================================
export ROS_DOMAIN_ID=7
source /opt/ros/humble/setup.bash 2>/dev/null
source ~/roboracer_ws/install/setup.bash 2>/dev/null
LOGS=~/rr_logs; mkdir -p "$LOGS"
USE_PERSON="${USE_PERSON:-0}"

start(){ setsid bash -c "source /opt/ros/humble/setup.bash; source ~/roboracer_ws/install/setup.bash; export ROS_DOMAIN_ID=7; exec $1" >"$LOGS/$2.log" 2>&1 </dev/null & }

if [ "$USE_PERSON" = "1" ]; then
  echo "[rr_aeb] chain: /drive_nav -> wall_aeb -> /drive_wall -> person_aeb -> /drive"
  start "python3 ~/rr/rr_wall_aeb.py --ros-args -p drive_in_topic:=/drive_nav -p drive_out_topic:=/drive_wall" wall_aeb
  start "ros2 run roboracer_camera person_detector" person_detector          # YOLO, needs ZED SDK
  start "ros2 run roboracer_camera emergency_brake --ros-args -p drive_in_topic:=/drive_wall -p drive_out_topic:=/drive" person_aeb
else
  echo "[rr_aeb] LiDAR wall-AEB only: /drive_nav -> wall_aeb -> /drive   (USE_PERSON=1 to add camera YOLO AEB)"
  start "python3 ~/rr/rr_wall_aeb.py --ros-args -p drive_in_topic:=/drive_nav -p drive_out_topic:=/drive" wall_aeb
fi
sleep 3
echo "[rr_aeb] running: $(pgrep -fc rr_wall_aeb) wall_aeb, $( [ "$USE_PERSON" = 1 ] && pgrep -fc emergency_brake || echo 0) person_aeb"
echo "[rr_aeb] reminder: controller_params drive_topic must be /drive_nav (restart controller if you just changed it)."
