#!/bin/bash
# rr_nav.sh — start Nav2 navigation servers + the /cmd_vel->/drive bridge.
# Localization must already provide /map + map->odom (slam_toolbox live, or amcl).
# Uses the TUNED real params (REEDS_SHEPP, r=0.40, 0.3 m/s). use_composition:=False
# (composition hangs stand-alone). Nav2 does NOT move the car until a /goal_pose is sent.
export ROS_DOMAIN_ID=7
source /opt/ros/humble/setup.bash 2>/dev/null
source /home/roboracer/roboracer_ws/install/setup.bash 2>/dev/null
PARAMS=/home/roboracer/roboracer_ws/src/RoboRacer-Shiran/roboracer_estimation/config/nav2_params_real.yaml

if pgrep -f bt_navigator >/dev/null; then
  echo "[rr_nav] nav2 servers already running"
else
  echo "[rr_nav] starting nav2 servers (params: nav2_params_real.yaml)"
  setsid bash -c "source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=7; \
    ros2 launch nav2_bringup navigation_launch.py use_composition:=False \
    use_sim_time:=false params_file:=$PARAMS" >/tmp/nav.log 2>&1 </dev/null &
  sleep 13
fi

if pgrep -f cmd_vel_to_ackermann >/dev/null; then
  echo "[rr_nav] cmd_vel_to_ackermann already running"
else
  echo "[rr_nav] starting cmd_vel_to_ackermann (/cmd_vel -> /drive, mux priority 10)"
  setsid bash -c "source /opt/ros/humble/setup.bash; source /home/roboracer/roboracer_ws/install/setup.bash; \
    export ROS_DOMAIN_ID=7; ros2 run roboracer_estimation cmd_vel_to_ackermann" \
    >/tmp/cmdvel.log 2>&1 </dev/null &
  sleep 3
fi

echo "[rr_nav] procs: bt=$(pgrep -fc bt_navigator) planner=$(pgrep -fc planner_server) ctrl=$(pgrep -fc controller_server) bridge=$(pgrep -fc cmd_vel_to_ackermann)"
echo "[rr_nav] logs: /tmp/nav.log  /tmp/cmdvel.log"
