#!/bin/bash
# Re-apply the joy_teleop fix (removes the always-on default 0-teleop that blocks
# nav). Run after any base-stack relaunch or a colcon rebuild of f1tenth_stack.
# Restarts ONLY the joy_teleop node (not joy_node) with ~/rr/joy_teleop_fixed.yaml.
source /opt/ros/humble/setup.bash; source ~/f1tenth_ws/install/setup.bash
export ROS_DOMAIN_ID=7
JT=$(pgrep -f "lib/joy_teleop/joy_teleop")
[ -n "$JT" ] && { echo "killing joy_teleop $JT"; kill $JT; sleep 2; }
setsid bash -c "source /opt/ros/humble/setup.bash; source ~/f1tenth_ws/install/setup.bash; \
  export ROS_DOMAIN_ID=7; ros2 run joy_teleop joy_teleop --ros-args -r __node:=joy_teleop \
  --params-file /home/roboracer/rr/joy_teleop_fixed.yaml" >/tmp/joyfix.log 2>&1 </dev/null &
sleep 3; echo "joy_teleop restarted; /teleop is silent unless LB(4) held."
