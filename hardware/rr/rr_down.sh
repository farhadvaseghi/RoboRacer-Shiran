#!/usr/bin/env bash
# rr_down.sh — cleanly stop everything we start on the car (base stack, Nav2,
# slam, foxglove, keepalive) and clear stale shared memory. Safe to run: it
# executes from a file, so the pkill -f patterns below never match this
# script's own command line, and pkill never signals itself.
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-7}

# 1. keepalive first (it publishes /drive)
pkill -f 'topic pub /drive' 2>/dev/null

# 2. graceful TERM to every stack component
for pat in navigation_launch 'nav2_' lifecycle_manager cmd_vel_to_ackermann \
           slam_toolbox foxglove joy_teleop bringup_launch vesc urg_node \
           ackermann_mux joy_node 'ros2 launch'; do
  pkill -f "$pat" 2>/dev/null
done
sleep 3

# 3. force-kill any stragglers
for pat in navigation_launch 'nav2_' slam_toolbox foxglove vesc urg_node \
           ackermann_mux joy_node cmd_vel_to_ackermann; do
  pkill -9 -f "$pat" 2>/dev/null
done

# 4. stop the ros2 daemon and clear stale FastRTPS shm
source /opt/ros/humble/setup.bash 2>/dev/null
ros2 daemon stop >/dev/null 2>&1
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
sleep 1

echo "remaining ROS procs:"
pgrep -af 'slam_toolbox|vesc|urg_node|foxglove|ackermann|joy_node|joy_teleop|nav2|amcl|map_server|topic pub /drive|bringup_launch|navigation_launch' \
  | grep -v rr_down || echo "  none"
