#!/usr/bin/env bash
# Full clean teardown: stop all ROS nodes for domain 7, then wipe FastDDS shm.
pkill -9 -f component_container 2>/dev/null
pkill -9 -f slam_toolbox 2>/dev/null
pkill -9 -f nav2 2>/dev/null
pkill -9 -f bt_navigator 2>/dev/null
pkill -9 -f planner_server 2>/dev/null
pkill -9 -f controller_server 2>/dev/null
pkill -9 -f foxglove_bridge 2>/dev/null
pkill -9 -f lifecycle_manager 2>/dev/null
pkill -9 -f map_server 2>/dev/null
pkill -9 -f robot_state_publisher 2>/dev/null
pkill -9 -f joy_node 2>/dev/null
pkill -9 -f urg_node 2>/dev/null
pkill -9 -f rr_wall_aeb 2>/dev/null   # else it survives and rejoins in a stale DDS generation
pkill -9 -f vesc 2>/dev/null
pkill -9 -f ackermann 2>/dev/null
pkill -9 -f throttle_interpolator 2>/dev/null
pkill -9 -f pure_pursuit 2>/dev/null
pkill -9 -f rr_autokeep 2>/dev/null
pkill -9 -f rr_costmap 2>/dev/null
pkill -9 -f map_odom_relay 2>/dev/null
pkill -9 -f plan_qos_relay 2>/dev/null
pkill -9 -f "bringup_launch" 2>/dev/null
pkill -9 -f "ros2 launch" 2>/dev/null
sleep 3
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
echo "TEARDOWN_DONE remaining_ros=$(pgrep -fc "ros2|slam_toolbox|nav2|vesc|foxglove") shm_fastrtps=$(ls /dev/shm 2>/dev/null | grep -c fastrtps)"
