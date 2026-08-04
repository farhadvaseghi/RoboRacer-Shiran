#!/usr/bin/env bash
# Kill ONLY the base stack (vesc/odom/joy/lidar/mux) — leaves slam/nav2/foxglove/pure_pursuit up.
pkill -9 -f "f1tenth_stack bringup_launch" 2>/dev/null
pkill -9 -f vesc 2>/dev/null
pkill -9 -f urg_node 2>/dev/null
pkill -9 -f ackermann 2>/dev/null
pkill -9 -f throttle_interpolator 2>/dev/null
pkill -9 -f joy_teleop 2>/dev/null
pkill -9 -f joy_node 2>/dev/null
sleep 3
echo "base_remaining=$(pgrep -fc "vesc_driver|urg_node|ackermann_to_vesc|vesc_to_odom|joy_node")"
