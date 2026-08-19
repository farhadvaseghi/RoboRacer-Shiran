#!/usr/bin/env bash
# T0 — stack real del carro (VESC + LiDAR + joystick + mux)
source /opt/ros/humble/setup.bash
source /home/roboracer/f1tenth_ws/install/setup.bash
exec ros2 launch f1tenth_stack bringup_launch.py
