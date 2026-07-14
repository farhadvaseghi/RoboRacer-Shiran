#!/bin/bash
# Localize in a saved map with slam_toolbox (NOT amcl). Run AFTER ~/t_stack.sh.
export ROS_DOMAIN_ID=7
source /opt/ros/humble/setup.bash
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
# zero-speed drive keepalive so /odom (odom->base_link TF) flows at rest (no motion)
setsid bash -c 'source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=7; ros2 topic pub /drive ackermann_msgs/msg/AckermannDriveStamped "{drive: {speed: 0.0}}" -r 20' >/tmp/zerodrive.log 2>&1 </dev/null &
exec ros2 run slam_toolbox localization_slam_toolbox_node --ros-args --params-file ~/rr/localize_slam_real.yaml
