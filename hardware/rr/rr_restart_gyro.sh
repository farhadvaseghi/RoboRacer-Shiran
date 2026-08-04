#!/usr/bin/env bash
# Restart just rr_gyro_odom (e.g. after editing it, or to re-measure the gyro
# bias). Detached, so an ssh drop cannot kill it mid-restart.
# The car MUST BE STILL: the first 4 s measure the bias.
# A restart resets x/y/yaw to zero, so RE-SEED afterwards:
#   python3 ~/rr/rr_pose_capture.py   (before)  ->  python3 ~/rr/seed_pose.py X Y YAW
RR=/home/roboracer/rr
pkill -9 -f "rr_gyro_odo[m]\.py" 2>/dev/null
sleep 2
setsid bash -c "source /opt/ros/humble/setup.bash; source /home/roboracer/f1tenth_ws/install/setup.bash; source /home/roboracer/roboracer_ws/install/setup.bash; export ROS_DOMAIN_ID=7; export FASTRTPS_DEFAULT_PROFILES_FILE=$RR/fastdds_udp_only.xml; exec python3 $RR/rr_gyro_odom.py" >/home/roboracer/rr_logs/rr_gyro_odom.log 2>&1 </dev/null &
