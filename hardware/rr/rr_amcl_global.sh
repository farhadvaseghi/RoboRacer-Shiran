#!/bin/bash
# Global (place-anywhere) localization with AMCL on the saved map. Run AFTER ~/t_stack.sh.
# Then DRIVE the car toward a junction/corner to make the particles converge.
export ROS_DOMAIN_ID=7
source /opt/ros/humble/setup.bash
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
MAP=${1:-/home/roboracer/rr_maps/corridor_clean.yaml}
setsid bash -c "source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=7; ros2 launch nav2_bringup localization_launch.py map:=$MAP params_file:=/home/roboracer/rr/amcl_global.yaml use_composition:=False use_sim_time:=false" >/tmp/amcl.log 2>&1 </dev/null &
sleep 14
# spread particles across the whole map
ros2 service call /reinitialize_global_localization std_srvs/srv/Empty "{}"
echo "AMCL global localization up. Now DRIVE the car to converge the particle cloud."
