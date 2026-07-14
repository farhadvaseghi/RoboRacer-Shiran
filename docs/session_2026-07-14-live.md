# Live Session — 2026-07-14 (ongoing)

User is now inside SSH on roboracer@192.168.50.10.

This file will be updated with live commands and results as we run the session.

## Goals for this session
- Assess current state of the car (what survived from 07-07)
- Fix immediate problems (clock, stale processes, domain, shm)
- Run healthcheck
- Get base stack healthy (/scan, /odom, TF, mux)
- Decide: continue live-SLAM out-and-back (fix REEDS_SHEPP) or switch to saved-map amcl flow
- Attempt first autonomous goal if safe

## Commands & results (live log)

### Initial state (right after user said "we are in terminal already")

**Command block 1 - Discovery + source + basic ROS state + healthcheck**

```bash
echo "=== 1. BASIC SYSTEM ==="
whoami
hostname
date
ip addr show | grep -E 'inet (192|100|169)' -A1 | head -10
ls -l /dev/sensors/ 2>/dev/null || echo "no /dev/sensors"

echo ""
echo "=== 2. SOURCE ROS + WORKSPACE + DOMAIN ==="
source /opt/ros/humble/setup.bash
source ~/roboracer_ws/install/setup.bash
export ROS_DOMAIN_ID=7
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

echo ""
echo "=== 3. CURRENT ROS NODES & TOPICS (domain 7) ==="
echo "Nodes:"
ros2 node list 2>/dev/null || echo "(node list failed or empty)"
echo ""
echo "Topics (first 40):"
ros2 topic list 2>/dev/null | head -40 || echo "(topic list failed)"

echo ""
echo "=== 4. RUN HEALTHCHECK ==="
ls -l ~/rr/ 2>/dev/null || echo "no ~/rr dir"
~/rr/rr_healthcheck.sh 2>&1 | cat

echo ""
echo "=== 5. PROCESS SNAPSHOT ==="
ps aux | grep -E 'ros2|urg_node|vesc|ackermann|slam_toolbox|nav2|bt_navigator|planner_server|controller_server|map_server|amcl' | grep -v grep || echo "no matching processes"
```

Run the entire block above and **paste ALL output** here.

We will then decide cleanup + next launch.
