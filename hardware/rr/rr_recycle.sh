#!/usr/bin/env bash
# Detached full recycle: teardown -> cold bringup. Survives ssh drops.
exec > ~/rr/recycle_last.log 2>&1
echo "[recycle] $(date) starting teardown"
bash ~/rr/rr_teardown.sh
sleep 3
# second sweep to be sure
bash ~/rr/rr_teardown.sh
sleep 2
echo "[recycle] nodes remaining: $(pgrep -fc "component_container|slam_toolbox|nav2|vesc_driver|foxglove_bridge|pure_pursuit")"
echo "[recycle] shm remaining: $(ls /dev/shm 2>/dev/null | grep -c fastrtps)"
echo "[recycle] starting cold bringup"
cd ~/rr && bash rr_bringup.sh
echo "[recycle] DONE $(date)"
