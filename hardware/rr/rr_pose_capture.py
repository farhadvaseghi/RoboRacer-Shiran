#!/usr/bin/env python3
"""Print the pose the stack currently believes, as "x y yaw_deg".

Exists so slam can be restarted WITHOUT teleporting the pose back to the map
origin. rr_seed_start.py always seeds (0,0,0), which is correct only when the
car is parked at the start. Capture the live pose with this before killing
slam, then feed it straight to seed_pose.py afterwards:

    POSE=$(python3 ~/rr/rr_pose_capture.py) && ... && python3 ~/rr/seed_pose.py $POSE

Prints nothing and exits non-zero if the pose cannot be read, so a caller can
fall back to a known seed rather than seeding garbage.

Usage:  ROS_DOMAIN_ID=7 python3 ~/rr/rr_pose_capture.py
"""

import math
import os
import sys
import time

os.environ.setdefault("ROS_DOMAIN_ID", "7")

import rclpy
from tf2_ros import Buffer, TransformListener

TF_WAIT_SECONDS = 10.0


def main():
    rclpy.init()
    node = rclpy.create_node("rr_pose_capture")
    buf = Buffer()
    TransformListener(buf, node)

    end = time.monotonic() + TF_WAIT_SECONDS
    tf = None
    while rclpy.ok() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        if buf.can_transform("map", "base_link", rclpy.time.Time()):
            tf = buf.lookup_transform("map", "base_link", rclpy.time.Time())
            break

    node.destroy_node()
    rclpy.shutdown()

    if tf is None:
        print("no map->base_link transform within %.0fs" % TF_WAIT_SECONDS,
              file=sys.stderr)
        return 1

    q = tf.transform.rotation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y ** 2 + q.z ** 2))
    print("%.4f %.4f %.3f" % (tf.transform.translation.x,
                              tf.transform.translation.y,
                              math.degrees(yaw)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
