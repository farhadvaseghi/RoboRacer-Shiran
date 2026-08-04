#!/usr/bin/env python3
"""Read raw wheel odometry against a tape measure. No slam involved.

Every odom number so far has been inferred by comparing against slam's pose --
but slam is losing the car, so that comparison is circular. This reads
odom->base_link directly and prints distance and heading change since the last
reset, so it can be checked against a tape measure and a marked angle.

TWO MEASUREMENTS, both from a standing start:

  1. SCALE   Mark the floor at the front bumper. Drive straight ~3 m. Mark the
             front bumper again. Measure between the marks. Compare to the
             'straight-line' number here.
             Front bumper at BOTH ends -- measuring front-to-rear is what
             produced a false calibration on 2026-07-23.

  2. HEADING Park, press r, then turn the car through a known angle (a right
             angle against a wall or floor tile is easiest). Compare the real
             angle to the 'heading' number here.
             This is the channel vesc_to_odom fabricates from the steering
             COMMAND, so it is the one most likely to be wrong.

Usage:  ROS_DOMAIN_ID=7 python3 ~/rr/rr_odom_ruler.py
        r + Enter  reset to zero      Ctrl+C  quit
"""

import math
import os
import select
import sys
import time

os.environ.setdefault("ROS_DOMAIN_ID", "7")

import rclpy
from tf2_ros import Buffer, TransformListener

SETUP_WAIT_SECONDS = 15.0


def yaw_of(rot):
    return math.atan2(2 * (rot.w * rot.z + rot.x * rot.y),
                      1 - 2 * (rot.y ** 2 + rot.z ** 2))


def unwrap(previous, current):
    """Keep a continuous heading so turns past 180 deg keep accumulating."""
    d = (current - previous + math.pi) % (2 * math.pi) - math.pi
    return previous + d


def main():
    rclpy.init()
    node = rclpy.create_node("rr_odom_ruler")
    buf = Buffer()
    TransformListener(buf, node)

    end = time.monotonic() + SETUP_WAIT_SECONDS
    while rclpy.ok() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        if buf.can_transform("odom", "base_link", rclpy.time.Time()):
            break
    if not buf.can_transform("odom", "base_link", rclpy.time.Time()):
        print("FAILED: no odom->base_link transform. The VESC is not producing "
              "odometry:\n  grep -c 'Out-of-sync' ~/rr_logs/base.log\n"
              "  bash ~/rr/kill_base.sh && ~/rr/rr_bringup.sh")
        return 1

    origin = None
    cont_yaw = None      # continuous (unwrapped) heading
    yaw_origin = None
    path = 0.0           # integrated along the actual route
    last_xy = None

    print("ready. drive; 'r'+Enter resets to zero; Ctrl+C quits\n")
    print("  straight-line   path-length   heading")
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                tf = buf.lookup_transform("odom", "base_link",
                                          rclpy.time.Time())
            except Exception:  # noqa: BLE001 - transient dropouts are fine
                continue

            x = tf.transform.translation.x
            y = tf.transform.translation.y
            raw_yaw = yaw_of(tf.transform.rotation)

            if origin is None:
                origin, last_xy = (x, y), (x, y)
                cont_yaw, yaw_origin = raw_yaw, raw_yaw
            cont_yaw = unwrap(cont_yaw, raw_yaw)

            path += math.hypot(x - last_xy[0], y - last_xy[1])
            last_xy = (x, y)
            straight = math.hypot(x - origin[0], y - origin[1])
            heading = math.degrees(cont_yaw - yaw_origin)

            sys.stdout.write("\r    %7.3f m     %7.3f m   %+8.2f deg   "
                             % (straight, path, heading))
            sys.stdout.flush()

            # Non-blocking check so driving is never interrupted by input.
            if select.select([sys.stdin], [], [], 0.0)[0]:
                if sys.stdin.readline().strip().lower().startswith("r"):
                    origin, last_xy = (x, y), (x, y)
                    cont_yaw, yaw_origin = raw_yaw, raw_yaw
                    path = 0.0
                    print("\n  -- reset --")
    except KeyboardInterrupt:
        print("\n")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
