#!/usr/bin/env python3
"""Measure how odom->base_link is actually delivered.

rr_loc_monitor showed odom accumulating 51 m while only 22% of its 4 Hz
samples saw any movement -- 1.47 m per moving sample, which this car cannot
do. That means the transform is arriving in lumps, not smoothly. A scan
matcher fed a lumpy, late prior will fail even when the underlying integration
is correct, which is exactly the state the gyro fix left us in.

This subscribes to raw /tf and reports, for transforms whose child is
base_link:

  RATE      how many arrive per second, versus the ~44 Hz the IMU drives
  GAPS      the distribution of intervals between them; bursts show up as many
            near-zero gaps plus a few long ones
  LAG       message stamp versus wall clock. A prior that is consistently late
            is misaligned with the scan it will be matched against.
  SOURCES   how many distinct publishers are sending this transform. Two
            publishers fighting over one transform produces exactly these
            symptoms, so it must be ruled out explicitly.

Read-only.

Usage:  ROS_DOMAIN_ID=7 python3 ~/rr/rr_tf_rate.py [seconds] [child_frame]
        child_frame defaults to base_link; pass "odom" to watch the
        map->odom transform slam publishes.
"""

import os
import sys
import time

os.environ.setdefault("ROS_DOMAIN_ID", "7")

import numpy as np
import rclpy
from tf2_msgs.msg import TFMessage

CHILD = sys.argv[2] if len(sys.argv) > 2 else "base_link"


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
    rclpy.init()
    node = rclpy.create_node("rr_tf_rate")

    arrivals = []   # (wall_time, stamp_seconds, x, y)

    def on_tf(msg):
        wall = time.monotonic()
        for tr in msg.transforms:
            if tr.child_frame_id != CHILD:
                continue
            stamp = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
            arrivals.append((wall, stamp,
                             tr.transform.translation.x,
                             tr.transform.translation.y))

    node.create_subscription(TFMessage, "/tf", on_tf, 200)

    print("listening to /tf for transforms with child '%s' for %.0fs ...\n"
          % (CHILD, seconds))
    start = time.monotonic()
    while rclpy.ok() and time.monotonic() - start < seconds:
        rclpy.spin_once(node, timeout_sec=0.1)

    pubs = node.count_publishers("/tf")
    clock_now = node.get_clock().now().nanoseconds * 1e-9
    node.destroy_node()
    rclpy.shutdown()

    print("publishers on /tf : %d  (several nodes publish OTHER transforms too,"
          % pubs)
    print("                     so this alone does not prove a duplicate)")
    print("transforms seen   : %d in %.0fs" % (len(arrivals), seconds))
    if len(arrivals) < 5:
        print("\nVERDICT: almost nothing is publishing %s. The odometry node is"
              % CHILD)
        print("         down -- check ~/rr_logs/rr_gyro_odom.log")
        return 1

    walls = np.array([a[0] for a in arrivals])
    stamps = np.array([a[1] for a in arrivals])
    gaps = np.diff(walls)

    print("")
    print("RATE")
    print("  %.1f Hz average" % (len(arrivals) / seconds))
    print("")
    print("ARRIVAL GAPS (seconds between consecutive transforms)")
    print("  median %.4f   mean %.4f   p90 %.4f   max %.4f"
          % (np.median(gaps), gaps.mean(), np.percentile(gaps, 90), gaps.max()))
    burst = int((gaps < 0.002).sum())
    stall = int((gaps > 0.1).sum())
    print("  %d gaps under 2 ms (back-to-back burst), %d over 100 ms (stall)"
          % (burst, stall))

    # A duplicate publisher shows up as two interleaved timestamp streams: the
    # stamp going BACKWARDS between consecutive messages.
    backwards = int((np.diff(stamps) < 0).sum())
    print("")
    print("TIMESTAMP ORDER")
    print("  %d of %d consecutive transforms went BACKWARDS in time"
          % (backwards, len(stamps) - 1))
    if backwards > len(stamps) * 0.1:
        print("  >>> TWO PUBLISHERS are fighting over %s. tf2 keeps whichever" % CHILD)
        print("      arrived last, so the pose flips between two estimates.")

    lag = clock_now - stamps[-1]
    print("")
    print("STAMP vs WALL CLOCK")
    print("  newest transform is %.3f s old" % lag)

    print("")
    if stall > seconds * 0.5:
        print("VERDICT: DELIVERY IS STALLING. The transform is not arriving")
        print("         steadily, so slam's motion prior is stale whenever a")
        print("         scan needs matching. Suspect CPU starvation --")
        print("         the Jetson had only 6-10%% idle.")
    elif burst > len(gaps) * 0.3:
        print("VERDICT: BURSTY. Messages arrive in clumps, which means the")
        print("         publisher is falling behind and then flushing a queued")
        print("         backlog. The integration can still be correct while")
        print("         the timing is useless to a scan matcher.")
    else:
        print("VERDICT: delivery looks steady. The lumpiness is somewhere else.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
