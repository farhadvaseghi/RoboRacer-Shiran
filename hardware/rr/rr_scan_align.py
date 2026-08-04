#!/usr/bin/env python3
"""Measure how well the live /scan sits on the saved map's walls.

Prints the fraction of scan returns landing on (or next to) an occupied map
cell at the current pose, then sweeps a yaw offset to find the alignment the
scan would prefer. A clear peak away from 0 deg means the scan is rotated
relative to the map -- either the LiDAR is physically rotated on its mount, or
the map no longer matches the room.

Run on the car:  ROS_DOMAIN_ID=7 python3 ~/rr/rr_scan_align.py

On any failure this prints a full diagnostic: which inputs arrived, which TF
links are live, and the whole TF tree as tf2 sees it -- so one run is enough to
tell whether the problem is localization, the sensor, or the map.
"""

import math
import os
import sys
import time

os.environ.setdefault("ROS_DOMAIN_ID", "7")

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

LATCHED = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
)
SENSOR = QoSProfile(
    depth=5,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
)

# The TF listener needs time to fill its buffer before any lookup can succeed.
# Looking up immediately after the first scan arrives fails with "frame does
# not exist" even when TF is perfectly healthy.
TF_WAIT_SECONDS = 12.0
INPUT_WAIT_SECONDS = 20.0


class Probe:
    """Collects everything needed for the measurement AND for diagnosis."""

    def __init__(self, node):
        self.node = node
        self.maps, self.scans = [], []
        self.tf_links, self.tf_static_links = {}, {}
        node.create_subscription(OccupancyGrid, "/map", self.maps.append, LATCHED)
        node.create_subscription(LaserScan, "/scan", self.scans.append, SENSOR)
        node.create_subscription(
            TFMessage, "/tf", lambda m: self._links(self.tf_links, m), 50
        )
        node.create_subscription(
            TFMessage, "/tf_static", lambda m: self._links(self.tf_static_links, m),
            LATCHED,
        )

    @staticmethod
    def _links(store, msg):
        for tr in msg.transforms:
            key = (tr.header.frame_id, tr.child_frame_id)
            store[key] = store.get(key, 0) + 1

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def spin_until(self, predicate, seconds):
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return True
        return bool(predicate())


def report_failure(node, probe, buf, reason, target_frame=None):
    """Everything needed to tell localization / sensor / map problems apart."""
    print("")
    print("=" * 68)
    print("FAILED: %s" % reason)
    print("=" * 68)

    print("\nINPUTS")
    print("  /map    messages=%-3d publishers=%d"
          % (len(probe.maps), node.count_publishers("/map")))
    print("  /scan   messages=%-3d publishers=%d"
          % (len(probe.scans), node.count_publishers("/scan")))
    print("  /tf     publishers=%d" % node.count_publishers("/tf"))
    print("  /tf_static publishers=%d" % node.count_publishers("/tf_static"))

    print("\nTF LINKS SEEN ON THE WIRE")
    if probe.tf_static_links:
        for (parent, child), count in sorted(probe.tf_static_links.items()):
            print("  static   %-12s -> %-14s x%d" % (parent, child, count))
    else:
        print("  static   NONE")
    if probe.tf_links:
        for (parent, child), count in sorted(probe.tf_links.items()):
            print("  dynamic  %-12s -> %-14s x%d" % (parent, child, count))
    else:
        print("  dynamic  NONE")

    print("\nTF TREE AS tf2 SEES IT")
    try:
        dump = buf.all_frames_as_string().strip()
        print("  " + (dump.replace("\n", "\n  ") if dump else "(empty)"))
    except Exception as exc:  # noqa: BLE001 - diagnostics must never crash
        print("  (could not dump: %s)" % exc)

    if target_frame:
        for parent, child in (("map", "odom"), ("odom", "base_link"),
                              ("base_link", target_frame)):
            try:
                ok = buf.can_transform(parent, child, rclpy.time.Time())
                note = "OK" if ok else "MISSING"
            except Exception as exc:  # noqa: BLE001
                note = "MISSING (%s)" % type(exc).__name__
            print("  link %-10s -> %-12s %s" % (parent, child, note))

    print("\nWHAT TO DO")
    has_map_odom = any(p == "map" for p, _c in probe.tf_links)
    has_odom_base = any(c == "base_link" and p == "odom"
                        for p, c in probe.tf_links)
    if not probe.scans:
        print("  No /scan. The LiDAR driver is down or holding no connection:")
        print("    grep 'Streaming data' ~/rr_logs/base.log")
        print("    ss -tnp | grep 192.168.0.10")
    elif not has_odom_base:
        print("  No odom->base_link: the VESC is not producing odometry.")
        print("  This is the post-power-cycle out-of-sync fault. Fix with:")
        print("    grep -c 'Out-of-sync\\|Invalid end-of-frame' ~/rr_logs/base.log")
        print("    bash ~/rr/kill_base.sh && ~/rr/rr_bringup.sh")
    elif not has_map_odom:
        print("  No map->odom: slam has no pose yet. It never self-localizes.")
        print("    python3 ~/rr/rr_seed_start.py     (car parked on the origin)")
    elif not probe.maps:
        print("  No /map. map_server published once and this subscriber missed")
        print("  it, or it is down:  python3 ~/rr/rr_map_republish.py")
    else:
        print("  Inputs and TF look present; the lookup timed out anyway.")
        print("  Re-run once -- if it persists, restart slam and re-seed.")
    print("")


def main():
    rclpy.init()
    node = rclpy.create_node("rr_scan_align")
    probe = Probe(node)
    buf = Buffer()
    TransformListener(buf, node)

    print("collecting /map, /scan and TF ...")
    got_inputs = probe.spin_until(
        lambda: probe.maps and len(probe.scans) >= 3, INPUT_WAIT_SECONDS
    )
    if not got_inputs:
        missing = []
        if not probe.maps:
            missing.append("/map")
        if len(probe.scans) < 3:
            missing.append("/scan")
        report_failure(node, probe, buf, "no %s within %.0fs"
                       % (" and ".join(missing), INPUT_WAIT_SECONDS))
        return 1

    scan = probe.scans[-1]
    grid_msg = probe.maps[0]
    frame = scan.header.frame_id

    # Give the listener time to build the tree before looking anything up.
    ready = probe.spin_until(
        lambda: buf.can_transform("map", frame, rclpy.time.Time()),
        TF_WAIT_SECONDS,
    )
    if not ready:
        report_failure(
            node, probe, buf,
            'no transform map -> %s within %.0fs' % (frame, TF_WAIT_SECONDS),
            target_frame=frame,
        )
        return 1

    try:
        tf = buf.lookup_transform("map", frame, rclpy.time.Time())
    except Exception as exc:  # noqa: BLE001 - report, never crash bare
        report_failure(node, probe, buf, "lookup_transform raised %s: %s"
                       % (type(exc).__name__, exc), target_frame=frame)
        return 1

    info = grid_msg.info
    grid = np.array(grid_msg.data, dtype=np.int16).reshape(info.height, info.width)
    occupied = grid >= 50
    # Accept a hit within one cell, so discretization is not read as error.
    near = occupied.copy()
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            near |= np.roll(np.roll(occupied, dy, 0), dx, 1)

    res = info.resolution
    ox, oy = info.origin.position.x, info.origin.position.y

    q = tf.transform.rotation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
    px, py = tf.transform.translation.x, tf.transform.translation.y

    ranges = np.array(scan.ranges, dtype=np.float64)
    angles = scan.angle_min + np.arange(ranges.size) * scan.angle_increment
    usable = np.isfinite(ranges) & (ranges > 0.2)
    usable &= ranges < min(scan.range_max, 12.0)
    ranges, angles = ranges[usable], angles[usable]

    if ranges.size < 20:
        report_failure(node, probe, buf,
                       "only %d usable scan returns" % ranges.size,
                       target_frame=frame)
        return 1

    def score(offset_deg):
        ang = angles + yaw + math.radians(offset_deg)
        xs = px + ranges * np.cos(ang)
        ys = py + ranges * np.sin(ang)
        cx = ((xs - ox) / res).astype(int)
        cy = ((ys - oy) / res).astype(int)
        inside = (cx >= 0) & (cx < info.width) & (cy >= 0) & (cy < info.height)
        if not inside.any():
            return 0.0
        return 100.0 * near[cy[inside], cx[inside]].sum() / ranges.size

    sweep = [(off, score(off)) for off in np.arange(-15, 15.01, 0.5)]
    best_offset, best_value = max(sweep, key=lambda pair: pair[1])
    here = score(0.0)

    print("")
    print("map        : %dx%d @ %.3f m/cell, origin (%.2f, %.2f)"
          % (info.width, info.height, res, ox, oy))
    print("laser frame: %s at x=%.3f y=%.3f yaw=%.2f deg"
          % (frame, px, py, math.degrees(yaw)))
    print("scan       : %d usable returns" % ranges.size)
    print("")
    print("ALIGNMENT AT THE CURRENT POSE : %.1f%% of returns on walls" % here)
    print("BEST OVER A YAW SWEEP         : %+.1f deg -> %.1f%%"
          % (best_offset, best_value))
    print("top 5: " + ", ".join("%+.1f:%.0f%%" % p
                                for p in sorted(sweep, key=lambda p: -p[1])[:5]))
    print("")
    if best_value < 40.0:
        print("VERDICT: poor at EVERY offset -- the map does not match the room.")
        print("         Re-map the corridor; a yaw correction will not fix this.")
    elif abs(best_offset) <= 1.0:
        print("VERDICT: aligned. The scan sits on the walls at the current pose.")
    else:
        print("VERDICT: rotated by %+.1f deg (%.1f%% -> %.1f%%)."
              % (best_offset, here, best_value))
        print("         Either the LiDAR is turned on its mount, or the static")
        print("         base_link->laser yaw (currently 0) needs that offset.")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
