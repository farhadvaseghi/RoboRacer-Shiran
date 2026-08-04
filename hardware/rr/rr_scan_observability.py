#!/usr/bin/env python3
"""Measure whether the LiDAR alone can constrain the robot's pose here.

rr_scan_align.py answers "does the scan sit on the walls". This answers the
question that decides whether odometry can be dropped: if the pose is nudged,
does the scan notice? A scan that scores just as well 0.5 m further down the
corridor cannot tell the localizer where the robot is along that corridor, and
a laser-only odometry will slide.

Metric is the likelihood field used by every scan matcher: the mean distance
from each scan endpoint to the nearest occupied map cell. Lower is better.
The pose is perturbed in three independent axes -- longitudinal (robot
forward), lateral (robot left), and yaw -- and the cost curve is printed for
each. A sharp bowl means that axis is observable from the scan alone; a flat
bottom means it is not, and dead reckoning is carrying that axis today.

Read-only: subscribes to /map, /scan and TF, publishes nothing, changes no
parameters. Safe to run against a live stack.

Run on the car:  ROS_DOMAIN_ID=7 python3 ~/rr/rr_scan_observability.py
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

TF_WAIT_SECONDS = 12.0
INPUT_WAIT_SECONDS = 20.0

# Endpoints further than this from any wall are treated as this far away, so a
# handful of returns through a doorway cannot dominate the mean.
DIST_CAP_M = 0.50

# How much the mean error may grow before an axis counts as "moved". The map
# is 0.05 m/cell, so 0.02 m is a real change, not discretization.
COST_RISE_M = 0.02

LONG_OFFSETS = [-0.60, -0.40, -0.30, -0.20, -0.15, -0.10, -0.05,
                0.0, 0.05, 0.10, 0.15, 0.20, 0.30, 0.40, 0.60]
YAW_OFFSETS = [-8.0, -5.0, -3.0, -2.0, -1.0, -0.5,
               0.0, 0.5, 1.0, 2.0, 3.0, 5.0, 8.0]


def collect(node, probe_state, buf):
    """Spin until /map, /scan and the TF tree are all usable."""
    end = time.monotonic() + INPUT_WAIT_SECONDS
    while rclpy.ok() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        if probe_state["maps"] and len(probe_state["scans"]) >= 3:
            break
    if not probe_state["maps"] or len(probe_state["scans"]) < 3:
        missing = []
        if not probe_state["maps"]:
            missing.append("/map")
        if len(probe_state["scans"]) < 3:
            missing.append("/scan")
        print("FAILED: no %s within %.0fs. Run rr_scan_align.py -- it prints "
              "the full TF/input diagnosis." % (" and ".join(missing),
                                                INPUT_WAIT_SECONDS))
        return None

    frame = probe_state["scans"][-1].header.frame_id
    end = time.monotonic() + TF_WAIT_SECONDS
    while rclpy.ok() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        if buf.can_transform("map", frame, rclpy.time.Time()):
            break
    if not buf.can_transform("map", frame, rclpy.time.Time()):
        print("FAILED: no transform map -> %s within %.0fs. Run "
              "rr_scan_align.py for the full diagnosis." % (frame,
                                                            TF_WAIT_SECONDS))
        return None
    return frame


def describe(name, unit, offsets, costs, rise_threshold):
    """Print one axis's cost curve and how far the pose can slide unnoticed."""
    base = costs[offsets.index(0.0)]
    print("\n  %s" % name)
    print("    offset : " + " ".join("%7.2f" % o for o in offsets))
    print("    cost m : " + " ".join("%7.3f" % c for c in costs))
    print("    rise   : " + " ".join("%7.3f" % (c - base) for c in costs))

    # How far from 0 the pose can move before the cost rises past the
    # threshold -- the width of the flat bottom, i.e. the blind zone. Take the
    # breach closest to zero on each side, not the outermost one.
    breached = [(o, c - base) for o, c in zip(offsets, costs)
                if c - base > rise_threshold]
    negatives = [o for o, _rise in breached if o < 0]
    positives = [o for o, _rise in breached if o > 0]
    slack_neg = max(negatives) if negatives else None
    slack_pos = min(positives) if positives else None

    def fmt(value, edge):
        # None means the sweep never breached the threshold on that side, so
        # the honest statement is "not within the range we looked at".
        if value is None:
            return "none within %+.2f%s" % (edge, unit)
        return "%+.2f%s" % (value, unit)

    print("    the scan first notices a move at %s / %s (cost +%.3f m)"
          % (fmt(slack_neg, offsets[0]), fmt(slack_pos, offsets[-1]),
             rise_threshold))
    best = min(range(len(costs)), key=lambda i: costs[i])
    spread = max(costs) - min(costs)
    print("    minimum sits at %+.2f%s (cost %.3f m); total spread %.3f m"
          % (offsets[best], unit, costs[best], spread))
    if abs(offsets[best]) > 1e-9:
        print("    NOTE: the believed pose is NOT the best-fitting pose -- the")
        print("          scan prefers %+.2f%s from where the stack thinks it is."
              % (offsets[best], unit))
    return offsets[best], base, slack_neg, slack_pos


def main():
    try:
        from scipy import ndimage
    except ImportError:
        print("FAILED: scipy is not importable, needed for the distance "
              "transform. rr_despeckle.py uses it, so check the interpreter.")
        return 1

    rclpy.init()
    node = rclpy.create_node("rr_scan_observability")
    state = {"maps": [], "scans": []}
    node.create_subscription(OccupancyGrid, "/map", state["maps"].append,
                             LATCHED)
    node.create_subscription(LaserScan, "/scan", state["scans"].append, SENSOR)
    buf = Buffer()
    TransformListener(buf, node)

    print("collecting /map, /scan and TF ...")
    frame = collect(node, state, buf)
    if frame is None:
        return 1

    scan = state["scans"][-1]
    grid_msg = state["maps"][0]
    tf = buf.lookup_transform("map", frame, rclpy.time.Time())

    info = grid_msg.info
    grid = np.array(grid_msg.data, dtype=np.int16).reshape(info.height,
                                                           info.width)
    occupied = grid >= 50
    if not occupied.any():
        print("FAILED: the map has no occupied cells at all.")
        return 1

    # Distance in metres from every cell to the nearest occupied cell. This is
    # the likelihood field: evaluating a scan against it is what a scan
    # matcher does internally, so the curve below is the cost the matcher sees.
    dist = ndimage.distance_transform_edt(~occupied) * info.resolution
    dist = np.minimum(dist, DIST_CAP_M)

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
        print("FAILED: only %d usable scan returns." % ranges.size)
        return 1

    def cost(d_long=0.0, d_lat=0.0, d_yaw_deg=0.0):
        """Mean distance-to-wall after nudging the pose in the robot frame."""
        # Longitudinal is along the robot's heading, lateral is 90 deg left.
        sx = px + d_long * math.cos(yaw) - d_lat * math.sin(yaw)
        sy = py + d_long * math.sin(yaw) + d_lat * math.cos(yaw)
        ang = angles + yaw + math.radians(d_yaw_deg)
        xs = sx + ranges * np.cos(ang)
        ys = sy + ranges * np.sin(ang)
        cx = ((xs - ox) / res).astype(int)
        cy = ((ys - oy) / res).astype(int)
        inside = (cx >= 0) & (cx < info.width) & (cy >= 0) & (cy < info.height)
        if inside.sum() < 20:
            return DIST_CAP_M
        # Endpoints leaving the map are counted at the cap, so sliding the
        # scan off the edge can never look cheap.
        total = dist[cy[inside], cx[inside]].sum()
        total += DIST_CAP_M * (ranges.size - inside.sum())
        return total / ranges.size

    print("")
    print("map        : %dx%d @ %.3f m/cell, origin (%.2f, %.2f)"
          % (info.width, info.height, res, ox, oy))
    print("laser frame: %s at x=%.3f y=%.3f yaw=%.2f deg"
          % (frame, px, py, math.degrees(yaw)))
    print("scan       : %d usable returns" % ranges.size)
    print("metric     : mean distance from each return to the nearest wall,")
    print("             capped at %.2f m. Lower is better." % DIST_CAP_M)
    print("")
    print("POSE OBSERVABILITY FROM THE SCAN ALONE")

    long_costs = [cost(d_long=o) for o in LONG_OFFSETS]
    lat_costs = [cost(d_lat=o) for o in LONG_OFFSETS]
    yaw_costs = [cost(d_yaw_deg=o) for o in YAW_OFFSETS]

    l_best, l_base, l_neg, l_pos = describe(
        "LONGITUDINAL (robot forward, along the corridor)", "m",
        LONG_OFFSETS, long_costs, COST_RISE_M)
    describe("LATERAL (robot left, across the corridor)", "m",
             LONG_OFFSETS, lat_costs, COST_RISE_M)
    describe("YAW", "deg", YAW_OFFSETS, yaw_costs, COST_RISE_M)

    # The blind zone in each translation axis is what a laser-only odometry
    # would have to guess at. Compare the two: if longitudinal is much flatter
    # than lateral, this is the classic corridor aperture problem.
    def blind_width(costs):
        base = costs[LONG_OFFSETS.index(0.0)]
        inside = [o for o, c in zip(LONG_OFFSETS, costs)
                  if c - base <= COST_RISE_M]
        return max(inside) - min(inside)

    long_blind = blind_width(long_costs)
    lat_blind = blind_width(lat_costs)

    print("")
    print("=" * 68)
    print("blind zone: longitudinal %.2f m wide, lateral %.2f m wide"
          % (long_blind, lat_blind))
    if long_blind >= 0.30 and long_blind > 2 * max(lat_blind, 1e-6):
        print("VERDICT: APERTURE PROBLEM. The scan pins the robot across the")
        print("         corridor but not along it. Odometry is carrying the")
        print("         longitudinal axis -- dropping it will let the pose")
        print("         slide forward/back with nothing to stop it.")
    elif long_blind >= 0.30:
        print("VERDICT: WEAK. Both translation axes are loose; this pose has")
        print("         little geometry for a scan matcher to lock onto.")
    else:
        print("VERDICT: OBSERVABLE. Both translation axes are constrained by")
        print("         the scan alone at this pose, so scan-only")
        print("         localization has real signal to work with here.")
    print("=" * 68)
    print("")
    print("NOTE: this is ONE pose. The corridor's featureless stretches are")
    print("      the ones that matter -- re-run parked mid-corridor before")
    print("      trusting the verdict.")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
