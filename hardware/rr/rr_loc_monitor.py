#!/usr/bin/env python3
"""Score localization quality live, so a config change can be A/B'd.

Samples at a fixed rate and summarises:

  1. SCAN FIT -- mean distance from each scan return to the nearest wall at the
     pose the stack currently believes. If the pose is wrong, the scan stops
     sitting on the walls. Reported with the percentage of returns on a wall
     cell so the numbers line up with rr_scan_align.py.

  2. map->odom -- the correction slam applies on top of odometry. A correction
     that grows or jumps is odometry error being caught after the fact.

  3. PATH LENGTH -- distance travelled by raw odom versus the corrected pose.

  4. LOOKUP TIMING -- how old the data being scored actually is. This exists
     because the first version of this tool got it wrong: it called
     rclpy.spin_once() once per sample, which services ONE callback, while /tf
     arrives at ~100 Hz and /scan at 40 Hz. Callbacks queued up and the TF
     buffer fell behind by 2-5 seconds, so the fit was scored against a pose
     the car had long since left. Every moving-fit number that version
     produced was wrong. It now runs the sampler on a ROS timer inside a
     continuous spin, and reports its own staleness so the failure can never
     be silent again.

Read-only: subscribes only, publishes nothing, changes no parameters.

Usage:  ROS_DOMAIN_ID=7 python3 ~/rr/rr_loc_monitor.py [seconds] [label]
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
# depth 1: only the newest scan is ever wanted. A deeper queue would let old
# scans pile up and be scored long after they were taken.
SENSOR = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
)

SAMPLE_HZ = 4.0
DIST_CAP_M = 0.50
SETUP_WAIT_SECONDS = 20.0

JUMP_M = 0.03
JUMP_DEG = 1.0
MOVE_EPS_M = 0.005

# Beyond this the sampled data is too old to describe where the car is now,
# and the run should not be trusted.
STALE_WARN_S = 0.30


def yaw_of(rot):
    return math.atan2(2 * (rot.w * rot.z + rot.x * rot.y),
                      1 - 2 * (rot.y ** 2 + rot.z ** 2))


def ang_diff(a, b):
    return (a - b + math.pi) % (2 * math.pi) - math.pi


class Monitor:
    def __init__(self, node, buf, seconds, label):
        self.node, self.buf = node, buf
        self.seconds, self.label = seconds, label
        self.map_msg = None
        self.scan = None            # newest scan only, never a growing list
        self.scan_count = 0
        node.create_subscription(OccupancyGrid, "/map", self._on_map, LATCHED)
        node.create_subscription(LaserScan, "/scan", self._on_scan, SENSOR)
        self.dist = None
        self.info = None
        self.near = None

        self.fit_costs, self.fit_pcts = [], []
        self.moving_costs, self.moving_pcts = [], []
        self.mo_jumps, self.mo_yaw_jumps = [], []
        self.stamp_skew, self.pose_age = [], []
        self.last_mo = None
        self.last_pose = None
        self.last_odom = None
        self.path_pose = 0.0
        self.path_odom = 0.0
        self.samples = 0
        self.relocalizations = 0
        self.off_map = 0
        self.lookup_failures = 0
        self.start = None
        self.next_print = 0.0

    def _on_map(self, msg):
        if self.map_msg is None:
            self.map_msg = msg

    def _on_scan(self, msg):
        self.scan = msg
        self.scan_count += 1

    def _spin_for(self, seconds):
        """Service callbacks continuously, not one per sample."""
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def wait_for_inputs(self):
        end = time.monotonic() + SETUP_WAIT_SECONDS
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            if self.map_msg is not None and self.scan_count >= 3:
                break
        if self.map_msg is None or self.scan_count < 3:
            print("FAILED: no /map and /scan within %.0fs. Run "
                  "rr_scan_align.py for the full diagnosis."
                  % SETUP_WAIT_SECONDS)
            return False

        frame = self.scan.header.frame_id
        end = time.monotonic() + SETUP_WAIT_SECONDS
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            if self.buf.can_transform("map", frame, rclpy.time.Time()):
                break
        if not self.buf.can_transform("map", frame, rclpy.time.Time()):
            print("FAILED: no map -> %s transform. Is slam seeded? "
                  "python3 ~/rr/rr_seed_start.py" % frame)
            return False

        from scipy import ndimage
        self.info = self.map_msg.info
        grid = np.array(self.map_msg.data, dtype=np.int16).reshape(
            self.info.height, self.info.width)
        occupied = grid >= 50
        self.dist = np.minimum(
            ndimage.distance_transform_edt(~occupied) * self.info.resolution,
            DIST_CAP_M)
        near = occupied.copy()
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                near |= np.roll(np.roll(occupied, dy, 0), dx, 1)
        self.near = near
        self.frame = frame
        return True

    def outside_map(self, pose):
        ox = self.info.origin.position.x
        oy = self.info.origin.position.y
        return not (ox <= pose[0] <= ox + self.info.width * self.info.resolution
                    and oy <= pose[1] <= oy + self.info.height * self.info.resolution)

    def score_scan(self, scan, tf):
        ranges = np.array(scan.ranges, dtype=np.float64)
        angles = scan.angle_min + np.arange(ranges.size) * scan.angle_increment
        usable = np.isfinite(ranges) & (ranges > 0.2)
        usable &= ranges < min(scan.range_max, 12.0)
        ranges, angles = ranges[usable], angles[usable]
        if ranges.size < 20:
            return None, None

        yaw = yaw_of(tf.transform.rotation)
        px = tf.transform.translation.x
        py = tf.transform.translation.y
        ang = angles + yaw
        xs = px + ranges * np.cos(ang)
        ys = py + ranges * np.sin(ang)
        cx = ((xs - self.info.origin.position.x) / self.info.resolution).astype(int)
        cy = ((ys - self.info.origin.position.y) / self.info.resolution).astype(int)
        inside = (cx >= 0) & (cx < self.info.width) & \
                 (cy >= 0) & (cy < self.info.height)
        if inside.sum() < 20:
            return None, None
        total = self.dist[cy[inside], cx[inside]].sum()
        total += DIST_CAP_M * (ranges.size - inside.sum())
        pct = 100.0 * self.near[cy[inside], cx[inside]].sum() / ranges.size
        return total / ranges.size, pct

    def sample(self):
        """Timer callback. Runs inside the spin, so the buffers stay current."""
        if self.scan is None:
            return
        now = rclpy.time.Time()
        try:
            tf_laser = self.buf.lookup_transform("map", self.frame, now)
            tf_mo = self.buf.lookup_transform("map", "odom", now)
            tf_ob = self.buf.lookup_transform("odom", "base_link", now)
            tf_mb = self.buf.lookup_transform("map", "base_link", now)
        except Exception:  # noqa: BLE001 - a dropped lookup is not fatal
            self.lookup_failures += 1
            return

        odom_now = (tf_ob.transform.translation.x, tf_ob.transform.translation.y)
        moving = False
        if self.last_odom is not None:
            step = math.hypot(odom_now[0] - self.last_odom[0],
                              odom_now[1] - self.last_odom[1])
            moving = step > MOVE_EPS_M

        cost, pct = self.score_scan(self.scan, tf_laser)
        if cost is not None:
            self.fit_costs.append(cost)
            self.fit_pcts.append(pct)
            if moving:
                self.moving_costs.append(cost)
                self.moving_pcts.append(pct)

        mo = (tf_mo.transform.translation.x, tf_mo.transform.translation.y,
              yaw_of(tf_mo.transform.rotation))
        if self.last_mo is not None:
            self.mo_jumps.append(math.hypot(mo[0] - self.last_mo[0],
                                            mo[1] - self.last_mo[1]))
            self.mo_yaw_jumps.append(
                abs(math.degrees(ang_diff(mo[2], self.last_mo[2]))))
        self.last_mo = mo

        pose = (tf_mb.transform.translation.x, tf_mb.transform.translation.y)
        if self.last_pose is not None:
            relocalized = bool(self.mo_jumps) and self.mo_jumps[-1] > JUMP_M
            if relocalized:
                self.relocalizations += 1
            else:
                self.path_pose += math.hypot(pose[0] - self.last_pose[0],
                                             pose[1] - self.last_pose[1])
            self.path_odom += math.hypot(odom_now[0] - self.last_odom[0],
                                         odom_now[1] - self.last_odom[1])

        t_mb = tf_mb.header.stamp.sec + tf_mb.header.stamp.nanosec * 1e-9
        t_ob = tf_ob.header.stamp.sec + tf_ob.header.stamp.nanosec * 1e-9
        self.stamp_skew.append(t_ob - t_mb)
        self.pose_age.append(
            self.node.get_clock().now().nanoseconds * 1e-9 - t_mb)

        if self.outside_map(pose):
            self.off_map += 1
        self.last_pose, self.last_odom = pose, odom_now
        self.samples += 1

        elapsed = time.monotonic() - self.start
        if elapsed >= self.next_print and self.fit_costs:
            self.next_print = elapsed + 2.0
            flag = "!" if self.outside_map(pose) else " "
            print("%5.0fs  %6.3f  %5.1f%%  (%6.2f,%6.2f)%s (%6.2f,%6.2f)"
                  "   %5.2f / %5.2f m  age %.2fs"
                  % (elapsed, self.fit_costs[-1], self.fit_pcts[-1],
                     pose[0], pose[1], flag, mo[0], mo[1],
                     self.path_pose, self.path_odom, self.pose_age[-1]))

    def run(self):
        print("sampling for %.0fs at %.0f Hz -- move the car now\n"
              % (self.seconds, SAMPLE_HZ))
        print("   t     fit(m)  on-wall     pose xy        map->odom xy     "
              "pose/odom path   age")
        self.start = time.monotonic()
        timer = self.node.create_timer(1.0 / SAMPLE_HZ, self.sample)
        self._spin_for(self.seconds)
        self.node.destroy_timer(timer)
        self.report()

    def report(self):
        print("")
        print("=" * 72)
        print("LOCALIZATION QUALITY  [%s]" % self.label)
        print("=" * 72)
        if not self.fit_costs:
            print("no usable samples (%d lookup failures)" % self.lookup_failures)
            return
        costs = np.array(self.fit_costs)
        pcts = np.array(self.fit_pcts)

        # Trust check FIRST: if the data was stale, nothing below means anything.
        age = np.array(self.pose_age) if self.pose_age else np.array([0.0])
        skew = np.array(self.stamp_skew) if self.stamp_skew else np.array([0.0])
        print("LOOKUP TIMING  (read this FIRST -- it validates everything else)")
        print("  pose age vs wall clock      : mean %+.3f s  max %+.3f s"
              % (age.mean(), age.max()))
        print("  odom stamp minus pose stamp : mean %+.3f s  max %+.3f s"
              % (skew.mean(), skew.max()))
        if age.mean() > STALE_WARN_S:
            print("  >>> STALE (over %.2f s). The scan is being scored against"
                  % STALE_WARN_S)
            print("      a pose the car has already left. DISCARD this run.")
        else:
            print("  fresh -- the numbers below describe the car's actual pose.")

        print("")
        print("samples            : %d  (%d lookup failures)"
              % (self.samples, self.lookup_failures))
        print("")
        print("SCAN FIT  (mean distance from each return to a wall; lower=better)")
        print("  ALL SAMPLES     mean %.3f m  median %.3f m  p90 %.3f m  worst %.3f m"
              % (costs.mean(), np.median(costs), np.percentile(costs, 90),
                 costs.max()))
        print("                  on-wall mean %.1f%%  worst %.1f%%"
              % (pcts.mean(), pcts.min()))
        if len(self.moving_costs) >= 5:
            mcosts = np.array(self.moving_costs)
            mpcts = np.array(self.moving_pcts)
            print("  WHILE MOVING    mean %.3f m  median %.3f m  p90 %.3f m  worst %.3f m"
                  % (mcosts.mean(), np.median(mcosts),
                     np.percentile(mcosts, 90), mcosts.max()))
            print("                  on-wall mean %.1f%%  worst %.1f%%"
                  % (mpcts.mean(), mpcts.min()))
            print("                  %d of %d samples were moving (%.0f%%)"
                  % (len(mcosts), len(costs), 100.0 * len(mcosts) / len(costs)))
            print("  --> COMPARE PROFILES ON THE 'WHILE MOVING' ROW.")
        else:
            print("  WHILE MOVING    only %d moving samples -- the car was parked"
                  % len(self.moving_costs))

        print("")
        print("map->odom CORRECTION  (how hard slam works to fix odometry)")
        if self.mo_jumps:
            jumps = np.array(self.mo_jumps)
            yaws = np.array(self.mo_yaw_jumps)
            print("  per-sample step   mean %.4f m   max %.4f m"
                  % (jumps.mean(), jumps.max()))
            print("  per-sample yaw    mean %.3f deg  max %.3f deg"
                  % (yaws.mean(), yaws.max()))
            print("  correction jumps  %d over %.0f cm, %d over %.1f deg"
                  % (int((jumps > JUMP_M).sum()), JUMP_M * 100,
                     int((yaws > JUMP_DEG).sum()), JUMP_DEG))
            print("  total correction  %.3f m of accumulated pull" % jumps.sum())

        if self.off_map:
            print("")
            print("OFF THE MAP: %d of %d samples (%.0f%%) placed the car outside"
                  % (self.off_map, self.samples,
                     100.0 * self.off_map / max(self.samples, 1)))
            print("  the mapped area entirely.")

        print("")
        print("PATH LENGTH")
        print("  corrected pose travelled %.2f m ; raw odom travelled %.2f m"
              % (self.path_pose, self.path_odom))
        if self.relocalizations:
            print("  (%d relocalization jump(s) excluded from the pose path."
                  % self.relocalizations)
            print("   With many jumps this figure is unreliable -- use")
            print("   rr_odom_ruler.py against a tape measure instead.)")
        print("=" * 72)


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
    label = sys.argv[2] if len(sys.argv) > 2 else "unlabelled"
    rclpy.init()
    node = rclpy.create_node("rr_loc_monitor")
    buf = Buffer()
    TransformListener(buf, node)
    mon = Monitor(node, buf, seconds, label)
    print("waiting for /map, /scan and TF ...")
    if not mon.wait_for_inputs():
        return 1
    mon.run()
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
