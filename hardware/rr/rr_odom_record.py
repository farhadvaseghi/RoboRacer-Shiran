#!/usr/bin/env python3
"""Record the whole odometry -> pose chain during a run, on the car.

Written to settle one question: when the car visibly moves but Foxglove shows
no movement, is the DATA wrong or is the DISPLAY behind? Foxglove cannot
answer that about itself, so this samples the same chain locally.

Three distance measures are recorded, and comparing them localises the fault
without any further guessing:

  wheel speed integral    what the VESC says the car travelled
  odom->base_link path    what rr_gyro_odom integrated from that + the gyro
  map->base_link path     the pose Foxglove actually draws

  moves, moves, moves     -> the data is FINE; the display was behind (CPU /
                             foxglove_bridge starvation is the usual cause)
  moves, flat,  flat      -> rr_gyro_odom is not integrating (its /odom speed
                             subscription is the thing to look at)
  moves, moves, flat      -> slam's map->odom is cancelling real motion, or
                             map->odom is dead (no localisation)
  flat,  flat,  flat      -> the VESC is reporting no speed at all

Sampling uses a ROS timer inside a normal spin. A previous tool drove its loop
with one spin_once per sample, serviced one callback per sample while /tf
arrived at ~100 Hz, and fell 2.2 s behind -- every number it produced was
scored against a pose the car had already left. Do not reintroduce that.

Usage:  python3 rr_odom_record.py [seconds] [label]
        Ctrl+C stops early and still writes the summary.
"""

import csv
import math
import os
import sys
import time

os.environ.setdefault('ROS_DOMAIN_ID', '7')

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

SAMPLE_HZ = 10.0
LOG_DIR = os.path.expanduser('~/rr_logs')

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 120.0
LABEL = sys.argv[2] if len(sys.argv) > 2 else 'run'

# Above this the car counts as moving, matching rr_gyro_odom's STILL_SPEED_MPS.
MOVING_MPS = 0.02


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * q.z * q.z)


def read_cpu():
    """Return (idle_jiffies, total_jiffies) from /proc/stat."""
    with open('/proc/stat') as fh:
        parts = [float(v) for v in fh.readline().split()[1:]]
    return parts[3], sum(parts)


class OdomRecorder(Node):
    def __init__(self):
        super().__init__('rr_odom_record')

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        self.odom_speed = None
        self.gyro = None
        self.drive_speed = None

        best_effort = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(Odometry, '/odom', self._on_odom, 20)
        self.create_subscription(Odometry, '/odom_gyro', self._on_gyro, 20)
        # /drive is published by the controller; best-effort matches any
        # publisher reliability so this cannot silently fail to connect.
        self.create_subscription(AckermannDriveStamped, '/drive',
                                 self._on_drive, best_effort)

        os.makedirs(LOG_DIR, exist_ok=True)
        self.path = os.path.join(
            LOG_DIR, 'odom_record_%s_%s.csv'
            % (LABEL, time.strftime('%Y%m%d_%H%M%S')))
        self.fh = open(self.path, 'w', newline='')
        self.csv = csv.writer(self.fh)
        self.csv.writerow([
            't', 'wheel_speed', 'drive_cmd',
            'gyro_x', 'gyro_y', 'gyro_yaw_deg',
            'ob_x', 'ob_y', 'ob_yaw_deg', 'ob_age',
            'mo_x', 'mo_y', 'mo_yaw_deg',
            'mb_x', 'mb_y', 'mb_yaw_deg', 'mb_age',
            'cpu_idle_pct',
        ])

        self.t0 = time.time()
        self.samples = 0
        self.moving = 0
        self.wheel_integral = 0.0
        self.ob_path = 0.0
        self.mb_path = 0.0
        self.ob_prev = None
        self.mb_prev = None
        self.ob_missing = 0
        self.mb_missing = 0
        self.ages = []
        self.mo_seen = []
        self.last_t = None
        self.cpu_prev = read_cpu()
        self.cpu_min = 100.0
        self.last_print = 0.0

        print('recording -> %s' % self.path)
        print('%-6s %8s %8s %10s %10s %8s'
              % ('t', 'wheel', 'ob_path', 'mb_path', 'mb_age', 'cpu_idle'))

        self.create_timer(1.0 / SAMPLE_HZ, self._tick)

    def _on_odom(self, msg):
        self.odom_speed = msg.twist.twist.linear.x

    def _on_gyro(self, msg):
        self.gyro = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                     math.degrees(yaw_of(msg.pose.pose.orientation)))

    def _on_drive(self, msg):
        self.drive_speed = msg.drive.speed

    def _lookup(self, parent, child):
        """Latest available transform, plus how old it is. None if missing."""
        try:
            tf = self.buffer.lookup_transform(parent, child, Time(),
                                              timeout=Duration(seconds=0.0))
        except Exception:
            return None
        stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        return (tf.transform.translation.x, tf.transform.translation.y,
                math.degrees(yaw_of(tf.transform.rotation)), age)

    def _tick(self):
        now = time.time()
        t = now - self.t0
        dt = 0.0 if self.last_t is None else now - self.last_t
        self.last_t = now

        idle, total = read_cpu()
        d_idle = idle - self.cpu_prev[0]
        d_total = total - self.cpu_prev[1]
        self.cpu_prev = (idle, total)
        cpu_idle = 100.0 * d_idle / d_total if d_total > 0 else float('nan')
        if not math.isnan(cpu_idle):
            self.cpu_min = min(self.cpu_min, cpu_idle)

        speed = self.odom_speed
        if speed is not None:
            self.wheel_integral += abs(speed) * dt
            if abs(speed) > MOVING_MPS:
                self.moving += 1

        ob = self._lookup('odom', 'base_link')
        mo = self._lookup('map', 'odom')
        mb = self._lookup('map', 'base_link')

        if ob is None:
            self.ob_missing += 1
        else:
            if self.ob_prev is not None:
                self.ob_path += math.hypot(ob[0] - self.ob_prev[0],
                                           ob[1] - self.ob_prev[1])
            self.ob_prev = ob

        if mb is None:
            self.mb_missing += 1
        else:
            if self.mb_prev is not None:
                self.mb_path += math.hypot(mb[0] - self.mb_prev[0],
                                           mb[1] - self.mb_prev[1])
            self.mb_prev = mb
            self.ages.append(mb[3])

        if mo is not None:
            self.mo_seen.append((mo[0], mo[1]))

        g = self.gyro or (float('nan'),) * 3
        self.csv.writerow([
            '%.3f' % t,
            '' if speed is None else '%.4f' % speed,
            '' if self.drive_speed is None else '%.4f' % self.drive_speed,
            '%.4f' % g[0], '%.4f' % g[1], '%.3f' % g[2],
            *(['', '', '', ''] if ob is None else
              ['%.4f' % ob[0], '%.4f' % ob[1], '%.3f' % ob[2], '%.3f' % ob[3]]),
            *(['', '', ''] if mo is None else
              ['%.4f' % mo[0], '%.4f' % mo[1], '%.3f' % mo[2]]),
            *(['', '', '', ''] if mb is None else
              ['%.4f' % mb[0], '%.4f' % mb[1], '%.3f' % mb[2], '%.3f' % mb[3]]),
            '%.1f' % cpu_idle,
        ])
        self.samples += 1

        if t - self.last_print >= 2.0:
            self.last_print = t
            print('%-6.1f %8.3f %8.2f %10.2f %10s %8.1f'
                  % (t, speed if speed is not None else float('nan'),
                     self.ob_path, self.mb_path,
                     '-' if mb is None else '%.3f' % mb[3], cpu_idle))

        if t >= DURATION:
            raise KeyboardInterrupt

    def summary(self):
        self.fh.close()
        dur = time.time() - self.t0
        print('\n--- %s: %d samples over %.1f s ---' % (LABEL, self.samples, dur))
        if not self.samples:
            return

        print('moving samples          %d / %d (%.0f%%)'
              % (self.moving, self.samples,
                 100.0 * self.moving / self.samples))
        print('wheel speed integral    %.2f m   (what the VESC reported)'
              % self.wheel_integral)
        print('odom->base_link path    %.2f m   (what rr_gyro_odom integrated)'
              % self.ob_path)
        print('map->base_link path     %.2f m   (the pose Foxglove draws)'
              % self.mb_path)
        if self.ob_missing:
            print('odom->base_link MISSING on %d samples' % self.ob_missing)
        if self.mb_missing:
            print('map->base_link MISSING on %d samples  <-- no localisation'
                  % self.mb_missing)
        if self.ages:
            mean_age = sum(self.ages) / len(self.ages)
            print('map->base_link age      mean %.3f s, max %.3f s'
                  % (mean_age, max(self.ages)))
            if mean_age > 0.30:
                print('  WARNING: stale pose -- these distances are suspect')
        if self.mo_seen:
            xs = [p[0] for p in self.mo_seen]
            ys = [p[1] for p in self.mo_seen]
            print('map->odom range         x %.2f..%.2f  y %.2f..%.2f'
                  % (min(xs), max(xs), min(ys), max(ys)))
        print('cpu idle                min %.1f%%' % self.cpu_min)

        # The verdict, stated plainly so it is not re-derived by eye later.
        wheel = self.wheel_integral
        print('\nverdict:')
        if wheel < 0.5:
            print('  The VESC reported almost no motion. If the car really '
                  'drove, the fault is upstream of everything here.')
        elif self.ob_path < 0.2 * wheel:
            print('  Wheels moved but odom->base_link did not: rr_gyro_odom '
                  'is not integrating. Check its /odom subscription.')
        elif self.mb_path < 0.2 * self.ob_path:
            print('  odom->base_link moved but map->base_link did not: slam '
                  'is cancelling the motion, or map->odom is dead.')
        else:
            print('  All three advanced together -- the odometry data is '
                  'FINE. A frozen Foxglove view was the display lagging, '
                  'not the pose. Check cpu idle above.')
        print('\ncsv: %s' % self.path)


def main():
    rclpy.init()
    node = OdomRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.summary()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
