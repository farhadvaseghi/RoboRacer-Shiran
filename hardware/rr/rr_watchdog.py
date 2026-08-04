#!/usr/bin/env python3
"""rr_watchdog.py -- watch whether the car is really where it thinks it is.

Runs at 2 Hz and records four independent signals, so that after a bump you can
look at the seconds before it and see which one moved FIRST, instead of
guessing:

  1. clearance_left / clearance_right   raw /scan, nearest return beside the car
  2. align_pct                          % of scan returns landing on map walls
  3. cte                                distance from the car to /control/plan
  4. slip                               wheel-odom motion vs slam motion

Why more than one: a pose-vs-plan check (signal 3) only works when the pose is
right. The dangerous failure is localization drifting while staying
self-consistent -- the controller reports ~0 cross-track error and drives into a
wall with every internal number looking healthy. Signals 1 and 2 come from the
raw LiDAR and do not trust localization at all, so they still see it.

Signal 4 catches contact after the fact: scraping a wall makes the wheels turn
while slam says the car is not moving.

Everything is written to ~/rr_logs/watchdog_<timestamp>.csv and published on
/perception/health.

STOPPING: with stop_on_danger:=true the watchdog publishes an EMPTY
/control/plan when a hard threshold is crossed. The pure-pursuit controller
drops its path and stops. That needs no re-wiring of /drive, so it composes
with the AEB rather than fighting it. Default is false: report only.
"""

import math
import os
import time
from datetime import datetime

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

LATCHED = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
)

LOG_DIR = os.path.expanduser('~/rr_logs')
# Laser mounting offset in base_link (matches the static TF: 0.27 0 0.11).
LASER_X = 0.27


class Watchdog(Node):
    def __init__(self):
        super().__init__('rr_watchdog')

        self.declare_parameter('rate_hz', 2.0)
        # Corridor is 1.0-1.5 m and the car is ~0.26 m wide, so centred leaves
        # 0.35-0.6 m per side. 0.20 m means genuinely off-centre; 0.12 m is
        # about to touch.
        self.declare_parameter('clearance_warn', 0.20)
        self.declare_parameter('clearance_stop', 0.12)
        # Healthy alignment measured 91% parked. 70/50 are wide margins.
        self.declare_parameter('align_warn', 70.0)
        self.declare_parameter('align_stop', 50.0)
        self.declare_parameter('cte_warn', 0.35)
        # Band beside the car used for the clearance check.
        self.declare_parameter('side_band_back', -0.25)
        self.declare_parameter('side_band_front', 0.60)
        self.declare_parameter('stop_on_danger', False)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.rate = g('rate_hz')
        self.clear_warn = g('clearance_warn')
        self.clear_stop = g('clearance_stop')
        self.align_warn = g('align_warn')
        self.align_stop = g('align_stop')
        self.cte_warn = g('cte_warn')
        self.band_back = g('side_band_back')
        self.band_front = g('side_band_front')
        self.stop_on_danger = g('stop_on_danger')

        self.scan = None
        self.grid = None
        self.near = None          # map walls dilated by one cell
        self.plan = None
        self.odom = None
        self.history = []         # (t, odom_xy, slam_xy) for the slip check

        self.buf = Buffer()
        TransformListener(self.buf, self)
        self.create_subscription(LaserScan, '/scan', self._on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, LATCHED)
        self.create_subscription(Path, '/control/plan', self._on_plan, LATCHED)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)

        self.health_pub = self.create_publisher(String, '/perception/health', 10)
        self.plan_pub = self.create_publisher(Path, '/control/plan', LATCHED)

        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(LOG_DIR, 'watchdog_%s.csv' % stamp)
        os.makedirs(LOG_DIR, exist_ok=True)
        self.csv = open(self.csv_path, 'w', buffering=1)
        self.csv.write('t,clear_left,clear_right,fwd,align_pct,cte,'
                       'odom_move,slam_move,slip,verdict\n')

        self.create_timer(1.0 / self.rate, self.tick)
        self.get_logger().info(
            'rr_watchdog: %.1f Hz, clearance warn/stop %.2f/%.2f m, align '
            'warn/stop %.0f/%.0f%%, cte warn %.2f m, stop_on_danger=%s'
            % (self.rate, self.clear_warn, self.clear_stop, self.align_warn,
               self.align_stop, self.cte_warn, self.stop_on_danger)
        )
        self.get_logger().info('logging to %s' % self.csv_path)

    # ------------------------------------------------------------- inputs
    def _on_scan(self, msg):
        self.scan = msg

    def _on_plan(self, msg):
        self.plan = msg

    def _on_odom(self, msg):
        self.odom = msg

    def _on_map(self, msg):
        self.grid = msg
        data = np.array(msg.data, dtype=np.int16).reshape(
            msg.info.height, msg.info.width)
        occupied = data >= 50
        near = occupied.copy()
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                near |= np.roll(np.roll(occupied, dy, 0), dx, 1)
        self.near = near
        self.get_logger().info('map cached %dx%d for the alignment check'
                               % (msg.info.width, msg.info.height))

    # ------------------------------------------------------------ signals
    def scan_points(self):
        """Scan returns as (x, y) in base_link."""
        r = np.asarray(self.scan.ranges, dtype=np.float64)
        a = self.scan.angle_min + np.arange(r.size) * self.scan.angle_increment
        ok = np.isfinite(r) & (r > 0.06) & (r < min(self.scan.range_max, 12.0))
        r, a = r[ok], a[ok]
        return LASER_X + r * np.cos(a), r * np.sin(a), r, a

    def clearances(self, x, y):
        """Nearest obstacle beside the car, left and right."""
        band = (x > self.band_back) & (x < self.band_front)
        left = y[band & (y > 0)]
        right = y[band & (y < 0)]
        return (float(np.min(left)) if left.size else float('inf'),
                float(np.min(-right)) if right.size else float('inf'))

    def forward_distance(self, x, y):
        ahead = (x > 0) & (np.abs(y) < 0.17)
        return float(np.min(x[ahead])) if ahead.any() else float('inf')

    def alignment(self, r, a):
        """% of returns landing on a map wall, using the current TF."""
        if self.near is None or self.scan is None:
            return float('nan')
        try:
            tf = self.buf.lookup_transform('map', self.scan.header.frame_id,
                                           rclpy.time.Time())
        except Exception:  # noqa: BLE001 - absent TF is a result, not a crash
            return float('nan')
        q = tf.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y ** 2 + q.z ** 2))
        px, py = tf.transform.translation.x, tf.transform.translation.y
        info = self.grid.info
        ang = a + yaw
        cx = ((px + r * np.cos(ang) - info.origin.position.x)
              / info.resolution).astype(int)
        cy = ((py + r * np.sin(ang) - info.origin.position.y)
              / info.resolution).astype(int)
        inside = (cx >= 0) & (cx < info.width) & (cy >= 0) & (cy < info.height)
        if not inside.any():
            return 0.0
        return 100.0 * self.near[cy[inside], cx[inside]].sum() / r.size

    def car_xy(self):
        try:
            tf = self.buf.lookup_transform('map', 'base_link',
                                           rclpy.time.Time())
            return tf.transform.translation.x, tf.transform.translation.y
        except Exception:  # noqa: BLE001
            return None

    def cross_track(self, pos):
        if self.plan is None or not self.plan.poses or pos is None:
            return float('nan')
        pts = np.array([[p.pose.position.x, p.pose.position.y]
                        for p in self.plan.poses])
        return float(np.min(np.hypot(pts[:, 0] - pos[0], pts[:, 1] - pos[1])))

    def slip(self, pos):
        """Wheel-odom travel vs slam travel over ~2 s.

        Scraping a wall spins the wheels while the car does not move, so odom
        grows and slam does not. Ratio near 1 is healthy.
        """
        if self.odom is None or pos is None:
            return float('nan'), float('nan'), float('nan')
        now = time.monotonic()
        o = self.odom.pose.pose.position
        self.history.append((now, (o.x, o.y), pos))
        self.history = [h for h in self.history if now - h[0] <= 2.0]
        if len(self.history) < 3:
            return float('nan'), float('nan'), float('nan')
        t0 = self.history[0]
        odom_move = math.hypot(o.x - t0[1][0], o.y - t0[1][1])
        slam_move = math.hypot(pos[0] - t0[2][0], pos[1] - t0[2][1])
        if odom_move < 0.05:
            return odom_move, slam_move, float('nan')
        return odom_move, slam_move, slam_move / odom_move

    # --------------------------------------------------------------- loop
    def tick(self):
        if self.scan is None:
            self.publish('NO SCAN', {})
            return

        x, y, r, a = self.scan_points()
        left, right = self.clearances(x, y)
        fwd = self.forward_distance(x, y)
        align = self.alignment(r, a)
        pos = self.car_xy()
        cte = self.cross_track(pos)
        odom_move, slam_move, slip = self.slip(pos)

        worst = min(left, right)
        verdict = 'OK'
        if worst < self.clear_stop:
            verdict = 'DANGER: clearance %.2f m' % worst
        elif not math.isnan(align) and align < self.align_stop:
            verdict = 'DANGER: alignment %.0f%% -- localization lost' % align
        elif worst < self.clear_warn:
            verdict = 'WARN: clearance %.2f m' % worst
        elif not math.isnan(align) and align < self.align_warn:
            verdict = 'WARN: alignment %.0f%%' % align
        elif not math.isnan(cte) and cte > self.cte_warn:
            verdict = 'WARN: off plan by %.2f m' % cte
        elif not math.isnan(slip) and slip < 0.5 and odom_move > 0.1:
            verdict = 'WARN: wheels slipping (slam moved %.0f%% of odom)' % (
                100 * slip)

        self.csv.write('%.3f,%.3f,%.3f,%.3f,%.1f,%.3f,%.3f,%.3f,%.3f,%s\n' % (
            time.time(), left, right, fwd, align, cte,
            odom_move, slam_move, slip, verdict.replace(',', ';')))

        if verdict.startswith('DANGER'):
            self.get_logger().error(verdict)
            if self.stop_on_danger:
                empty = Path()
                empty.header.frame_id = 'map'
                empty.header.stamp = self.get_clock().now().to_msg()
                self.plan_pub.publish(empty)
                self.get_logger().error('published EMPTY /control/plan -- the '
                                        'controller will stop')
        elif verdict.startswith('WARN'):
            self.get_logger().warn(verdict)

        self.publish(verdict, {
            'L': left, 'R': right, 'fwd': fwd, 'align': align, 'cte': cte,
        })

    def publish(self, verdict, values):
        parts = []
        for key, value in values.items():
            parts.append('%s=%s' % (key, 'inf' if math.isinf(value)
                                    else ('nan' if math.isnan(value)
                                          else '%.2f' % value)))
        self.health_pub.publish(String(data='%s | %s' % (verdict, ' '.join(parts))))


def main():
    rclpy.init()
    node = Watchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.csv.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
