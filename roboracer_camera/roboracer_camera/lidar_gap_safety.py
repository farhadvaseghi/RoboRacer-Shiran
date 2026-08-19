#!/usr/bin/env python3
"""lidar_gap_safety — reactive Follow-The-Gap safety layer on live LiDAR.

The custom pure_pursuit controller follows Nav2's GLOBAL plan and never reads
/scan, so at racing speed its lookahead cuts tight corners into the wall. This
node sits between the controller and the car and uses YOUR live LiDAR in real
time to steer around walls, WITHOUT changing any teammate code:

    controller /drive ─► (remapped to) /drive_raw ─┐
                                                    ├─►  this node  ─►  /drive ─► gym
    /scan  (live LiDAR)  ──────────────────────────┘

Policy — the controller keeps ownership of intent (heading toward the goal, and
stopping AT the goal); this node only overrides when a wall is close:

  * Follow-The-Gap (FTG) on /scan finds the best open direction.
  * "danger" = how close the nearest return is in the cone the controller is
    steering into (this is exactly the corner it would clip).
  * steering = blend(controller_steer, ftg_steer) weighted by danger — open
    track → follow the controller; near a wall → steer into the gap.
  * speed is capped by forward clearance and reduced in hard turns.
  * when the controller commands ~0 speed (goal reached / e-stop) we pass 0
    through, so goal-stopping and the e-stop still work.

Everything is a ROS parameter. Pure reactive geometry — no map, no teammate
topics, no camera (there is none in this sim).
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class LidarGapSafety(Node):
    def __init__(self):
        super().__init__('lidar_gap_safety')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('drive_in_topic', '/drive_raw')
        self.declare_parameter('drive_out_topic', '/drive')
        self.declare_parameter('max_range', 10.0)
        self.declare_parameter('proc_half_angle', 1.75)   # rad, ±~100° window used
        self.declare_parameter('bubble_radius', 0.30)      # m, safety bubble round nearest pt
        self.declare_parameter('gap_min_range', 0.60)      # m, closer than this = blocked
        self.declare_parameter('danger_cone', 0.35)        # rad half-angle of clearance cone
        self.declare_parameter('look_gain', 2.5)           # ctrl steer -> look direction
        self.declare_parameter('look_max', 0.9)            # rad, cap on look direction
        self.declare_parameter('danger_dist', 1.6)         # m, clearance where override begins
        self.declare_parameter('crit_dist', 0.5)           # m, clearance where override is full
        self.declare_parameter('steer_gain', 0.55)         # gap angle -> steering
        self.declare_parameter('max_steer', 0.40)          # rad
        self.declare_parameter('min_speed', 0.5)           # m/s crawl so it gets through turns
        self.declare_parameter('speed_gain', 0.9)          # speed cap = speed_gain * clearance
        self.declare_parameter('stop_eps', 0.05)           # |raw speed| below -> pass 0 through

        g = self.get_parameter
        self._max_range = float(g('max_range').value)
        self._proc = float(g('proc_half_angle').value)
        self._bubble = float(g('bubble_radius').value)
        self._gap_min = float(g('gap_min_range').value)
        self._cone = float(g('danger_cone').value)
        self._look_gain = float(g('look_gain').value)
        self._look_max = float(g('look_max').value)
        self._danger_dist = float(g('danger_dist').value)
        self._crit_dist = float(g('crit_dist').value)
        self._steer_gain = float(g('steer_gain').value)
        self._max_steer = float(g('max_steer').value)
        self._min_speed = float(g('min_speed').value)
        self._speed_gain = float(g('speed_gain').value)
        self._stop_eps = float(g('stop_eps').value)

        self._angles = None
        self._ranges = None

        self.create_subscription(LaserScan, g('scan_topic').value, self._scan_cb, 10)
        self.create_subscription(
            AckermannDriveStamped, g('drive_in_topic').value, self._drive_cb, 10)
        self._pub = self.create_publisher(
            AckermannDriveStamped, g('drive_out_topic').value, 10)

        self.get_logger().info(
            'lidar_gap_safety: %s + %s -> %s  [reactive follow-the-gap]' % (
                g('scan_topic').value, g('drive_in_topic').value,
                g('drive_out_topic').value))

    def _scan_cb(self, msg: LaserScan):
        r = np.asarray(msg.ranges, dtype=float)
        # inf/nan -> max_range (far = free); clip.
        r[~np.isfinite(r)] = self._max_range
        self._ranges = np.clip(r, 0.0, self._max_range)
        self._angles = msg.angle_min + np.arange(r.size) * msg.angle_increment
        self._ainc = msg.angle_increment

    def _best_gap_angle(self, r, a):
        """Follow-The-Gap: bubble the nearest point, take the deepest point of the
        widest remaining gap. r,a are the processing-window ranges/angles."""
        work = r.copy()
        imin = int(np.argmin(work))
        if work[imin] > 1e-3:
            half = int(math.ceil(
                math.asin(min(1.0, self._bubble / max(work[imin], 1e-3))) / self._ainc))
        else:
            half = 5
        lo, hi = max(0, imin - half), min(work.size, imin + half + 1)
        work[lo:hi] = 0.0

        free = work > self._gap_min
        if not free.any():
            return a[int(np.argmax(work))]          # nothing open — aim at the most open ray
        # widest contiguous run of free rays
        best_len = best_s = best_e = 0
        s = None
        for i, f in enumerate(free):
            if f and s is None:
                s = i
            elif not f and s is not None:
                if i - s > best_len:
                    best_len, best_s, best_e = i - s, s, i
                s = None
        if s is not None and free.size - s > best_len:
            best_s, best_e = s, free.size
        seg = work[best_s:best_e]
        return a[best_s + int(np.argmax(seg))]      # deepest point of the widest gap

    def _clearance(self, r, a, look):
        sel = (np.abs(a) <= self._cone) | (np.abs(a - look) <= self._cone)
        return float(np.min(r[sel])) if sel.any() else self._max_range

    def _drive_cb(self, msg: AckermannDriveStamped):
        raw_speed = msg.drive.speed
        raw_steer = msg.drive.steering_angle

        out = AckermannDriveStamped()
        out.header = msg.header

        # Respect the controller's stop (goal reached / e-stop), and pass through
        # untouched until the first scan arrives.
        if abs(raw_speed) < self._stop_eps or self._ranges is None:
            out.drive.speed = raw_speed
            out.drive.steering_angle = raw_steer
            self._pub.publish(out)
            return

        win = np.abs(self._angles) <= self._proc
        r, a = self._ranges[win], self._angles[win]

        ftg_steer = float(np.clip(self._steer_gain * self._best_gap_angle(r, a),
                                  -self._max_steer, self._max_steer))
        look = float(np.clip(self._look_gain * raw_steer, -self._look_max, self._look_max))
        clr = self._clearance(r, a, look)

        danger = float(np.clip(
            (self._danger_dist - clr) / max(self._danger_dist - self._crit_dist, 1e-3),
            0.0, 1.0))
        steer = float(np.clip((1.0 - danger) * raw_steer + danger * ftg_steer,
                              -self._max_steer, self._max_steer))

        cap = max(self._min_speed, self._speed_gain * clr)
        speed = min(abs(raw_speed), cap)
        speed *= (1.0 - 0.5 * abs(steer) / self._max_steer)   # ease off in hard turns
        speed = max(speed, self._min_speed)                    # keep crawling through turns
        out.drive.speed = math.copysign(speed, raw_speed)
        out.drive.steering_angle = steer
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LidarGapSafety()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
