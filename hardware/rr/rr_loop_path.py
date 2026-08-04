#!/usr/bin/env python3
"""rr_loop_path.py — CONTINUOUS-lap path publisher (receding horizon).

The custom pure_pursuit_controller stops at the LAST point of the path (it treats it
as the goal), so a static closed loop makes it "arrive" instantly and never move.
This node instead republishes a SLIDING window of the loop that always starts at the
car's nearest point and extends `horizon` metres ahead (wrapping around the loop), so
the path END is always far ahead of the car -> the controller never declares "goal
reached" -> the car laps forever.

Reads a CLOSED loop CSV (rr_record_path output, gap stitched shut), subscribes
/odometry/map, publishes nav_msgs/Path on /control/plan (RELIABLE + TRANSIENT_LOCAL,
the QoS the controller wants). Ctrl+C to stop.

Usage:  python3 rr_loop_path.py [loop.csv] [--horizon M] [--rate HZ]
        default: ~/rr_maps/lap_line_closed.csv , horizon 5.0 m , rate 5 Hz
"""
import sys, os, math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped


def load_csv(path):
    pts = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.lower().startswith('x'):
                continue
            x, y = line.split(',')[:2]
            pts.append((float(x), float(y)))
    return pts


class Looper(Node):
    def __init__(self, pts, horizon, rate):
        super().__init__('rr_loop_path')
        self.pts = pts
        self.n = len(pts)
        # per-segment lengths (wrapping i -> i+1)
        self.seg = [math.hypot(pts[(i + 1) % self.n][0] - pts[i][0],
                               pts[(i + 1) % self.n][1] - pts[i][1]) for i in range(self.n)]
        self.loop_len = sum(self.seg)
        # cap horizon so the window END never wraps back onto the car
        self.horizon = min(horizon, 0.85 * self.loop_len)
        self.car = None
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(Path, '/control/plan', qos)
        self.create_subscription(Odometry, '/odometry/map', self.odom_cb, 10)
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(
            'rr_loop_path: %d-pt loop, len %.1f m, horizon %.1f m, %.0f Hz -> /control/plan'
            % (self.n, self.loop_len, self.horizon, rate))

    def odom_cb(self, m):
        self.car = (m.pose.pose.position.x, m.pose.pose.position.y)

    def nearest_idx(self):
        cx, cy = self.car
        best_i, best_d = 0, float('inf')
        for i, (x, y) in enumerate(self.pts):
            d = (x - cx) ** 2 + (y - cy) ** 2
            if d < best_d:
                best_d, best_i = d, i
        return best_i

    def tick(self):
        if self.car is None:
            return
        start = self.nearest_idx()
        idxs = [start]
        acc = 0.0
        i = start
        while acc < self.horizon and len(idxs) <= self.n:
            acc += self.seg[i]
            i = (i + 1) % self.n
            idxs.append(i)
        p = Path()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        for k, idx in enumerate(idxs):
            x, y = self.pts[idx]
            nx, ny = self.pts[idxs[min(k + 1, len(idxs) - 1)]]
            yaw = math.atan2(ny - y, nx - x)
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.z = math.sin(yaw / 2.0)
            ps.pose.orientation.w = math.cos(yaw / 2.0)
            p.poses.append(ps)
        self.pub.publish(p)


def main():
    argv = sys.argv[1:]
    horizon, rate, pos = 5.0, 5.0, []
    i = 0
    while i < len(argv):
        a = argv[i]
        if a == '--horizon':
            horizon = float(argv[i + 1]); i += 2
        elif a == '--rate':
            rate = float(argv[i + 1]); i += 2
        else:
            pos.append(a); i += 1
    path = os.path.expanduser(pos[0] if pos else '~/rr_maps/lap_line_closed.csv')
    pts = load_csv(path)
    if len(pts) < 3:
        print('need >=3 points in %s' % path); return
    rclpy.init()
    try:
        rclpy.spin(Looper(pts, horizon, rate))
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
