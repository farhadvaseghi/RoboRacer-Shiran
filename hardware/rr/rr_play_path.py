#!/usr/bin/env python3
"""rr_play_path.py — publish a recorded racing line for the custom controller to follow.

Loads a CSV recorded by rr_record_path.py and publishes it as a nav_msgs/Path on
/control/plan (RELIABLE + TRANSIENT_LOCAL, the QoS the pure_pursuit controller wants).
The controller picks the nearest point ahead and tracks the line in the recorded
DIRECTION, so a goal on the far side of a loop is reached by driving the proper lap
(not nav2's shortest / wrong-way path), and the car stays on the vetted safe line.

This BYPASSES nav2 planning entirely for lap following — no /goal_pose needed; the
car starts tracking as soon as this publishes. Stop it (Ctrl+C) to hand control back;
the controller gets an empty path from rr_costmap_reset on the next /initialpose, or
you can send a normal /goal_pose again.

Usage:  python3 rr_play_path.py  [input.csv] [--loop] [--yaw-from-path]
        default input: ~/rr_maps/racing_line.csv
  --loop           append the first point after the last (continuous lap)
Notes:  heading in each PoseStamped is derived from the segment direction.
"""
import sys, os, math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Path
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


class Player(Node):
    def __init__(self, pts):
        super().__init__('rr_play_path')
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(Path, '/control/plan', qos)
        self.msg = self.build(pts)
        self.pub.publish(self.msg)                 # latched
        self.create_timer(1.0, lambda: self.pub.publish(self.msg))  # refresh
        self.get_logger().info('rr_play_path: publishing %d-point racing line on /control/plan' % len(pts))

    def build(self, pts):
        p = Path()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        for i, (x, y) in enumerate(pts):
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x = x
            ps.pose.position.y = y
            nx, ny = pts[min(i + 1, len(pts) - 1)]
            yaw = math.atan2(ny - y, nx - x)
            ps.pose.orientation.z = math.sin(yaw / 2.0)
            ps.pose.orientation.w = math.cos(yaw / 2.0)
            p.poses.append(ps)
        return p


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    loop = '--loop' in sys.argv
    path = args[0] if args else os.path.expanduser('~/rr_maps/racing_line.csv')
    pts = load_csv(path)
    if loop and pts:
        pts = pts + [pts[0]]
    if not pts:
        print('no points in %s' % path); return
    rclpy.init()
    try:
        rclpy.spin(Player(pts))
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
