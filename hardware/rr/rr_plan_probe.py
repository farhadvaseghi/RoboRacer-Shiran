#!/usr/bin/env python3
"""Read-only: grab the latched /plan and locate its sharpest heading changes.

Prints where the path turns hardest and how far each of those points is from
the mission waypoints, so a 'weird move' seen in Foxglove can be attributed to
a specific waypoint instead of estimated off the screen.
"""

import math
import os
import sys
import time

os.environ.setdefault('ROS_DOMAIN_ID', '7')

import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

WAYPOINTS = (
    ('start', 0.0, 0.0),
    ('Goal 1', 8.3917886242894575, 0.96402365578670501),
    ('Goal 2', 11.84582138568256, 5.3344034612391624),
    ('Goal 3', 0.5100927104261217, 6.4594229731848376),
)

TOPIC = sys.argv[1] if len(sys.argv) > 1 else '/plan'


class PlanProbe(Node):
    def __init__(self):
        super().__init__('rr_plan_probe')
        self.path = None
        # Durability has to MATCH what is out there, and the two plan topics
        # differ: /plan is VOLATILE (planner_server publishes it only while a
        # goal is being planned) and /control/plan is TRANSIENT_LOCAL
        # (plan_qos_relay latches it for the controller). Getting this wrong
        # fails in two different silent ways -- a TRANSIENT_LOCAL sub does not
        # match a VOLATILE pub at all, and a VOLATILE sub matches a
        # TRANSIENT_LOCAL pub but never receives the already-latched sample.
        # So ask the graph what the publishers offer and mirror it.
        # Discovery is not instant -- asking the graph in the constructor
        # reports zero publishers on a healthy topic and silently picks the
        # wrong durability. Poll until somebody shows up.
        self.durability = QoSDurabilityPolicy.VOLATILE
        info = []
        for _ in range(25):
            info = self.get_publishers_info_by_topic(TOPIC)
            if info:
                break
            time.sleep(0.2)
        if any(p.qos_profile.durability == QoSDurabilityPolicy.TRANSIENT_LOCAL
               for p in info):
            self.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.publisher_count = len(info)

        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=self.durability,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(Path, TOPIC, self._on_path, qos)

    def _on_path(self, msg):
        self.path = msg


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def main():
    rclpy.init()
    node = PlanProbe()
    print('listening on %s (%d publisher(s), subscribing %s)'
          % (TOPIC, node.publisher_count,
             'TRANSIENT_LOCAL' if node.durability
             == QoSDurabilityPolicy.TRANSIENT_LOCAL else 'VOLATILE'))

    deadline = node.get_clock().now().nanoseconds * 1e-9 + 8.0
    while node.path is None:
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.get_clock().now().nanoseconds * 1e-9 > deadline:
            if node.publisher_count == 0:
                print('Nothing publishes %s -- is nav2 up?' % TOPIC)
            elif TOPIC == '/plan':
                print('No %s in 8 s. It is VOLATILE: planner_server publishes '
                      'it only while a goal is being planned, so run this '
                      'DURING a route, or read the latched /control/plan '
                      'instead:\n    python3 %s /control/plan'
                      % (TOPIC, sys.argv[0]))
            else:
                print('No %s in 8 s (nothing latched yet -- has a route run '
                      'since the last restart?)' % TOPIC)
            node.destroy_node()
            rclpy.shutdown()
            return 1

    poses = node.path.poses
    print('%s: %d poses, frame %s'
          % (TOPIC, len(poses), node.path.header.frame_id))
    if not poses:
        # Not a fault: rr_costmap_reset publishes an EMPTY path (8x over 2 s)
        # to make the controller drop a finished or stale route, so between
        # runs this is exactly what is latched.
        print('The latched plan is EMPTY -- no route is loaded. That is the '
              'idle state (rr_costmap_reset clears the plan after a goal '
              'ends). Run this again WHILE a route is driving.')
        node.destroy_node()
        rclpy.shutdown()
        return 1
    if len(poses) < 3:
        print('only %d pose(s) -- too short to measure curvature' % len(poses))
        node.destroy_node()
        rclpy.shutdown()
        return 1

    pts = [(p.pose.position.x, p.pose.position.y) for p in poses]
    yaws = [yaw_of(p.pose.orientation) for p in poses]

    # Direction of travel between consecutive points, and how much it swings.
    turns = []
    for i in range(1, len(pts) - 1):
        dx0, dy0 = pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]
        dx1, dy1 = pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]
        if math.hypot(dx0, dy0) < 1e-6 or math.hypot(dx1, dy1) < 1e-6:
            continue
        swing = wrap(math.atan2(dy1, dx1) - math.atan2(dy0, dx0))
        turns.append((abs(swing), i, swing))

    total_len = sum(math.hypot(pts[i + 1][0] - pts[i][0],
                               pts[i + 1][1] - pts[i][1])
                    for i in range(len(pts) - 1))
    print('path length %.2f m' % total_len)

    # A cusp/loop shows up as a cluster of large swings; report the worst few,
    # spaced out so one corner does not fill the whole list.
    turns.sort(reverse=True)
    chosen = []
    for mag, i, swing in turns:
        if all(abs(i - j) > 8 for _, j, _ in chosen):
            chosen.append((mag, i, swing))
        if len(chosen) == 6:
            break

    print('\nsharpest direction changes (between consecutive plan points):')
    for mag, i, swing in chosen:
        x, y = pts[i]
        near = min(WAYPOINTS,
                   key=lambda w: math.hypot(x - w[1], y - w[2]))
        d = math.hypot(x - near[1], y - near[2])
        print('  idx %4d  (%.2f, %.2f)  swing %+7.1f deg  pose_yaw %+7.1f deg'
              '   nearest: %s at %.2f m'
              % (i, x, y, math.degrees(swing), math.degrees(yaws[i]),
                 near[0], d))

    print('\npose yaw at the plan point closest to each waypoint:')
    for name, wx, wy in WAYPOINTS:
        best = min(range(len(pts)),
                   key=lambda i: math.hypot(pts[i][0] - wx, pts[i][1] - wy))
        d = math.hypot(pts[best][0] - wx, pts[best][1] - wy)
        print('  %-7s target (%.2f, %.2f)  plan idx %4d at %.2f m  '
              'yaw %+7.1f deg' % (name, wx, wy, best, d,
                                  math.degrees(yaws[best])))

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
