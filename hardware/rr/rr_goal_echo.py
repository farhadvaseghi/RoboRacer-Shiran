#!/usr/bin/env python3
"""Read-only: print every /goal_pose with its yaw decoded to degrees.

'ros2 topic echo /goal_pose' prints a raw quaternion, which is not something
you can sanity-check by eye against the mission's waypoint table. This prints
x, y, yaw in degrees and the quaternion norm, in the same format as the
waypoint listing, so a goal picked in Foxglove can be compared directly.

The norm column matters: a quaternion pasted with a truncated w is still
accepted by Nav2 and silently means a different heading. Anything that is not
1.000000 is a bad goal.

Usage:  python3 rr_goal_echo.py [topic]     (default /goal_pose)
"""

import math
import os
import sys
import time

os.environ.setdefault('ROS_DOMAIN_ID', '7')

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

TOPIC = sys.argv[1] if len(sys.argv) > 1 else '/goal_pose'


class GoalEcho(Node):
    def __init__(self):
        super().__init__('rr_goal_echo')

        # Match what the publishers offer. foxglove_bridge publishes
        # TRANSIENT_LOCAL, so a TRANSIENT_LOCAL subscriber also replays the
        # LAST goal already sent -- useful, you see the current goal
        # immediately instead of waiting for the next one. But a
        # TRANSIENT_LOCAL subscriber does NOT match a VOLATILE publisher at
        # all, so if anything volatile (rviz, a script) is also publishing,
        # drop to VOLATILE, which matches everything and only costs the replay.
        info = []
        for _ in range(25):
            info = self.get_publishers_info_by_topic(TOPIC)
            if info:
                break
            time.sleep(0.2)

        durability = QoSDurabilityPolicy.VOLATILE
        if info and all(
            p.qos_profile.durability == QoSDurabilityPolicy.TRANSIENT_LOCAL
            for p in info
        ):
            durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(
            PoseStamped, TOPIC, self._on_goal,
            QoSProfile(
                depth=10,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=durability,
                history=QoSHistoryPolicy.KEEP_LAST,
            ),
        )

        print('listening on %s -- %d publisher(s), subscribing %s'
              % (TOPIC, len(info),
                 'TRANSIENT_LOCAL (replays the last goal)'
                 if durability == QoSDurabilityPolicy.TRANSIENT_LOCAL
                 else 'VOLATILE (new goals only)'))
        print()
        print('%-8s %9s %9s %10s %10s  %s'
              % ('#', 'x', 'y', 'yaw_deg', '|q|', 'frame'))
        self.count = 0

    def _on_goal(self, msg):
        self.count += 1
        q = msg.pose.orientation
        yaw = math.degrees(math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * q.z * q.z))
        norm = math.sqrt(q.x ** 2 + q.y ** 2 + q.z ** 2 + q.w ** 2)
        flag = '' if abs(norm - 1.0) < 1e-6 else '   <-- NOT NORMALISED'
        print('%-8d %9.3f %9.3f %10.2f %10.6f  %s%s'
              % (self.count, msg.pose.position.x, msg.pose.position.y,
                 yaw, norm, msg.header.frame_id, flag))
        print('         quaternion  z=%.17g  w=%.17g' % (q.z, q.w))


def main():
    rclpy.init()
    node = GoalEcho()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl+C or a SIGTERM from `timeout` -- both are normal ways to stop
        # a watcher, neither deserves a traceback.
        print('\n%d goal(s) seen' % node.count)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
