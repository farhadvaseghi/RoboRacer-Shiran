#!/usr/bin/env python3
"""rr_mark_pose.py -- capture the car's current pose as a mission waypoint.

Drive the car by hand to a place that matters -- the point where it should
start a turn, the apex, wherever -- and press Enter. The pose is read from the
live map -> base_link transform and printed in the exact tuple format that
rr_waypoint_mission.py's WAYPOINTS uses, so it can be pasted straight in.

    ROS_DOMAIN_ID=7 python3 ~/rr/rr_mark_pose.py

Every mark is also appended to ~/rr_maps/marked_poses.txt with a timestamp, so
nothing is lost if the terminal scrolls away or the Wi-Fi drops.

The pose recorded is where the car ACTUALLY IS according to slam, so localize
first (the car must be tracking properly) or the marks inherit the error. The
alignment percentage from rr_scan_align.py is a good pre-check.
"""

import math
import os
import sys
from datetime import datetime

os.environ.setdefault('ROS_DOMAIN_ID', '7')

import rclpy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

MARKS_FILE = os.path.expanduser('~/rr_maps/marked_poses.txt')
TF_WAIT_SECONDS = 10.0


def spin_for(node, seconds):
    end = node.get_clock().now().nanoseconds + seconds * 1e9
    while rclpy.ok() and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def main():
    rclpy.init()
    node = rclpy.create_node('rr_mark_pose')
    buf = Buffer()
    TransformListener(buf, node)

    print('waiting for map -> base_link ...')
    end = node.get_clock().now().nanoseconds + TF_WAIT_SECONDS * 1e9
    while rclpy.ok() and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        if buf.can_transform('map', 'base_link', Time()):
            break
    else:
        print('NO map -> base_link transform.')
        print('  slam has no pose: park on the origin and run '
              'python3 ~/rr/rr_seed_start.py')
        print('  no odom at all:   bash ~/rr/kill_base.sh && ~/rr/rr_bringup.sh')
        return 1

    os.makedirs(os.path.dirname(MARKS_FILE), exist_ok=True)
    print('')
    print('Drive the car to a point, then press Enter to mark it.')
    print('Type q then Enter to finish. Marks are appended to %s' % MARKS_FILE)
    print('')

    marks = []
    while True:
        try:
            answer = input('[Enter]=mark  [q]=quit > ').strip().lower()
        except (EOFError, KeyboardInterrupt):
            break
        if answer in ('q', 'quit', 'exit'):
            break

        # Freshen the buffer right before reading, so the mark is current.
        spin_for(node, 0.3)
        try:
            tf = buf.lookup_transform('map', 'base_link', Time())
        except Exception as exc:  # noqa: BLE001 - report and keep going
            print('  lookup failed (%s); is the car still localized?'
                  % type(exc).__name__)
            continue

        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                      1 - 2 * (q.y ** 2 + q.z ** 2)))
        marks.append((t.x, t.y, q.z, q.w, yaw))

        name = 'Goal %d' % len(marks)
        block = (
            "    (\n"
            "        '%s',\n"
            "        %.17g,\n"
            "        %.17g,\n"
            "        %.17g,\n"
            "        %.17g,\n"
            "    ),"
        ) % (name, t.x, t.y, q.z, q.w)

        print('  marked %d: x=%.3f y=%.3f yaw=%.1f deg' % (len(marks), t.x, t.y, yaw))
        print(block)
        with open(MARKS_FILE, 'a') as handle:
            handle.write('# %s  mark %d  x=%.3f y=%.3f yaw=%.1f deg\n'
                         % (datetime.now().isoformat(timespec='seconds'),
                            len(marks), t.x, t.y, yaw))
            handle.write(block + '\n')

    if marks:
        print('')
        print('=' * 62)
        print('Paste into WAYPOINTS in ~/rr/rr_waypoint_mission.py:')
        print('=' * 62)
        print('WAYPOINTS = (')
        for index, (x, y, qz, qw, _yaw) in enumerate(marks, start=1):
            print("    (\n        'Goal %d',\n        %.17g,\n        %.17g,\n"
                  "        %.17g,\n        %.17g,\n    )," % (index, x, y, qz, qw))
        print(')')
        print('')
        print('Also saved to %s' % MARKS_FILE)
    else:
        print('no marks taken')

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
