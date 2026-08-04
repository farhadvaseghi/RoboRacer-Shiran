#!/usr/bin/env python3
"""Sample the pose chain to see whether slam is correcting or the pose is
pure dead reckoning.

map->odom  changes ONLY when slam scan-matches. If it is frozen while the car
drives, slam is not correcting and the pose is wheel odometry alone.
odom->base_link is the raw wheel odometry.
"""

import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


def yaw_of(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class Probe(Node):
    def __init__(self, seconds):
        super().__init__('rr_pose_probe')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.odom = None
        self.odom_msgs = 0
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.mapodom_msgs = 0
        self.create_subscription(
            Odometry, '/odometry/map', self.on_mapodom, 10
        )
        self.seconds = seconds

    def on_odom(self, msg):
        self.odom = msg
        self.odom_msgs += 1

    def on_mapodom(self, _msg):
        self.mapodom_msgs += 1

    def sample(self, label):
        out = [label]
        for parent, child in (('map', 'odom'), ('odom', 'base_link'),
                              ('map', 'base_link')):
            try:
                t = self.buffer.lookup_transform(
                    parent, child, rclpy.time.Time()
                ).transform
                out.append(
                    f'  {parent}->{child}: x={t.translation.x:+.3f} '
                    f'y={t.translation.y:+.3f} yaw={yaw_of(t.rotation):+.3f}'
                )
            except Exception as exc:  # noqa: BLE001 - report, don't crash
                out.append(f'  {parent}->{child}: UNAVAILABLE ({type(exc).__name__})')

        if self.odom is not None:
            v = self.odom.twist.twist.linear.x
            p = self.odom.pose.pose.position
            out.append(f'  /odom: x={p.x:+.3f} y={p.y:+.3f} v={v:+.3f} m/s')
        else:
            out.append('  /odom: NO MESSAGES RECEIVED')
        print('\n'.join(out), flush=True)


def main():
    rclpy.init()
    probe = Probe(seconds=10)
    end = time.time() + 3.0
    while time.time() < end:
        rclpy.spin_once(probe, timeout_sec=0.1)

    probe.sample('=== SAMPLE 1 ===')
    end = time.time() + probe.seconds
    while time.time() < end:
        rclpy.spin_once(probe, timeout_sec=0.1)
    probe.sample(f'=== SAMPLE 2 (after {probe.seconds}s) ===')

    print(
        f'\nrates over {probe.seconds}s: /odom {probe.odom_msgs} msgs, '
        f'/odometry/map {probe.mapodom_msgs} msgs',
        flush=True,
    )
    probe.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
