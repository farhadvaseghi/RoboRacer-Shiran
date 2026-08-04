#!/usr/bin/env python3
"""rr_record_path.py — record a racing line by driving it, flushing to disk LIVE.

Samples the car's map-frame pose (/odometry/map) every `spacing` metres and
APPENDS each new point to the CSV immediately (flush + fsync), so a Wi-Fi drop,
brownout, or reboot can never lose more than the last sample. Ctrl+C to stop.
Drive the lap with the joystick (LB deadman) OR via Foxglove goals; either way
the driven /odometry/map is captured. Play it back with rr_play_path.py
(add --loop for continuous autonomous laps).

Usage:  python3 rr_record_path.py [output.csv] [spacing_m]
        default: ~/rr_maps/racing_line.csv , spacing 0.15 m
"""
import sys, os, math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class Recorder(Node):
    def __init__(self, path, spacing):
        super().__init__('rr_record_path')
        self.path = path
        self.spacing = spacing
        self.last = None
        self.n = 0
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        self.f = open(self.path, 'w')
        self.f.write('x,y\n')
        self._sync()
        self.create_subscription(Odometry, '/odometry/map', self.cb, 10)
        self.get_logger().info(
            'rr_record_path: drive the lap now. Writing LIVE -> %s (spacing %.2f m). Ctrl+C to stop.'
            % (self.path, self.spacing))

    def _sync(self):
        self.f.flush()
        os.fsync(self.f.fileno())

    def cb(self, m):
        x = m.pose.pose.position.x
        y = m.pose.pose.position.y
        if self.last is None or math.hypot(x - self.last[0], y - self.last[1]) >= self.spacing:
            self.last = (x, y)
            self.n += 1
            self.f.write('%.4f,%.4f\n' % (x, y))
            self._sync()
            if self.n % 20 == 0:
                self.get_logger().info('recorded %d points' % self.n)

    def close(self):
        try:
            self._sync()
            self.f.close()
        except Exception:
            pass
        self.get_logger().info('STOPPED: %d points -> %s' % (self.n, self.path))


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser('~/rr_maps/racing_line.csv')
    spacing = float(sys.argv[2]) if len(sys.argv) > 2 else 0.15
    rclpy.init()
    n = Recorder(path, spacing)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
