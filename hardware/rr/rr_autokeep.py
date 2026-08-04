#!/usr/bin/env python3
"""rr_autokeep.py - gated zero-speed /drive keepalive.

The VESC only emits /odom (odom->base_link TF) while a drive command is flowing,
so at rest we must publish a zero /drive to keep localization + the map-frame
odom bridge alive. But a plain keeper fights the controller (both publish /drive).

This version is GATED: it watches /drive and, whenever it sees a non-zero speed
command (the pure_pursuit controller driving), it stays SILENT for `hold` seconds.
So it only fills in the zeros at rest and never fights the controller.
"""
import time
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped


class AutoKeep(Node):
    def __init__(self):
        super().__init__('rr_autokeep')
        self.hold = 0.6                 # stay quiet this long after an external command
        self.thresh = 0.02              # |speed| above this = "someone else is driving"
        self.last_active = 0.0
        self.pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.create_subscription(AckermannDriveStamped, '/drive', self.on_drive, 10)
        self.create_timer(0.05, self.tick)   # 20 Hz
        self.get_logger().info('rr_autokeep: zero /drive at rest; pauses while controller drives')

    def on_drive(self, msg):
        if abs(msg.drive.speed) > self.thresh:
            self.last_active = time.monotonic()

    def tick(self):
        if time.monotonic() - self.last_active < self.hold:
            return                       # controller is driving -> stay silent
        m = AckermannDriveStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.drive.speed = 0.0
        m.drive.steering_angle = 0.0
        self.pub.publish(m)


def main():
    rclpy.init()
    try:
        rclpy.spin(AutoKeep())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
