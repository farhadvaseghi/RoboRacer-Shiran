#!/usr/bin/env python3
"""sim_test_drive — constant forward command on /drive_nav for AEB testing.

Stands in for the navigation controller in the isolated AEB sim test: publishes a
steady forward AckermannDrive on /drive_nav so the ego drives, letting us watch
emergency_brake force it to zero when a person enters the danger corridor.
Not used on hardware (the real controller publishes /drive_nav there).
"""

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped


class SimTestDrive(Node):
    def __init__(self):
        super().__init__('sim_test_drive')
        self.declare_parameter('drive_topic', '/drive_nav')
        self.declare_parameter('speed', 1.5)
        self.declare_parameter('steering', 0.0)
        self.declare_parameter('rate', 20.0)
        self._pub = self.create_publisher(
            AckermannDriveStamped, self.get_parameter('drive_topic').value, 10)
        self.create_timer(1.0 / self.get_parameter('rate').value, self._tick)

    def _tick(self):
        m = AckermannDriveStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        m.drive.speed = float(self.get_parameter('speed').value)
        m.drive.steering_angle = float(self.get_parameter('steering').value)
        self._pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = SimTestDrive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
