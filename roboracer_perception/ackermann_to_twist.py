#!/usr/bin/env python3
"""Convert AckermannDriveStamped → Twist for Gazebo differential drive.

Subscribes:  /ego_racecar/drive  (ackermann_msgs/AckermannDriveStamped)
Publishes:   /cmd_vel            (geometry_msgs/Twist)

The differential drive plugin in Gazebo takes linear.x (m/s) and
angular.z (rad/s).  We derive angular.z from the Ackermann steering
angle using the bicycle model:

    angular_z = speed * tan(steering_angle) / wheelbase
"""

import math

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist

WHEELBASE = 0.324   # metres (front axle to rear axle)


class AckermannToTwist(Node):
    def __init__(self):
        super().__init__('ackermann_to_twist')
        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._sub = self.create_subscription(
            AckermannDriveStamped,
            '/ego_racecar/drive',
            self._cb, 10)
        self.get_logger().info('ackermann_to_twist ready.')

    def _cb(self, msg: AckermannDriveStamped):
        speed = msg.drive.speed
        steer = msg.drive.steering_angle
        twist = Twist()
        twist.linear.x  = float(speed)
        twist.angular.z = float(speed * math.tan(steer) / WHEELBASE)
        self._pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = AckermannToTwist()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
