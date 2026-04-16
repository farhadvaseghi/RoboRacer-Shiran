#!/usr/bin/env python3
"""Convert Nav2 Twist commands → AckermannDriveStamped for the f1tenth simulator.

Subscribes:  /cmd_vel  (geometry_msgs/Twist)
Publishes:   /nav      (ackermann_msgs/AckermannDriveStamped)

Nav2's RegulatedPurePursuitController outputs linear.x (m/s) and
angular.z (rad/s).  We recover the steering angle using the bicycle model:

    steering_angle = atan(angular_z * wheelbase / linear_x)

When the car is stopped (|linear_x| < threshold) we hold steering at zero
to avoid a division-by-zero.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped

WHEELBASE = 0.3302   # metres — must match params.yaml


class TwistToAckermann(Node):
    def __init__(self):
        super().__init__('twist_to_ackermann')
        self._pub = self.create_publisher(
            AckermannDriveStamped, '/nav', 10)
        self._sub = self.create_subscription(
            Twist, '/cmd_vel', self._cb, 10)
        self.get_logger().info('twist_to_ackermann ready.')

    def _cb(self, msg: Twist):
        speed = msg.linear.x
        if abs(speed) > 0.001:
            steering_angle = math.atan(msg.angular.z * WHEELBASE / speed)
        else:
            steering_angle = 0.0

        out = AckermannDriveStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        out.drive.speed = float(speed)
        out.drive.steering_angle = float(steering_angle)
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TwistToAckermann()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
