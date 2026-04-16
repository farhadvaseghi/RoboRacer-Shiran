#!/usr/bin/env python3
"""Drive command adapter: AckermannDriveStamped → AutoDRIVE Float32 commands.

AutoDRIVE expects two separate normalized Float32 topics:
  /autodrive/roboracer_1/throttle_command  [-1.0 … +1.0]  (speed)
  /autodrive/roboracer_1/steering_command  [-1.0 … +1.0]  (steering, +1=left)

Our teleop publishes:
  /ego_racecar/drive  AckermannDriveStamped
    drive.speed          m/s   (±SPEED_MAX)
    drive.steering_angle rad   (±STEER_MAX)

This node subscribes to /ego_racecar/drive and re-publishes the two
normalized Float32 commands that the AutoDRIVE devkit bridge expects.
"""

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32

SPEED_MAX = 4.0   # m/s  — must match teleop_key.py SPEED_MAX
STEER_MAX = 0.4   # rad  — must match teleop_key.py STEER_MAX


class AckermannToAutoDRIVE(Node):
    def __init__(self):
        super().__init__('ackermann_to_autodrive')
        self._sub = self.create_subscription(
            AckermannDriveStamped,
            '/ego_racecar/drive',
            self._callback,
            10,
        )
        self._pub_throttle = self.create_publisher(
            Float32, '/autodrive/roboracer_1/throttle_command', 10
        )
        self._pub_steering = self.create_publisher(
            Float32, '/autodrive/roboracer_1/steering_command', 10
        )
        self.get_logger().info('ackermann_to_autodrive ready.')

    def _callback(self, msg: AckermannDriveStamped) -> None:
        throttle = Float32()
        throttle.data = float(
            max(-1.0, min(1.0, msg.drive.speed / SPEED_MAX))
        )

        steering = Float32()
        # AutoDRIVE steering: +1 = left (same sign as ROS convention)
        steering.data = float(
            max(-1.0, min(1.0, msg.drive.steering_angle / STEER_MAX))
        )

        self._pub_throttle.publish(throttle)
        self._pub_steering.publish(steering)


def main(args=None):
    rclpy.init(args=args)
    node = AckermannToAutoDRIVE()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
