import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped

# F1TENTH wheelbase (rear axle to front axle), metres
WHEELBASE = 0.3302


class CmdVelToAckermann(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_ackermann')
        self.sub = self.create_subscription(Twist, '/cmd_vel', self._cb, 10)
        self.pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

    def _cb(self, msg: Twist):
        out = AckermannDriveStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'ego_racecar/base_link'
        out.drive.speed = msg.linear.x
        if abs(msg.linear.x) > 0.001:
            out.drive.steering_angle = math.atan2(
                WHEELBASE * msg.angular.z, abs(msg.linear.x)
            )
        else:
            out.drive.steering_angle = 0.0
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToAckermann()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
