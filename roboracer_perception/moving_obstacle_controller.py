#!/usr/bin/env python3
"""Drive the second simulator agent on the oval track.

The opponent follows the baseline oval in moving-only scenarios and applies a
smooth lateral bypass around the two fixed static obstacles when the combined
static-plus-dynamic map is used.
"""

import math

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


WHEELBASE = 0.3302
STRAIGHT_LOOKAHEAD = 1.35
TURN_LOOKAHEAD = 1.0
STEER_LIMIT = 0.30
CONTROL_DT = 0.05
CRUISE_SPEED = 0.9
TURN_SPEED = 0.75
BYPASS_SPEED = 0.78
STRAIGHT_CLEARANCE_X = 2.8
BOTTOM_STRAIGHT_Y = 0.0
TOP_STRAIGHT_Y = 5.0
BOTTOM_BYPASS_SHIFT = -0.72
TOP_BYPASS_SHIFT = 0.72

STATIC_OBSTACLES = (
    {'center': (6.5, 0.55), 'shift_y': BOTTOM_BYPASS_SHIFT},
    {'center': (13.5, 4.45), 'shift_y': TOP_BYPASS_SHIFT},
)


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def bypass_profile(x: float) -> tuple[float, float]:
    """Return a smooth lateral offset and activity level near static obstacles."""
    offset = 0.0
    activity = 0.0
    for obstacle in STATIC_OBSTACLES:
        center_x, _ = obstacle['center']
        dx = abs(x - center_x)
        if dx >= STRAIGHT_CLEARANCE_X:
            continue
        weight = 0.5 * (1.0 + math.cos(math.pi * dx / STRAIGHT_CLEARANCE_X))
        offset += obstacle['shift_y'] * weight
        activity = max(activity, weight)
    return offset, activity


class MovingObstacleController(Node):
    def __init__(self):
        super().__init__('moving_obstacle_controller')
        self._drive_pub = self.create_publisher(AckermannDriveStamped, '/opp_drive', 10)
        self._odom_sub = self.create_subscription(
            Odometry, '/opp_racecar/odom', self._odom_callback, 10
        )
        self._pose = None
        self._timer = self.create_timer(CONTROL_DT, self._timer_callback)

    def _odom_callback(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self._pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            normalize_angle(yaw),
        )

    def _timer_callback(self) -> None:
        if self._pose is None:
            self._publish(0.0, 0.0)
            return

        x, y, yaw = self._pose
        speed, steer = self._oval_follow_command(x, y, yaw)
        self._publish(speed, steer)

    def _publish(self, speed: float, steer: float) -> None:
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(max(-STEER_LIMIT, min(STEER_LIMIT, steer)))
        self._drive_pub.publish(msg)

    def _oval_follow_command(self, x: float, y: float, yaw: float) -> tuple[float, float]:
        if x < 20.0 and y < 2.5:
            tx = min(20.0, x + STRAIGHT_LOOKAHEAD)
            offset, activity = bypass_profile(tx)
            ty = BOTTOM_STRAIGHT_Y + offset
            target_speed = CRUISE_SPEED - activity * (CRUISE_SPEED - BYPASS_SPEED)
        elif x >= 20.0 and y <= 5.0:
            cx, cy, radius = 20.0, 2.5, 2.5
            ang = math.atan2(y - cy, x - cx)
            ang = max(-math.pi / 2.0, min(math.pi / 2.0, ang + TURN_LOOKAHEAD / radius))
            tx = cx + radius * math.cos(ang)
            ty = cy + radius * math.sin(ang)
            target_speed = TURN_SPEED
        elif x > 0.0 and y >= 2.5:
            tx = max(0.0, x - STRAIGHT_LOOKAHEAD)
            offset, activity = bypass_profile(tx)
            ty = TOP_STRAIGHT_Y + offset
            target_speed = CRUISE_SPEED - activity * (CRUISE_SPEED - BYPASS_SPEED)
        else:
            cx, cy, radius = 0.0, 2.5, 2.5
            ang = math.atan2(y - cy, x - cx)
            if ang < 0.0:
                ang += 2.0 * math.pi
            ang = max(math.pi / 2.0, min(3.0 * math.pi / 2.0, ang + TURN_LOOKAHEAD / radius))
            tx = cx + radius * math.cos(ang)
            ty = cy + radius * math.sin(ang)
            target_speed = TURN_SPEED

        dx = tx - x
        dy = ty - y
        local_x = math.cos(-yaw) * dx - math.sin(-yaw) * dy
        local_y = math.sin(-yaw) * dx + math.cos(-yaw) * dy

        if local_x <= 1e-6:
            return target_speed, 0.0

        lookahead = STRAIGHT_LOOKAHEAD if target_speed > TURN_SPEED else TURN_LOOKAHEAD
        curvature = 2.0 * local_y / (lookahead ** 2)
        steer = math.atan(WHEELBASE * curvature)
        return target_speed, steer


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MovingObstacleController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
