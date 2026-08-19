"""
ekf_validator_node.py — EKF output validator for sim integration (Phase 4).

Subscribes to both /odometry/filtered (EKF output) and /ego_racecar/odom
(gym ground truth) and computes position, yaw, and velocity errors.

Prints a rolling statistics report every `report_interval` seconds and emits
WARN-level log lines when any error exceeds its configured threshold.

Run in sim alongside estimation.launch.py:
    ros2 run roboracer_estimation ekf_validator_node
"""

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def _yaw_from_quat(q) -> float:
    """Extract yaw from a geometry_msgs quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _angle_diff(a: float, b: float) -> float:
    """Signed angular difference a − b, wrapped to (−π, π]."""
    d = a - b
    while d > math.pi:
        d -= 2.0 * math.pi
    while d <= -math.pi:
        d += 2.0 * math.pi
    return d


class EkfValidatorNode(Node):
    def __init__(self):
        super().__init__('ekf_validator_node')

        self.declare_parameter('report_interval', 5.0)        # seconds
        self.declare_parameter('max_position_error', 0.20)    # metres
        self.declare_parameter('max_yaw_error', 0.10)         # radians
        self.declare_parameter('max_velocity_error', 0.30)    # m/s

        self._report_interval = self.get_parameter('report_interval').get_parameter_value().double_value
        self._max_pos_err = self.get_parameter('max_position_error').get_parameter_value().double_value
        self._max_yaw_err = self.get_parameter('max_yaw_error').get_parameter_value().double_value
        self._max_vel_err = self.get_parameter('max_velocity_error').get_parameter_value().double_value

        # Latest messages
        self._filtered: Odometry | None = None
        self._ground_truth: Odometry | None = None

        # Statistics accumulators
        self._reset_stats()

        self._sub_filtered = self.create_subscription(
            Odometry, '/odometry/filtered', self._cb_filtered, 10)
        self._sub_gt = self.create_subscription(
            Odometry, '/ego_racecar/odom', self._cb_ground_truth, 10)

        self._report_timer = self.create_timer(self._report_interval, self._report)
        self.get_logger().info(
            f'EkfValidatorNode started — reporting every {self._report_interval:.0f}s')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _cb_filtered(self, msg: Odometry) -> None:
        self._filtered = msg
        self._try_compare()

    def _cb_ground_truth(self, msg: Odometry) -> None:
        self._ground_truth = msg
        self._try_compare()

    def _try_compare(self) -> None:
        if self._filtered is None or self._ground_truth is None:
            return

        f = self._filtered
        g = self._ground_truth

        pos_err = math.hypot(
            f.pose.pose.position.x - g.pose.pose.position.x,
            f.pose.pose.position.y - g.pose.pose.position.y,
        )
        yaw_err = abs(_angle_diff(
            _yaw_from_quat(f.pose.pose.orientation),
            _yaw_from_quat(g.pose.pose.orientation),
        ))
        vel_err = abs(f.twist.twist.linear.x - g.twist.twist.linear.x)

        # Accumulate
        self._n += 1
        self._sum_pos += pos_err
        self._sum_yaw += yaw_err
        self._sum_vel += vel_err
        self._max_pos = max(self._max_pos, pos_err)
        self._max_yaw = max(self._max_yaw, yaw_err)
        self._max_vel = max(self._max_vel, vel_err)

        # Immediate warn if thresholds exceeded
        if pos_err > self._max_pos_err:
            self.get_logger().warn(
                f'Position error {pos_err:.3f} m exceeds threshold {self._max_pos_err:.3f} m')
        if yaw_err > self._max_yaw_err:
            self.get_logger().warn(
                f'Yaw error {math.degrees(yaw_err):.1f}° exceeds threshold '
                f'{math.degrees(self._max_yaw_err):.1f}°')
        if vel_err > self._max_vel_err:
            self.get_logger().warn(
                f'Velocity error {vel_err:.3f} m/s exceeds threshold {self._max_vel_err:.3f} m/s')

    # ------------------------------------------------------------------
    # Periodic report
    # ------------------------------------------------------------------

    def _report(self) -> None:
        if self._filtered is None:
            self.get_logger().warn('No /odometry/filtered messages received yet')
            return
        if self._ground_truth is None:
            self.get_logger().warn('No /ego_racecar/odom messages received yet')
            return
        if self._n == 0:
            return

        mean_pos = self._sum_pos / self._n
        mean_yaw = self._sum_yaw / self._n
        mean_vel = self._sum_vel / self._n

        self.get_logger().info(
            f'\n--- EKF validation report ({self._n} samples) ---\n'
            f'  position : mean={mean_pos:.4f} m   max={self._max_pos:.4f} m   '
            f'limit={self._max_pos_err:.3f} m\n'
            f'  yaw      : mean={math.degrees(mean_yaw):.2f}°  '
            f'max={math.degrees(self._max_yaw):.2f}°  '
            f'limit={math.degrees(self._max_yaw_err):.2f}°\n'
            f'  velocity : mean={mean_vel:.4f} m/s  max={self._max_vel:.4f} m/s  '
            f'limit={self._max_vel_err:.3f} m/s\n'
            f'-----------------------------------------------'
        )
        self._reset_stats()

    def _reset_stats(self) -> None:
        self._n = 0
        self._sum_pos = 0.0
        self._sum_yaw = 0.0
        self._sum_vel = 0.0
        self._max_pos = 0.0
        self._max_yaw = 0.0
        self._max_vel = 0.0


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = EkfValidatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
