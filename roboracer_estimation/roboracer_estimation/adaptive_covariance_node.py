"""
Adaptive Covariance Pre-Processor for the RoboRacer estimation module.

Subscribes to raw sensor topics (real hardware) or gym ground-truth odom
(simulation) and republishes each message with dynamically adjusted covariance
matrices before they are fed into the EKF.

Adaptive rules (real mode only):
  - |ω| > omega_threshold  →  inflate VESC odom covariance  (wheel slip)
  - |ay| > ay_threshold    →  inflate ZED visual odom covariance (motion blur)

Sim mode passes gym odom through with nominal covariances; the adaptive code
path is still exercised so the integration is tested end-to-end.
"""

import copy

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


# ---------------------------------------------------------------------------
# Nominal covariance matrices (row-major 6×6 for Odometry messages).
# Large values (1e6) on unused dimensions prevent the EKF from fusing them.
# ---------------------------------------------------------------------------

_NOMINAL_POSE_COV = [
    0.05, 0.0,  0.0,  0.0,  0.0,  0.0,
    0.0,  0.05, 0.0,  0.0,  0.0,  0.0,
    0.0,  0.0,  1e6,  0.0,  0.0,  0.0,
    0.0,  0.0,  0.0,  1e6,  0.0,  0.0,
    0.0,  0.0,  0.0,  0.0,  1e6,  0.0,
    0.0,  0.0,  0.0,  0.0,  0.0,  0.05,
]

_NOMINAL_TWIST_COV = [
    0.01, 0.0,  0.0,  0.0,  0.0,  0.0,
    0.0,  1e6,  0.0,  0.0,  0.0,  0.0,
    0.0,  0.0,  1e6,  0.0,  0.0,  0.0,
    0.0,  0.0,  0.0,  1e6,  0.0,  0.0,
    0.0,  0.0,  0.0,  0.0,  1e6,  0.0,
    0.0,  0.0,  0.0,  0.0,  0.0,  0.01,
]

_LARGE_COV_THRESHOLD = 1e5  # diagonal values above this are masked; don't re-inflate them


class AdaptiveCovarianceNode(Node):
    def __init__(self):
        super().__init__('adaptive_covariance_node')

        self.declare_parameter('is_sim', True)
        self.declare_parameter('omega_threshold', 1.5)       # rad/s
        self.declare_parameter('ay_threshold', 3.0)          # m/s²
        self.declare_parameter('slip_inflation_factor', 10.0)
        self.declare_parameter('visual_inflation_factor', 5.0)

        self._is_sim = self.get_parameter('is_sim').get_parameter_value().bool_value
        self._omega_thr = self.get_parameter('omega_threshold').get_parameter_value().double_value
        self._ay_thr = self.get_parameter('ay_threshold').get_parameter_value().double_value
        self._slip_k = self.get_parameter('slip_inflation_factor').get_parameter_value().double_value
        self._visual_k = self.get_parameter('visual_inflation_factor').get_parameter_value().double_value

        # Latest IMU state cached for use in odom callbacks (real mode only).
        self._omega: float = 0.0   # angular velocity z  [rad/s]
        self._ay: float = 0.0      # lateral acceleration [m/s²]

        if self._is_sim:
            self._init_sim()
        else:
            self._init_real()

    # ------------------------------------------------------------------
    # Initialisation helpers
    # ------------------------------------------------------------------

    def _init_sim(self) -> None:
        self._sub_sim = self.create_subscription(
            Odometry, '/ego_racecar/odom', self._cb_sim_odom, 10)
        self._pub_sim = self.create_publisher(
            Odometry, '/estimation/sim_odom_adaptive', 10)
        self.get_logger().info(
            'AdaptiveCovarianceNode started in SIM mode — '
            'subscribing /ego_racecar/odom → /estimation/sim_odom_adaptive')

    def _init_real(self) -> None:
        # Subscriptions — raw sensor topics
        self._sub_imu = self.create_subscription(
            Imu, '/zed/zed_node/imu/data', self._cb_imu, 20)
        self._sub_zed = self.create_subscription(
            Odometry, '/zed/zed_node/odom', self._cb_zed_odom, 10)
        self._sub_vesc = self.create_subscription(
            Odometry, '/vesc/odom', self._cb_vesc_odom, 10)

        # Publishers — adaptive topics consumed by the EKF
        self._pub_imu = self.create_publisher(
            Imu, '/estimation/imu_adaptive', 20)
        self._pub_zed = self.create_publisher(
            Odometry, '/estimation/zed_odom_adaptive', 10)
        self._pub_vesc = self.create_publisher(
            Odometry, '/estimation/vesc_odom_adaptive', 10)
        self.get_logger().info(
            'AdaptiveCovarianceNode started in REAL mode — '
            'subscribing ZED IMU + odom, VESC odom')

    # ------------------------------------------------------------------
    # Sim callbacks
    # ------------------------------------------------------------------

    def _cb_sim_odom(self, msg: Odometry) -> None:
        out = copy.deepcopy(msg)
        _set_nominal_odom_covariance(out)
        self._pub_sim.publish(out)

    # ------------------------------------------------------------------
    # Real callbacks
    # ------------------------------------------------------------------

    def _cb_imu(self, msg: Imu) -> None:
        self._omega = msg.angular_velocity.z
        self._ay = msg.linear_acceleration.y
        # ZED already sets covariance fields; republish without modification.
        self._pub_imu.publish(msg)

    def _cb_zed_odom(self, msg: Odometry) -> None:
        out = copy.deepcopy(msg)
        if abs(self._ay) > self._ay_thr:
            _inflate_pose_covariance(out, self._visual_k)
            _inflate_twist_covariance(out, self._visual_k)
            self.get_logger().debug(
                f'ZED odom covariance inflated ×{self._visual_k:.1f} '
                f'(ay={self._ay:.2f} m/s²)')
        self._pub_zed.publish(out)

    def _cb_vesc_odom(self, msg: Odometry) -> None:
        out = copy.deepcopy(msg)
        if abs(self._omega) > self._omega_thr:
            _inflate_twist_covariance(out, self._slip_k)
            self.get_logger().debug(
                f'VESC odom covariance inflated ×{self._slip_k:.1f} '
                f'(ω={self._omega:.2f} rad/s)')
        self._pub_vesc.publish(out)


# ---------------------------------------------------------------------------
# Covariance helpers — operate on message fields in-place
# ---------------------------------------------------------------------------

def _set_nominal_odom_covariance(msg: Odometry) -> None:
    msg.pose.covariance = list(_NOMINAL_POSE_COV)
    msg.twist.covariance = list(_NOMINAL_TWIST_COV)


def _inflate_pose_covariance(msg: Odometry, factor: float) -> None:
    cov = list(msg.pose.covariance)
    for i in range(36):
        if cov[i] < _LARGE_COV_THRESHOLD:
            cov[i] *= factor
    msg.pose.covariance = cov


def _inflate_twist_covariance(msg: Odometry, factor: float) -> None:
    cov = list(msg.twist.covariance)
    for i in range(36):
        if cov[i] < _LARGE_COV_THRESHOLD:
            cov[i] *= factor
    msg.twist.covariance = cov


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = AdaptiveCovarianceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
