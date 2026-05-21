#!/usr/bin/env python3
"""LiDAR cone detection node for RoboRacer.

Pipeline:
  /scan (LaserScan)
    → range/angle filter
    → Cartesian projection
    → DBSCAN clustering
    → algebraic circle fit per cluster
    → cone radius validation
    → /perception/cones        (roboracer_perception/ConeArray) ← for Estimation/Planning
    → /perception/cone_markers (visualization_msgs/MarkerArray) ← for RViz
"""

import math

import numpy as np
from sklearn.cluster import DBSCAN

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from roboracer_perception.msg import Cone, ConeArray


# ---------------------------------------------------------------------------
# Pure-function helpers (no ROS dependency — easy to unit-test)
# ---------------------------------------------------------------------------

def filter_scan(ranges, angle_min: float, angle_increment: float,
                range_min: float, range_max: float) -> np.ndarray:
    """Convert a LaserScan range array to a (N, 2) Cartesian point array.

    Drops any reading that is non-finite or outside [range_min, range_max].
    Points are expressed in the laser frame (x forward, y left).
    """
    pts = []
    for i, r in enumerate(ranges):
        if not math.isfinite(r) or r < range_min or r > range_max:
            continue
        angle = angle_min + i * angle_increment
        pts.append((r * math.cos(angle), r * math.sin(angle)))
    return np.array(pts, dtype=np.float64) if pts else np.empty((0, 2), dtype=np.float64)


def fit_circle(points: np.ndarray):
    """Algebraic (Kasa) least-squares circle fit.

    Solves  2·cx·x  +  2·cy·y  +  c  =  x² + y²  for (cx, cy, c), then
    recovers  radius = sqrt(c + cx² + cy²).

    Args:
        points: (N, 2) array, N ≥ 3.

    Returns:
        (cx, cy, radius) tuple, or None if the fit is degenerate.
    """
    if len(points) < 3:
        return None
    x, y = points[:, 0], points[:, 1]
    A = np.column_stack([2.0 * x, 2.0 * y, np.ones(len(x))])
    b = x ** 2 + y ** 2
    try:
        result, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
    except np.linalg.LinAlgError:
        return None
    cx, cy = result[0], result[1]
    r_sq = result[2] + cx ** 2 + cy ** 2
    if r_sq <= 0.0:
        return None
    return cx, cy, math.sqrt(r_sq)


def circle_fit_confidence(points: np.ndarray, cx: float, cy: float,
                          radius: float) -> float:
    """Return a confidence score in [0, 1] based on RMS residual of the fit.

    Maps 0 mm residual → 1.0 and ≥50 mm residual → 0.0, linearly.
    """
    distances = np.sqrt((points[:, 0] - cx) ** 2 + (points[:, 1] - cy) ** 2)
    rms = float(np.sqrt(np.mean((distances - radius) ** 2)))
    return max(0.0, 1.0 - rms / 0.05)


def cluster_and_fit(points: np.ndarray, eps: float, min_samples: int,
                    cone_r_min: float, cone_r_max: float) -> list:
    """Run DBSCAN then fit + validate a circle on each cluster.

    Args:
        points:       (N, 2) Cartesian point array.
        eps:          DBSCAN neighbourhood radius (metres).
        min_samples:  DBSCAN minimum cluster size.
        cone_r_min:   Minimum valid cone radius (metres).
        cone_r_max:   Maximum valid cone radius (metres).

    Returns:
        List of dicts with keys: cx, cy, radius, confidence.
    """
    if len(points) < min_samples:
        return []

    labels = DBSCAN(eps=eps, min_samples=min_samples).fit_predict(points)

    cones = []
    for label in set(labels):
        if label == -1:          # DBSCAN noise
            continue
        cluster = points[labels == label]
        result = fit_circle(cluster)
        if result is None:
            continue
        cx, cy, radius = result
        if not (cone_r_min <= radius <= cone_r_max):
            continue
        cones.append({
            'cx': cx,
            'cy': cy,
            'radius': radius,
            'confidence': circle_fit_confidence(cluster, cx, cy, radius),
        })
    return cones


# ---------------------------------------------------------------------------
# Marker construction helpers
# ---------------------------------------------------------------------------

# Standard F1TENTH traffic cone: 228 mm tall, used for marker height only.
_CONE_HEIGHT_M = 0.228
# Marker lifetime slightly longer than one 40 Hz scan period (25 ms).
_MARKER_LIFETIME_NS = 150_000_000


def _make_deleteall_marker(header) -> Marker:
    m = Marker()
    m.header = header
    m.ns = 'cone_candidates'
    m.action = Marker.DELETEALL
    return m


def _make_cone_marker(header, marker_id: int, cone: dict) -> Marker:
    """Build a white CYLINDER marker for a single cone candidate."""
    m = Marker()
    m.header = header
    m.ns = 'cone_candidates'
    m.id = marker_id
    m.type = Marker.CYLINDER
    m.action = Marker.ADD

    m.pose.position.x = cone['cx']
    m.pose.position.y = cone['cy']
    m.pose.position.z = _CONE_HEIGHT_M / 2.0   # centre the cylinder vertically
    m.pose.orientation.w = 1.0

    diameter = cone['radius'] * 2.0
    m.scale.x = diameter
    m.scale.y = diameter
    m.scale.z = _CONE_HEIGHT_M

    # White cone; alpha encodes confidence (floor at 0.3 so it's always visible)
    m.color.r = 1.0
    m.color.g = 1.0
    m.color.b = 1.0
    m.color.a = max(0.3, cone['confidence'])

    m.lifetime.nanosec = _MARKER_LIFETIME_NS
    return m


def build_marker_array(header, cones: list) -> MarkerArray:
    """Build a MarkerArray from a list of cone dicts.

    Prepends a DELETEALL marker so stale cones from the previous frame
    are cleared before the new set is drawn.
    """
    array = MarkerArray()
    array.markers.append(_make_deleteall_marker(header))
    for i, cone in enumerate(cones):
        array.markers.append(_make_cone_marker(header, i, cone))
    return array


# ---------------------------------------------------------------------------
# ROS 2 Node
# ---------------------------------------------------------------------------

class LidarProcessor(Node):
    """ROS 2 node that detects cone candidates from a 2-D LaserScan.

    Subscribes:
        /scan  (sensor_msgs/LaserScan)

    Publishes:
        /perception/cones        (roboracer_perception/ConeArray)  — structured data for Estimation/Planning/Control
        /perception/cone_markers (visualization_msgs/MarkerArray)  — cylinders for RViz
    """

    # Static laser→base_link translation (x forward only; y/z don't affect 2-D position).
    # base_link is at the rear axle (ground level). laser is at x=+0.270 m — confirmed
    # from the working f1tenth hardware bringup (bringup_launch.py / tf_publisher.py).
    _LASER_X_OFFSET = 0.270

    def __init__(self):
        super().__init__('lidar_processor')

        self.declare_parameter('range_min',          0.06)
        self.declare_parameter('range_max',          8.0)
        self.declare_parameter('dbscan_eps',         0.12)
        self.declare_parameter('dbscan_min_samples', 3)
        self.declare_parameter('cone_radius_min',    0.02)
        self.declare_parameter('cone_radius_max',    0.15)

        # BEST_EFFORT + depth=1: never buffer stale scans.
        scan_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._sub = self.create_subscription(
            LaserScan, '/scan', self._scan_callback, scan_qos)

        # Structured output — consumed by Estimation / Planning / Control
        self._cone_pub = self.create_publisher(ConeArray, '/perception/cones', 10)
        # Visualization output — consumed by RViz
        self._marker_pub = self.create_publisher(
            MarkerArray, '/perception/cone_markers', 10)

        self.get_logger().info('LidarProcessor started — waiting for /scan …')

    def _scan_callback(self, msg: LaserScan) -> None:
        p = self._params()

        points = filter_scan(
            msg.ranges, msg.angle_min, msg.angle_increment,
            p['range_min'], p['range_max'],
        )

        cones = cluster_and_fit(
            points,
            eps=p['dbscan_eps'],
            min_samples=p['dbscan_min_samples'],
            cone_r_min=p['cone_radius_min'],
            cone_r_max=p['cone_radius_max'],
        )

        # Build base_link header (frame for structured output)
        base_header = Header()
        base_header.stamp = msg.header.stamp
        base_header.frame_id = 'base_link'

        # Publish structured ConeArray in base_link frame
        self._cone_pub.publish(self._build_cone_array(base_header, cones))

        # Publish MarkerArray in laser frame (for RViz — markers already positioned correctly)
        self._marker_pub.publish(build_marker_array(msg.header, cones))

        self.get_logger().debug(
            f'{len(points)} pts → {len(cones)} cones')

    def _build_cone_array(self, header: Header, cones: list) -> ConeArray:
        """Convert cone dicts to a ConeArray in base_link frame.

        Cone positions from cluster_and_fit are in the laser frame.
        base_link is the rear axle. The laser is at x=+0.270 m from base_link,
        so base_link_x = laser_x + 0.270.  y is unchanged.
        Color is always UNKNOWN (0) in LiDAR-only mode.
        """
        msg = ConeArray()
        msg.header = header
        for c in cones:
            cone = Cone()
            cone.x = c['cx'] + self._LASER_X_OFFSET  # laser → base_link (rear axle): +0.270 m
            cone.y = c['cy']
            cone.color = Cone.COLOR_UNKNOWN
            cone.confidence = c['confidence']
            cone.radius = c['radius']
            msg.cones.append(cone)
        return msg

    def _params(self) -> dict:
        return {
            'range_min':          self.get_parameter('range_min').value,
            'range_max':          self.get_parameter('range_max').value,
            'dbscan_eps':         self.get_parameter('dbscan_eps').value,
            'dbscan_min_samples': self.get_parameter('dbscan_min_samples').value,
            'cone_radius_min':    self.get_parameter('cone_radius_min').value,
            'cone_radius_max':    self.get_parameter('cone_radius_max').value,
        }


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = LidarProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
