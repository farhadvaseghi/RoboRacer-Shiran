#!/usr/bin/env python3
"""LIDAR detector that distinguishes walls from discrete obstacles.

Designed for the solid_wall scenario where the LIDAR sees continuous walls
plus a small opponent car. The tube-track-era lidar_processor_pca treated
everything as a small object and showed wall fragments as spurious detections.
This node classifies each DBSCAN cluster into one of:
    - WALL    : long, elongated cluster (line-like)
    - OPPONENT: small, roughly rectangular cluster (car-shaped)
    - IGNORED : everything else (noise, range artifacts)

Publishes:
    /perception/walls    (MarkerArray) — one LINE_STRIP per wall, red
    /perception/opponent (MarkerArray) — one CUBE per opponent, green
"""

import math

import numpy as np
from sklearn.cluster import DBSCAN

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from roboracer_perception.msg import Detection, DetectionArray


_MARKER_LIFE_NS = 100_000_000
_OBSTACLE_HEIGHT = 0.30
_LASER_X_OFFSET = 0.270   # laser → base_link (rear axle) — see HOWTO.md


def _laser_to_xy(ranges, angle_min, angle_increment, range_min, range_max):
    pts = []
    for i, r in enumerate(ranges):
        if not math.isfinite(r) or r < range_min or r > range_max:
            continue
        a = angle_min + i * angle_increment
        pts.append((r * math.cos(a), r * math.sin(a)))
    return np.array(pts, dtype=np.float64) if pts else np.empty((0, 2), dtype=np.float64)


def _pca_shape(points: np.ndarray):
    """Return (cx, cy, length, width, orientation, aspect_ratio, sorted_proj_major)."""
    centroid = points.mean(axis=0)
    centered = points - centroid
    cov = np.cov(centered.T)
    if cov.ndim < 2:
        return None
    eigenvalues, eigenvectors = np.linalg.eigh(cov)
    major_vec = eigenvectors[:, 1]
    minor_vec = eigenvectors[:, 0]
    proj_major = centered @ major_vec
    proj_minor = centered @ minor_vec
    length = float(proj_major.max() - proj_major.min())
    width = float(proj_minor.max() - proj_minor.min())
    orientation = math.atan2(float(major_vec[1]), float(major_vec[0]))
    aspect = length / (width + 1e-9)
    return {
        'cx': float(centroid[0]), 'cy': float(centroid[1]),
        'length': length, 'width': width,
        'orientation': orientation, 'aspect_ratio': aspect,
        'major_vec': major_vec, 'proj_major': proj_major,
        'points': points,
    }


def _classify(shape, wall_min_length, opp_max_length, opp_max_width, opp_min_length):
    if shape['length'] >= wall_min_length and shape['aspect_ratio'] >= 4.0:
        return 'wall'
    if (opp_min_length <= shape['length'] <= opp_max_length
            and shape['width'] <= opp_max_width
            and shape['aspect_ratio'] < 4.0):
        return 'opponent'
    return 'ignore'


def _yaw_to_quat(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def _wall_marker(header, marker_id, shape, color_rgba):
    """LINE_STRIP through the two PCA-axis endpoints of the cluster."""
    m = Marker()
    m.header = header
    m.ns = 'walls'
    m.id = marker_id
    m.type = Marker.LINE_STRIP
    m.action = Marker.ADD
    m.scale.x = 0.08
    m.color.r, m.color.g, m.color.b, m.color.a = color_rgba

    cx, cy = shape['cx'], shape['cy']
    half = shape['length'] / 2.0
    vx, vy = float(shape['major_vec'][0]), float(shape['major_vec'][1])
    p1, p2 = Point(), Point()
    p1.x, p1.y, p1.z = cx - half * vx, cy - half * vy, 0.15
    p2.x, p2.y, p2.z = cx + half * vx, cy + half * vy, 0.15
    m.points = [p1, p2]
    m.lifetime.nanosec = _MARKER_LIFE_NS
    return m


def _opponent_marker(header, marker_id, shape, color_rgba):
    m = Marker()
    m.header = header
    m.ns = 'opponent'
    m.id = marker_id
    m.type = Marker.CUBE
    m.action = Marker.ADD
    m.pose.position.x = shape['cx']
    m.pose.position.y = shape['cy']
    m.pose.position.z = _OBSTACLE_HEIGHT / 2.0
    m.pose.orientation = _yaw_to_quat(shape['orientation'])
    m.scale.x = max(shape['length'], 0.10)
    m.scale.y = max(shape['width'], 0.10)
    m.scale.z = _OBSTACLE_HEIGHT
    m.color.r, m.color.g, m.color.b, m.color.a = color_rgba
    m.lifetime.nanosec = _MARKER_LIFE_NS
    return m


def _deleteall(header, ns):
    m = Marker()
    m.header = header
    m.ns = ns
    m.action = Marker.DELETEALL
    return m


def _make_detection(shape, kind, laser_x_offset):
    """Pack a PCA shape into a Detection for downstream teammates.

    Walls and opponent both use COLOR_UNKNOWN — env_modeling classifies further
    using position and tracking history. Full PCA shape (length, width, yaw)
    is preserved so teammates can use the opponent's heading or wall direction.
    """
    c = Detection()
    c.x = float(shape['cx'] + laser_x_offset)   # laser frame → base_link frame
    c.y = float(shape['cy'])
    c.color = Detection.COLOR_UNKNOWN
    c.confidence = 0.9 if kind == 'opponent' else 0.75
    c.length = float(shape['length'])
    c.width = float(shape['width'])
    c.yaw = float(shape['orientation'])
    c.radius = float(max(shape['length'], shape['width']) / 2.0)
    return c


class WallOpponentDetector(Node):
    def __init__(self):
        super().__init__('wall_opponent_detector')

        self.declare_parameter('range_min', 0.06)
        self.declare_parameter('range_max', 10.0)
        self.declare_parameter('dbscan_eps', 0.15)
        self.declare_parameter('dbscan_min_samples', 4)
        self.declare_parameter('wall_min_length', 0.8)
        self.declare_parameter('opponent_min_length', 0.15)
        self.declare_parameter('opponent_max_length', 0.80)
        self.declare_parameter('opponent_max_width', 0.50)

        scan_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._sub = self.create_subscription(LaserScan, '/scan', self._scan_cb, scan_qos)
        self._walls_pub = self.create_publisher(MarkerArray, '/perception/walls', 10)
        self._opp_pub = self.create_publisher(MarkerArray, '/perception/opponent', 10)
        # Canonical structured-data output for downstream teammates (env_modeling,
        # estimation, planning). Positions in base_link frame. See HOWTO.md
        # for the full interface contract.
        self._detections_pub = self.create_publisher(
            DetectionArray, '/perception/detections', 10)

        self.get_logger().info('WallOpponentDetector started — waiting for /scan …')

    def _p(self, name):
        return self.get_parameter(name).value

    def _scan_cb(self, msg: LaserScan):
        points = _laser_to_xy(
            msg.ranges, msg.angle_min, msg.angle_increment,
            self._p('range_min'), self._p('range_max'),
        )

        wall_min_length = float(self._p('wall_min_length'))
        opp_min_length = float(self._p('opponent_min_length'))
        opp_max_length = float(self._p('opponent_max_length'))
        opp_max_width = float(self._p('opponent_max_width'))

        wall_ma = MarkerArray()
        opp_ma = MarkerArray()
        wall_ma.markers.append(_deleteall(msg.header, 'walls'))
        opp_ma.markers.append(_deleteall(msg.header, 'opponent'))

        detections_msg = DetectionArray()
        detections_msg.header.stamp = msg.header.stamp
        detections_msg.header.frame_id = 'base_link'

        if len(points) >= self._p('dbscan_min_samples'):
            labels = DBSCAN(
                eps=float(self._p('dbscan_eps')),
                min_samples=int(self._p('dbscan_min_samples')),
            ).fit_predict(points)

            wall_id, opp_id = 1, 1
            for label in set(labels):
                if label == -1:
                    continue
                cluster = points[labels == label]
                if len(cluster) < 2:
                    continue
                shape = _pca_shape(cluster)
                if shape is None:
                    continue
                kind = _classify(shape, wall_min_length, opp_max_length,
                                 opp_max_width, opp_min_length)
                if kind == 'wall':
                    wall_ma.markers.append(
                        _wall_marker(msg.header, wall_id, shape, (1.0, 0.0, 0.0, 0.9)))
                    wall_id += 1
                    detections_msg.detections.append(
                        _make_detection(shape, kind, _LASER_X_OFFSET))
                elif kind == 'opponent':
                    opp_ma.markers.append(
                        _opponent_marker(msg.header, opp_id, shape, (0.0, 1.0, 0.0, 0.9)))
                    opp_id += 1
                    detections_msg.detections.append(
                        _make_detection(shape, kind, _LASER_X_OFFSET))

        self._walls_pub.publish(wall_ma)
        self._opp_pub.publish(opp_ma)
        self._detections_pub.publish(detections_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WallOpponentDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
