#!/usr/bin/env python3
"""opponent_detector — LiDAR opponent detection + tracking -> /opp_racecar/odom.

This is the deployed port of the team's wall_opponent_detector (DBSCAN + PCA
wall/opponent classification — verified correct) plus the missing adapter that
makes its output *consumable* by the overtaking controller:

  * the detection logic (_laser_to_xy / _pca_shape / _classify) is unchanged and
    keeps publishing the RViz markers /perception/walls and /perception/opponent;
  * NEW: the opponent cluster is transformed into the estimator's world frame
    (odom / ego_racecar/odom), tracked across frames to estimate velocity, and
    republished as nav_msgs/Odometry on /opp_racecar/odom — the exact topic +
    type + fields the controller already subscribes to (pose + twist.linear.x).

On the real car this is the real-world replacement for the simulator's
ground-truth /opp_racecar/odom cheat: the controller consumes it with ZERO
changes. In sim, publish it on a different topic (/perception/opp_odom) and
compare against the ground truth to validate accuracy.
"""

import math

import numpy as np
from sklearn.cluster import DBSCAN

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Quaternion, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry

import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped transforms)


_MARKER_LIFE_NS = 100_000_000
_OBSTACLE_HEIGHT = 0.30


# --- detection primitives (verbatim from wall_opponent_detector — verified) ----
def _laser_to_xy(ranges, angle_min, angle_increment, range_min, range_max):
    pts = []
    for i, r in enumerate(ranges):
        if not math.isfinite(r) or r < range_min or r > range_max:
            continue
        a = angle_min + i * angle_increment
        pts.append((r * math.cos(a), r * math.sin(a)))
    return np.array(pts, dtype=np.float64) if pts else np.empty((0, 2), dtype=np.float64)


def _pca_shape(points):
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
    return {'cx': float(centroid[0]), 'cy': float(centroid[1]),
            'length': length, 'width': width, 'orientation': orientation,
            'aspect_ratio': aspect, 'major_vec': major_vec}


def _classify(shape, wall_min_length, opp_max_length, opp_max_width, opp_min_length):
    if shape['length'] >= wall_min_length and shape['aspect_ratio'] >= 4.0:
        return 'wall'
    if (opp_min_length <= shape['length'] <= opp_max_length
            and shape['width'] <= opp_max_width
            and shape['aspect_ratio'] < 4.0):
        return 'opponent'
    return 'ignore'


# --- tracking primitives (new, pure — unit-testable) --------------------------
def pick_opponent(opponents):
    """Choose the single opponent to report: nearest one ahead (x>0)."""
    ahead = [o for o in opponents if o[0] > 0.0]
    pool = ahead if ahead else opponents
    if not pool:
        return None
    return min(pool, key=lambda o: math.hypot(o[0], o[1]))


def estimate_velocity(px, py, pt, x, y, t, alpha, pvx, pvy,
                      max_dt=0.5, gate=1.5):
    """EMA velocity from consecutive opponent positions in a fixed frame.

    Returns (vx, vy, reset). Resets (vel=0) if the track is stale (dt>max_dt),
    dt<=0, or the jump exceeds `gate` (association break / new opponent).
    """
    if pt is None:
        return 0.0, 0.0, True
    dt = t - pt
    if dt <= 0.0 or dt > max_dt or math.hypot(x - px, y - py) > gate:
        return 0.0, 0.0, True
    vx_raw = (x - px) / dt
    vy_raw = (y - py) / dt
    vx = alpha * vx_raw + (1.0 - alpha) * pvx
    vy = alpha * vy_raw + (1.0 - alpha) * pvy
    return vx, vy, False


def _yaw_to_quat(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def _wall_marker(header, mid, shape):
    m = Marker(); m.header = header; m.ns = 'walls'; m.id = mid
    m.type = Marker.LINE_STRIP; m.action = Marker.ADD; m.scale.x = 0.08
    m.color.r, m.color.a = 1.0, 0.9
    cx, cy = shape['cx'], shape['cy']; half = shape['length'] / 2.0
    vx, vy = float(shape['major_vec'][0]), float(shape['major_vec'][1])
    p1, p2 = Point(), Point()
    p1.x, p1.y, p1.z = cx - half * vx, cy - half * vy, 0.15
    p2.x, p2.y, p2.z = cx + half * vx, cy + half * vy, 0.15
    m.points = [p1, p2]; m.lifetime.nanosec = _MARKER_LIFE_NS
    return m


def _opponent_marker(header, mid, shape):
    m = Marker(); m.header = header; m.ns = 'opponent'; m.id = mid
    m.type = Marker.CUBE; m.action = Marker.ADD
    m.pose.position.x = shape['cx']; m.pose.position.y = shape['cy']
    m.pose.position.z = _OBSTACLE_HEIGHT / 2.0
    m.pose.orientation = _yaw_to_quat(shape['orientation'])
    m.scale.x = max(shape['length'], 0.10); m.scale.y = max(shape['width'], 0.10)
    m.scale.z = _OBSTACLE_HEIGHT
    m.color.g, m.color.a = 1.0, 0.9; m.lifetime.nanosec = _MARKER_LIFE_NS
    return m


def _deleteall(header, ns):
    m = Marker(); m.header = header; m.ns = ns; m.action = Marker.DELETEALL
    return m


class OpponentDetector(Node):
    def __init__(self):
        super().__init__('opponent_detector')
        # detection params (same defaults as wall_opponent_detector)
        self.declare_parameter('range_min', 0.06)
        self.declare_parameter('range_max', 10.0)
        self.declare_parameter('dbscan_eps', 0.15)
        self.declare_parameter('dbscan_min_samples', 4)
        self.declare_parameter('wall_min_length', 0.8)
        self.declare_parameter('opponent_min_length', 0.15)
        self.declare_parameter('opponent_max_length', 0.80)
        self.declare_parameter('opponent_max_width', 0.50)
        # adapter params
        self.declare_parameter('output_frame', 'odom')        # EKF world frame
        self.declare_parameter('odom_topic', '/opp_racecar/odom')
        self.declare_parameter('vel_alpha', 0.5)

        self._tf_buf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buf, self)

        self._pt = None; self._px = 0.0; self._py = 0.0
        self._pvx = 0.0; self._pvy = 0.0; self._yaw = 0.0

        scan_qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                              history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, scan_qos)
        self._walls_pub = self.create_publisher(MarkerArray, '/perception/walls', 10)
        self._opp_pub = self.create_publisher(MarkerArray, '/perception/opponent', 10)
        self._odom_pub = self.create_publisher(
            Odometry, self.get_parameter('odom_topic').value, 10)
        self.get_logger().info('opponent_detector started -> %s (frame %s)'
                               % (self.get_parameter('odom_topic').value,
                                  self.get_parameter('output_frame').value))

    def _p(self, n):
        return self.get_parameter(n).value

    def _scan_cb(self, msg):
        pts = _laser_to_xy(msg.ranges, msg.angle_min, msg.angle_increment,
                           self._p('range_min'), self._p('range_max'))
        wall_ma = MarkerArray(); opp_ma = MarkerArray()
        wall_ma.markers.append(_deleteall(msg.header, 'walls'))
        opp_ma.markers.append(_deleteall(msg.header, 'opponent'))

        opponents = []
        if len(pts) >= self._p('dbscan_min_samples'):
            labels = DBSCAN(eps=float(self._p('dbscan_eps')),
                            min_samples=int(self._p('dbscan_min_samples'))
                            ).fit_predict(pts)
            wid, oid = 1, 1
            for label in set(labels):
                if label == -1:
                    continue
                cluster = pts[labels == label]
                if len(cluster) < 2:
                    continue
                shape = _pca_shape(cluster)
                if shape is None:
                    continue
                kind = _classify(shape, float(self._p('wall_min_length')),
                                 float(self._p('opponent_max_length')),
                                 float(self._p('opponent_max_width')),
                                 float(self._p('opponent_min_length')))
                if kind == 'wall':
                    wall_ma.markers.append(_wall_marker(msg.header, wid, shape)); wid += 1
                elif kind == 'opponent':
                    opp_ma.markers.append(_opponent_marker(msg.header, oid, shape)); oid += 1
                    opponents.append((shape['cx'], shape['cy'], shape))

        self._walls_pub.publish(wall_ma)
        self._opp_pub.publish(opp_ma)
        self._publish_opponent_odom(opponents, msg.header)

    def _publish_opponent_odom(self, opponents, header):
        best = pick_opponent(opponents)
        if best is None:
            return
        # scan-frame point -> EKF world frame via TF
        ps = PointStamped()
        ps.header.frame_id = header.frame_id
        ps.point.x, ps.point.y = float(best[0]), float(best[1])
        out_frame = self._p('output_frame')
        # NON-BLOCKING: look up the LATEST transform (Time()) — never block the
        # scan callback (a blocking timeout stalls the executor and, with a
        # dynamic odom<-laser, extrapolation-fails). Small time skew is fine here.
        try:
            tf = self._tf_buf.lookup_transform(out_frame, header.frame_id,
                                               rclpy.time.Time())
            tp = tf2_geometry_msgs.do_transform_point(ps, tf)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('opp TF %s<-%s failed: %s'
                                   % (out_frame, header.frame_id, exc),
                                   throttle_duration_sec=2.0)
            return
        x, y = tp.point.x, tp.point.y
        t = header.stamp.sec + header.stamp.nanosec * 1e-9
        vx, vy, reset = estimate_velocity(self._px, self._py, self._pt, x, y, t,
                                          float(self._p('vel_alpha')),
                                          self._pvx, self._pvy)
        speed = math.hypot(vx, vy)
        if speed > 0.10:
            self._yaw = math.atan2(vy, vx)
        self._px, self._py, self._pt = x, y, t
        self._pvx, self._pvy = vx, vy

        odom = Odometry()
        odom.header.stamp = header.stamp
        odom.header.frame_id = out_frame
        odom.child_frame_id = 'opponent'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation = _yaw_to_quat(self._yaw)
        odom.twist.twist.linear.x = speed
        self._odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = OpponentDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
