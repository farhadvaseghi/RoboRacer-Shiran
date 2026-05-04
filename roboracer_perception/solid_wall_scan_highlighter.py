#!/usr/bin/env python3
"""Highlight scan hits that land on the cylinder wall geometry."""

from __future__ import annotations

from collections import deque
import math

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from roboracer_perception.solid_wall_geometry import build_wall_segments


LASER_X_OFFSET = 0.270


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def point_to_segment_distance(px: float, py: float, x1: float, y1: float, x2: float, y2: float) -> float:
    """Return the shortest 2-D distance from a point to a line segment."""
    dx = x2 - x1
    dy = y2 - y1
    seg_len_sq = dx * dx + dy * dy
    if seg_len_sq <= 1e-12:
        return math.hypot(px - x1, py - y1)
    t = ((px - x1) * dx + (py - y1) * dy) / seg_len_sq
    t = max(0.0, min(1.0, t))
    qx = x1 + t * dx
    qy = y1 + t * dy
    return math.hypot(px - qx, py - qy)


def evenly_sample_hits(
    hits: list[tuple[float, float]],
    max_hits: int,
) -> list[tuple[float, float]]:
    """Keep hit coverage across the full scan instead of truncating one side."""
    if len(hits) <= max_hits:
        return hits
    stride = len(hits) / max_hits
    return [hits[min(int(i * stride), len(hits) - 1)] for i in range(max_hits)]


class SolidWallScanHighlighter(Node):
    """Publish red markers for LaserScan points close to the cylinder walls."""

    def __init__(self) -> None:
        super().__init__('solid_wall_scan_highlighter')

        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('topic', '/visualization/solid_wall_hit_markers')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('wall_diameter_m', 0.30)
        self.declare_parameter('hit_tolerance_m', 0.08)
        self.declare_parameter('arc_segment_length_m', 0.60)
        self.declare_parameter('marker_size_m', 0.12)
        self.declare_parameter('beam_width_m', 0.03)
        self.declare_parameter('hit_height_m', 0.22)
        self.declare_parameter('history_scans', 6)
        self.declare_parameter('max_hits_per_scan', 120)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        marker_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._frame_id = self.get_parameter('frame_id').value
        self._wall_radius = 0.5 * float(self.get_parameter('wall_diameter_m').value)
        self._hit_tolerance = float(self.get_parameter('hit_tolerance_m').value)
        self._marker_size = float(self.get_parameter('marker_size_m').value)
        self._beam_width = float(self.get_parameter('beam_width_m').value)
        self._hit_height = float(self.get_parameter('hit_height_m').value)
        history_scans = max(1, int(self.get_parameter('history_scans').value))
        self._max_hits_per_scan = max(1, int(self.get_parameter('max_hits_per_scan').value))
        arc_segment_length = float(self.get_parameter('arc_segment_length_m').value)
        self._segments = build_wall_segments(arc_segment_length)
        self._recent_hits: deque[tuple[tuple[float, float], list[tuple[float, float]]]] = deque(
            maxlen=history_scans
        )

        self._pose = None

        scan_topic = self.get_parameter('scan_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        topic = self.get_parameter('topic').value

        self._scan_sub = self.create_subscription(LaserScan, scan_topic, self._scan_callback, sensor_qos)
        self._odom_sub = self.create_subscription(Odometry, odom_topic, self._odom_callback, sensor_qos)
        self._marker_pub = self.create_publisher(MarkerArray, topic, marker_qos)

        self.get_logger().info(f'SolidWallScanHighlighter publishing wall-hit markers on {topic}')

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

    def _scan_callback(self, msg: LaserScan) -> None:
        if self._pose is None:
            return

        pose_x, pose_y, yaw = self._pose
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        laser_origin = (
            pose_x + cos_yaw * LASER_X_OFFSET,
            pose_y + sin_yaw * LASER_X_OFFSET,
        )

        wall_hits: list[tuple[float, float]] = []
        threshold = self._wall_radius + self._hit_tolerance

        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            angle = msg.angle_min + i * msg.angle_increment

            x_laser = r * math.cos(angle)
            y_laser = r * math.sin(angle)
            x_base = x_laser + LASER_X_OFFSET
            y_base = y_laser
            x_world = pose_x + cos_yaw * x_base - sin_yaw * y_base
            y_world = pose_y + sin_yaw * x_base + cos_yaw * y_base

            if any(
                point_to_segment_distance(x_world, y_world, x1, y1, x2, y2) <= threshold
                for x1, y1, x2, y2 in self._segments
            ):
                wall_hits.append((x_world, y_world))

        wall_hits = evenly_sample_hits(wall_hits, self._max_hits_per_scan)

        self._recent_hits.append((laser_origin, wall_hits))

        array = MarkerArray()

        clear = Marker()
        clear.header.frame_id = self._frame_id
        clear.ns = 'solid_wall_hits'
        clear.action = Marker.DELETEALL
        array.markers.append(clear)

        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self._frame_id
        marker.ns = 'solid_wall_hits'
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = self._marker_size
        marker.scale.y = self._marker_size
        marker.color = ColorRGBA(r=1.0, g=0.15, b=0.15, a=0.95)

        beams = Marker()
        beams.header.stamp = marker.header.stamp
        beams.header.frame_id = self._frame_id
        beams.ns = 'solid_wall_hits'
        beams.id = 1
        beams.type = Marker.LINE_LIST
        beams.action = Marker.ADD
        beams.scale.x = self._beam_width
        beams.color = ColorRGBA(r=1.0, g=0.35, b=0.35, a=0.55)

        for origin, hits in self._recent_hits:
            origin_point = Point()
            origin_point.x = origin[0]
            origin_point.y = origin[1]
            origin_point.z = self._hit_height
            for x_world, y_world in hits:
                point = Point()
                point.x = x_world
                point.y = y_world
                point.z = self._hit_height
                marker.points.append(point)
                beams.points.append(origin_point)
                beams.points.append(point)

        array.markers.append(marker)
        array.markers.append(beams)
        self._marker_pub.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SolidWallScanHighlighter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
