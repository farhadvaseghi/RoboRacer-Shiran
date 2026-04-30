#!/usr/bin/env python3
"""Persistent cone tracking for environment modeling.

This node turns per-frame cone detections in the vehicle frame into a stable
world-frame cone map. In simulation it uses /ego_racecar/odom as the vehicle
pose source and tracks LiDAR cone detections from /perception/cones.

The implementation is deliberately split into pure helper functions plus a
thin ROS 2 wrapper so the core tracking logic can be tested without ROS.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from roboracer_perception.msg import Cone, ConeArray


_CONE_HEIGHT_M = 0.228
_DEFAULT_CONE_RADIUS_M = 0.114

_COLOR_RGB = {
    Cone.COLOR_BLUE: (0.0, 0.3, 1.0),
    Cone.COLOR_YELLOW: (1.0, 0.9, 0.0),
    Cone.COLOR_ORANGE: (1.0, 0.4, 0.0),
    Cone.COLOR_UNKNOWN: (0.55, 0.55, 0.55),
}


@dataclass
class Track:
    """Mutable track state for one world-frame cone."""

    x: float
    y: float
    color: int
    confidence: float
    radius: float
    hits: int = 1
    misses: int = 0


def yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    """Return planar yaw from a quaternion."""
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


def body_to_world(x_body: float, y_body: float,
                  pose_x: float, pose_y: float, yaw: float) -> tuple[float, float]:
    """Transform a 2-D point from the vehicle frame into the world frame."""
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    x_world = pose_x + cos_yaw * x_body - sin_yaw * y_body
    y_world = pose_y + sin_yaw * x_body + cos_yaw * y_body
    return x_world, y_world


def cone_to_dict(cone: Cone) -> dict:
    """Convert a Cone message into a plain dict."""
    return {
        'x': float(cone.x),
        'y': float(cone.y),
        'color': int(cone.color),
        'confidence': float(cone.confidence),
        'radius': float(cone.radius),
    }


def transform_detections_to_world(detections: list[dict],
                                  pose_x: float, pose_y: float,
                                  yaw: float) -> list[dict]:
    """Transform body-frame detections into the world frame."""
    transformed = []
    for detection in detections:
        x_world, y_world = body_to_world(
            detection['x'], detection['y'], pose_x, pose_y, yaw)
        world_detection = dict(detection)
        world_detection['x'] = x_world
        world_detection['y'] = y_world
        transformed.append(world_detection)
    return transformed


def associate_tracks(tracks: list[Track], detections: list[dict],
                     max_distance: float) -> tuple[list[tuple[int, int]], list[int], list[int]]:
    """Associate detections with tracks by greedy global nearest neighbour."""
    candidates: list[tuple[float, int, int]] = []
    for track_idx, track in enumerate(tracks):
        for detection_idx, detection in enumerate(detections):
            distance = math.hypot(track.x - detection['x'], track.y - detection['y'])
            if distance <= max_distance:
                candidates.append((distance, track_idx, detection_idx))

    candidates.sort()

    used_tracks: set[int] = set()
    used_detections: set[int] = set()
    matches: list[tuple[int, int]] = []

    for _, track_idx, detection_idx in candidates:
        if track_idx in used_tracks or detection_idx in used_detections:
            continue
        used_tracks.add(track_idx)
        used_detections.add(detection_idx)
        matches.append((track_idx, detection_idx))

    unmatched_tracks = [i for i in range(len(tracks)) if i not in used_tracks]
    unmatched_detections = [i for i in range(len(detections)) if i not in used_detections]
    return matches, unmatched_tracks, unmatched_detections


def update_tracks(tracks: list[Track], detections: list[dict],
                  association_distance: float, position_alpha: float,
                  max_missed_frames: int) -> list[Track]:
    """Update a track set with one frame of world-frame detections."""
    matches, unmatched_tracks, unmatched_detections = associate_tracks(
        tracks, detections, association_distance)

    for track_idx, detection_idx in matches:
        track = tracks[track_idx]
        detection = detections[detection_idx]

        track.x = (1.0 - position_alpha) * track.x + position_alpha * detection['x']
        track.y = (1.0 - position_alpha) * track.y + position_alpha * detection['y']
        track.radius = ((1.0 - position_alpha) * track.radius
                        + position_alpha * detection['radius'])
        track.confidence = max(track.confidence, detection['confidence'])
        if detection['color'] != Cone.COLOR_UNKNOWN:
            track.color = detection['color']
        track.hits += 1
        track.misses = 0

    for track_idx in unmatched_tracks:
        tracks[track_idx].misses += 1

    for detection_idx in unmatched_detections:
        detection = detections[detection_idx]
        tracks.append(Track(
            x=detection['x'],
            y=detection['y'],
            color=detection['color'],
            confidence=detection['confidence'],
            radius=detection['radius'],
        ))

    return [track for track in tracks if track.misses <= max_missed_frames]


def confirmed_tracks(tracks: list[Track], min_observations: int) -> list[Track]:
    """Return only tracks mature enough to publish as map cones."""
    return [track for track in tracks if track.hits >= min_observations]


def build_cone_array(frame_id: str, stamp, tracks: list[Track]) -> ConeArray:
    """Build a ConeArray message from confirmed tracks."""
    msg = ConeArray()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    for track in tracks:
        cone = Cone()
        cone.x = float(track.x)
        cone.y = float(track.y)
        cone.color = int(track.color)
        cone.confidence = float(track.confidence)
        cone.radius = float(track.radius)
        msg.cones.append(cone)
    return msg


def build_marker_array(frame_id: str, stamp, tracks: list[Track]) -> MarkerArray:
    """Build a color-coded marker array from confirmed tracks."""
    array = MarkerArray()

    clear = Marker()
    clear.header.stamp = stamp
    clear.header.frame_id = frame_id
    clear.action = Marker.DELETEALL
    array.markers.append(clear)

    for marker_id, track in enumerate(tracks):
        r, g, b = _COLOR_RGB.get(track.color, _COLOR_RGB[Cone.COLOR_UNKNOWN])
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = frame_id
        marker.ns = 'cone_map'
        marker.id = marker_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = track.x
        marker.pose.position.y = track.y
        marker.pose.position.z = _CONE_HEIGHT_M / 2.0
        marker.pose.orientation.w = 1.0
        radius = max(track.radius, _DEFAULT_CONE_RADIUS_M / 2.0)
        marker.scale.x = radius * 2.0
        marker.scale.y = radius * 2.0
        marker.scale.z = _CONE_HEIGHT_M
        marker.color = ColorRGBA(r=r, g=g, b=b, a=max(0.45, track.confidence))
        array.markers.append(marker)

    return array


class ConeTracker(Node):
    """ROS 2 node that builds a persistent world-frame cone map."""

    def __init__(self):
        super().__init__('cone_tracker')

        self.declare_parameter('input_topic', '/perception/cones')
        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('map_frame', 'ego_racecar/odom')
        self.declare_parameter('association_distance_m', 0.50)
        self.declare_parameter('position_alpha', 0.20)
        self.declare_parameter('min_observations', 3)
        self.declare_parameter('max_missed_frames', 15)

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        input_topic = self.get_parameter('input_topic').value
        odom_topic = self.get_parameter('odom_topic').value

        self._tracks: list[Track] = []
        self._pose_x: float | None = None
        self._pose_y: float | None = None
        self._yaw: float | None = None
        self._last_odom_frame: str | None = None

        self._odom_sub = self.create_subscription(
            Odometry, odom_topic, self._odom_callback, sensor_qos)
        self._detection_sub = self.create_subscription(
            ConeArray, input_topic, self._detection_callback, 10)

        self._cone_pub = self.create_publisher(ConeArray, '/perception/cone_map', 10)
        self._marker_pub = self.create_publisher(
            MarkerArray, '/perception/cone_map_markers', 10)

        self.get_logger().info(
            f'ConeTracker started — input={input_topic}, odom={odom_topic}')

    def _odom_callback(self, msg: Odometry) -> None:
        self._pose_x = msg.pose.pose.position.x
        self._pose_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self._yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self._last_odom_frame = msg.header.frame_id or None

    def _detection_callback(self, msg: ConeArray) -> None:
        if self._pose_x is None or self._pose_y is None or self._yaw is None:
            self.get_logger().warn(
                'Skipping cone tracking update until odometry is available.',
                throttle_duration_sec=5.0,
            )
            return

        params = self._params()
        detections = [cone_to_dict(cone) for cone in msg.cones]
        world_detections = transform_detections_to_world(
            detections, self._pose_x, self._pose_y, self._yaw)

        self._tracks = update_tracks(
            self._tracks,
            world_detections,
            association_distance=params['association_distance_m'],
            position_alpha=params['position_alpha'],
            max_missed_frames=params['max_missed_frames'],
        )

        confirmed = confirmed_tracks(self._tracks, params['min_observations'])
        map_frame = params['map_frame'] or self._last_odom_frame or 'odom'

        self._cone_pub.publish(build_cone_array(map_frame, msg.header.stamp, confirmed))
        self._marker_pub.publish(build_marker_array(map_frame, msg.header.stamp, confirmed))

    def _params(self) -> dict:
        return {
            'association_distance_m': self.get_parameter('association_distance_m').value,
            'position_alpha': self.get_parameter('position_alpha').value,
            'min_observations': self.get_parameter('min_observations').value,
            'max_missed_frames': self.get_parameter('max_missed_frames').value,
            'map_frame': self.get_parameter('map_frame').value,
        }


def main(args=None):
    rclpy.init(args=args)
    node = ConeTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
