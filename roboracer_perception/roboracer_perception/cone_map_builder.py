#!/usr/bin/env python3
"""Environment Modeling — Cone Map Builder.

Maintains a persistent map of cones in the world (odom) frame by accumulating
per-frame detections from the perception fusion node.

Architecture role
-----------------
  Sensor Data Processing layer  (lidar_processor, camera_processor, perception_fusion)
      publishes per-frame cone candidates in base_link frame
                   │
                   ▼
  Environment Modeling layer    (this node — cone_map_builder)
      • transforms detections into the odom frame
      • associates each detection with an existing map cone (nearest-neighbour)
      • accumulates position (exponential moving average) and color evidence
      • promotes a candidate to "confirmed" after MIN_OBS observations
      • removes a cone that goes MAX_MISSES frames without a match

Subscribes
----------
  /perception/fused_cones   ConeArray  — per-frame, base_link frame

Publishes
---------
  /perception/cone_map          ConeArray    — confirmed cones in odom frame
  /perception/cone_map_markers  MarkerArray  — for RViz, color-coded cylinders
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PointStamped
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import TransformException

from roboracer_perception.msg import Cone, ConeArray   # custom messages

# ── Tuning constants ────────────────────────────────────────────────────────
ASSOC_DIST   = 0.50   # m — max distance to match a detection to a map cone
ALPHA        = 0.20   # position EMA weight for incoming observation
MIN_OBS      = 3      # observations required before a cone is "confirmed"
MAX_MISSES   = 15     # frames without observation before cone is removed
SOURCE_FRAME = 'ego_racecar/odom' # world frame the map is built in

# ── Cone visual dimensions (F1TENTH standard cone) ──────────────────────────
CONE_RADIUS  = 0.114  # m
CONE_HEIGHT  = 0.228  # m

_COLOR_RGB = {
    Cone.COLOR_BLUE:    (0.0, 0.3, 1.0),
    Cone.COLOR_YELLOW:  (1.0, 0.9, 0.0),
    Cone.COLOR_ORANGE:  (1.0, 0.4, 0.0),
    Cone.COLOR_UNKNOWN: (0.55, 0.55, 0.55),
}


class _MapCone:
    """Internal representation of a single cone in the map."""
    __slots__ = ('x', 'y', 'color', 'confidence', 'obs_count', 'miss_count')

    def __init__(self, x: float, y: float, color: int, confidence: float):
        self.x = x
        self.y = y
        self.color = color
        self.confidence = confidence
        self.obs_count = 1
        self.miss_count = 0


class ConeMapBuilder(Node):
    """Environment modeling node: builds a persistent cone map from detections."""

    def __init__(self):
        super().__init__('cone_map_builder')

        qos_in = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._sub = self.create_subscription(
            ConeArray, '/perception/fused_cones', self._on_fused, qos_in
        )
        self._map_pub    = self.create_publisher(ConeArray,   '/perception/cone_map',          10)
        self._marker_pub = self.create_publisher(MarkerArray, '/perception/cone_map_markers',   10)

        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._map: list[_MapCone] = []

        self.get_logger().info(
            f'ConeMapBuilder started — map frame: {SOURCE_FRAME}, '
            f'assoc_dist={ASSOC_DIST} m, min_obs={MIN_OBS}, max_misses={MAX_MISSES}'
        )

    # ── Main callback ────────────────────────────────────────────────────────

    def _on_fused(self, msg: ConeArray) -> None:
        src_frame = msg.header.frame_id or 'base_link'

        # --- 1. Look up transform from detection frame to map frame ----------
        try:
            tf = self._tf_buffer.lookup_transform(
                SOURCE_FRAME, src_frame,
                msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f'TF {src_frame}→{SOURCE_FRAME} unavailable: {exc}',
                throttle_duration_sec=5.0,
            )
            return

        # --- 2. Transform detections and associate with map ------------------
        matched_indices: set[int] = set()

        for cone in msg.cones:
            # Transform detected cone position to odom frame
            pt = PointStamped()
            pt.header = msg.header
            pt.point.x = cone.x
            pt.point.y = cone.y
            pt.point.z = 0.0
            try:
                pt_world = tf2_geometry_msgs.do_transform_point(pt, tf)
            except Exception:
                continue

            wx, wy = pt_world.point.x, pt_world.point.y

            # Nearest-neighbour association in the map
            best_idx, best_dist = -1, ASSOC_DIST
            for i, mc in enumerate(self._map):
                d = math.hypot(mc.x - wx, mc.y - wy)
                if d < best_dist:
                    best_dist = d
                    best_idx  = i

            if best_idx >= 0:
                # Update existing map cone
                mc = self._map[best_idx]
                mc.x          = (1.0 - ALPHA) * mc.x + ALPHA * wx
                mc.y          = (1.0 - ALPHA) * mc.y + ALPHA * wy
                if cone.color != Cone.COLOR_UNKNOWN:
                    mc.color  = cone.color          # trust any coloured detection
                mc.confidence = min(1.0, mc.confidence + 0.05)
                mc.obs_count += 1
                mc.miss_count = 0
                matched_indices.add(best_idx)
            else:
                # New candidate
                self._map.append(_MapCone(wx, wy, cone.color, cone.confidence))

        # --- 3. Age unmatched cones and prune stale ones ---------------------
        for i, mc in enumerate(self._map):
            if i not in matched_indices:
                mc.miss_count += 1

        self._map = [mc for mc in self._map if mc.miss_count < MAX_MISSES]

        # --- 4. Publish confirmed cones --------------------------------------
        confirmed = [mc for mc in self._map if mc.obs_count >= MIN_OBS]
        self._publish(confirmed, msg.header.stamp)

    # ── Publishing ───────────────────────────────────────────────────────────

    def _publish(self, confirmed: list[_MapCone], stamp) -> None:
        cone_array = ConeArray()
        cone_array.header.stamp    = stamp
        cone_array.header.frame_id = SOURCE_FRAME

        marker_array = MarkerArray()
        # Delete all previously published markers in one shot
        clear = Marker()
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)

        for idx, mc in enumerate(confirmed):
            # ConeArray message
            c            = Cone()
            c.x          = mc.x
            c.y          = mc.y
            c.color      = mc.color
            c.confidence = mc.confidence
            c.radius     = CONE_RADIUS
            cone_array.cones.append(c)

            # RViz marker (cylinder)
            r, g, b = _COLOR_RGB.get(mc.color, _COLOR_RGB[Cone.COLOR_UNKNOWN])
            m = Marker()
            m.header.stamp    = stamp
            m.header.frame_id = SOURCE_FRAME
            m.ns              = 'cone_map'
            m.id              = idx
            m.type            = Marker.CYLINDER
            m.action          = Marker.ADD
            m.pose.position.x = mc.x
            m.pose.position.y = mc.y
            m.pose.position.z = CONE_HEIGHT / 2.0
            m.pose.orientation.w = 1.0
            m.scale.x = CONE_RADIUS * 2
            m.scale.y = CONE_RADIUS * 2
            m.scale.z = CONE_HEIGHT
            m.color   = ColorRGBA(r=r, g=g, b=b, a=0.85)
            marker_array.markers.append(m)

        self._map_pub.publish(cone_array)
        self._marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = ConeMapBuilder()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
