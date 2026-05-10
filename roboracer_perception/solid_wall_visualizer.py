#!/usr/bin/env python3
"""Publish raised wall markers for the solid-wall RoboRacer scenarios."""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from roboracer_perception.solid_wall_geometry import build_wall_segments


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """Return quaternion components from roll, pitch, yaw."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def marker_for_segment(
    marker_id: int,
    frame_id: str,
    x1: float,
    y1: float,
    x2: float,
    y2: float,
    diameter: float,
    color: ColorRGBA,
) -> Marker:
    """Build a horizontal cylinder marker spanning one wall segment."""
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = 'solid_walls'
    marker.id = marker_id
    marker.type = Marker.CYLINDER
    marker.action = Marker.ADD

    dx = x2 - x1
    dy = y2 - y1
    length = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    qx, qy, qz, qw = quaternion_from_euler(0.0, math.pi / 2.0, yaw)

    marker.pose.position.x = 0.5 * (x1 + x2)
    marker.pose.position.y = 0.5 * (y1 + y2)
    marker.pose.position.z = diameter * 0.5
    marker.pose.orientation.x = qx
    marker.pose.orientation.y = qy
    marker.pose.orientation.z = qz
    marker.pose.orientation.w = qw

    marker.scale.x = diameter
    marker.scale.y = diameter
    marker.scale.z = length
    marker.color = color
    return marker


class SolidWallVisualizer(Node):
    """Publish a persistent MarkerArray describing raised cylindrical walls."""

    def __init__(self) -> None:
        super().__init__('solid_wall_visualizer')

        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('topic', '/visualization/solid_wall_markers')
        self.declare_parameter('wall_diameter_m', 0.30)
        self.declare_parameter('arc_segment_length_m', 0.60)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        topic = self.get_parameter('topic').value
        self._frame_id = self.get_parameter('frame_id').value
        self._wall_diameter = float(self.get_parameter('wall_diameter_m').value)
        self._arc_segment_length = float(self.get_parameter('arc_segment_length_m').value)
        self._publisher = self.create_publisher(MarkerArray, topic, qos)
        self._timer = self.create_timer(1.0, self._publish_markers)

        self.get_logger().info(
            f'SolidWallVisualizer publishing horizontal cylinder walls on {topic}'
        )

    def _publish_markers(self) -> None:
        color = ColorRGBA(r=0.72, g=0.72, b=0.76, a=0.95)
        array = MarkerArray()

        clear = Marker()
        clear.header.frame_id = self._frame_id
        clear.ns = 'solid_walls'
        clear.action = Marker.DELETEALL
        array.markers.append(clear)

        segments = build_wall_segments(self._arc_segment_length)

        for marker_id, (x1, y1, x2, y2) in enumerate(segments):
            array.markers.append(
                marker_for_segment(
                    marker_id, self._frame_id, x1, y1, x2, y2, self._wall_diameter, color,
                )
            )

        self._publisher.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SolidWallVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
