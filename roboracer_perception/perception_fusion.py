#!/usr/bin/env python3
"""Perception fusion node for RoboRacer.

Associates LiDAR cone positions with camera color detections to produce a
fused ConeArray with accurate positions (from LiDAR) and colors (from camera).

Subscribes:
    /perception/cones        (roboracer_perception/ConeArray) — LiDAR detections,
                              accurate position, color always COLOR_UNKNOWN
    /perception/camera_cones (roboracer_perception/ConeArray) — camera detections,
                              colored, less accurate position

Publishes:
    /perception/fused_cones  (roboracer_perception/ConeArray) — fused result
                              in base_link frame

Association strategy:
    On each LiDAR message, associate every LiDAR cone with the nearest camera
    cone within `association_distance_m`.  If a match is found, take the LiDAR
    position and the camera color; average the confidence values.  Unmatched
    LiDAR cones are published as-is with COLOR_UNKNOWN.
    Camera cones are cached from the most recent message — no synchronisation
    required because LiDAR is the authoritative position source.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Header

from roboracer_perception.msg import Cone, ConeArray


# ---------------------------------------------------------------------------
# Pure-function helpers (no ROS dependency — easy to unit-test)
# ---------------------------------------------------------------------------

def euclidean_2d(ax: float, ay: float, bx: float, by: float) -> float:
    """Return 2-D Euclidean distance between two points."""
    dx = ax - bx
    dy = ay - by
    return math.sqrt(dx * dx + dy * dy)


def associate_cones(lidar_cones: list, camera_cones: list,
                    max_dist: float) -> list:
    """Associate LiDAR cones with camera cones by nearest-neighbour in 2-D.

    Each camera cone can be matched to at most one LiDAR cone (greedy nearest
    neighbour; good enough for F1TENTH cone densities).

    Args:
        lidar_cones:  List of dicts with keys x, y, color, confidence, radius.
        camera_cones: List of dicts with keys x, y, color, confidence, radius.
        max_dist:     Maximum association distance in metres.

    Returns:
        List of dicts with keys x, y, color, confidence, radius.
    """
    used_cam: set = set()
    fused: list = []

    for lc in lidar_cones:
        best_dist = max_dist
        best_idx = -1

        for i, cc in enumerate(camera_cones):
            if i in used_cam:
                continue
            d = euclidean_2d(lc['x'], lc['y'], cc['x'], cc['y'])
            if d < best_dist:
                best_dist = d
                best_idx = i

        if best_idx >= 0:
            cc = camera_cones[best_idx]
            used_cam.add(best_idx)
            fused.append({
                'x':          lc['x'],
                'y':          lc['y'],
                'color':      cc['color'],
                'confidence': (lc['confidence'] + cc['confidence']) / 2.0,
                'radius':     lc['radius'],
            })
        else:
            # No camera match — publish LiDAR cone as-is
            fused.append({
                'x':          lc['x'],
                'y':          lc['y'],
                'color':      Cone.COLOR_UNKNOWN,
                'confidence': lc['confidence'],
                'radius':     lc['radius'],
            })

    return fused


def cone_msg_to_dict(cone: Cone) -> dict:
    """Convert a Cone message to a plain dict."""
    return {
        'x':          cone.x,
        'y':          cone.y,
        'color':      cone.color,
        'confidence': cone.confidence,
        'radius':     cone.radius,
    }


def build_cone_array(header: Header, cones: list) -> ConeArray:
    """Convert a list of cone dicts into a ConeArray message."""
    msg = ConeArray()
    msg.header = header
    for c in cones:
        cone = Cone()
        cone.x = float(c['x'])
        cone.y = float(c['y'])
        cone.color = int(c['color'])
        cone.confidence = float(c['confidence'])
        cone.radius = float(c['radius'])
        msg.cones.append(cone)
    return msg


# ---------------------------------------------------------------------------
# ROS 2 Node
# ---------------------------------------------------------------------------

class PerceptionFusion(Node):
    """ROS 2 node that fuses LiDAR and camera cone detections.

    LiDAR messages trigger fusion immediately using the most recently cached
    camera detections.  This avoids blocking on strict synchronisation while
    still exploiting temporal locality (the ZED publishes at ≥15 Hz, the
    LiDAR at 40 Hz).
    """

    def __init__(self):
        super().__init__('perception_fusion')

        self.declare_parameter('association_distance_m', 0.30)

        # Cache for most recent camera detections
        self._camera_cones: list = []

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._lidar_sub = self.create_subscription(
            ConeArray, '/perception/cones',
            self._lidar_callback, sensor_qos)

        self._camera_sub = self.create_subscription(
            ConeArray, '/perception/camera_cones',
            self._camera_callback, 10)

        self._fused_pub = self.create_publisher(
            ConeArray, '/perception/fused_cones', 10)

        self.get_logger().info('PerceptionFusion started.')

    # ------------------------------------------------------------------
    def _camera_callback(self, msg: ConeArray) -> None:
        """Cache the latest camera detections for use on the next LiDAR tick."""
        self._camera_cones = [cone_msg_to_dict(c) for c in msg.cones]

    # ------------------------------------------------------------------
    def _lidar_callback(self, msg: ConeArray) -> None:
        """Fuse LiDAR cones with cached camera cones and publish."""
        max_dist = self.get_parameter('association_distance_m').value

        lidar_cones = [cone_msg_to_dict(c) for c in msg.cones]
        fused = associate_cones(lidar_cones, self._camera_cones, max_dist)

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = 'base_link'

        self._fused_pub.publish(build_cone_array(header, fused))
        self.get_logger().debug(
            f'Fused {len(lidar_cones)} LiDAR + {len(self._camera_cones)} '
            f'camera → {len(fused)} cones.')


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
