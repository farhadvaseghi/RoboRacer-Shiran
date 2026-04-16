#!/usr/bin/env python3
"""Camera cone detection node for RoboRacer (real hardware only).

Pipeline:
  /zed/zed_node/rgb/image_rect_color  (sensor_msgs/Image, bgr8)
  /zed/zed_node/depth/depth_registered (sensor_msgs/Image, 32FC1, metres)
  /zed/zed_node/rgb/camera_info       (sensor_msgs/CameraInfo)
    → approximate-time sync (image + depth)
    → BGR → HSV conversion
    → per-color HSV threshold → contours → area filter
    → depth lookup at centroid → 3-D projection (camera frame)
    → rigid transform: camera frame → base_link frame
    → /perception/camera_cones  (roboracer_perception/ConeArray)
"""

import math

import cv2
import numpy as np
import message_filters

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from cv_bridge import CvBridge

from roboracer_perception.msg import Cone, ConeArray


# ---------------------------------------------------------------------------
# Pure-function helpers (no ROS dependency — easy to unit-test)
# ---------------------------------------------------------------------------

# Camera position relative to base_link (rear axle, ground level) in metres.
# Confirmed: same x as laser (0.270) + 45 mm above it. Matches perception.launch.py.
_CAM_X = 0.270   # forward from rear axle
_CAM_Y = -0.005  # right (negative Y in base_link = right)
_CAM_Z = 0.155   # above ground

# Area filter for contours (pixels²)
_AREA_MIN = 200
_AREA_MAX = 50_000

# Depth validity range (metres)
_DEPTH_MIN = 0.1
_DEPTH_MAX = 8.0

# Confidence scaling: blob of 5000 px² → confidence = 1.0
_CONF_AREA_SCALE = 5000.0

# Morphological kernel
_MORPH_KERNEL = np.ones((3, 3), np.uint8)


def _hsv_params(color: str, params: dict) -> tuple:
    """Return (lower, upper) HSV bounds as numpy arrays for the given color."""
    lo = np.array([
        params[f'{color}_h_low'],
        params[f'{color}_s_low'],
        params[f'{color}_v_low'],
    ], dtype=np.uint8)
    hi = np.array([
        params[f'{color}_h_high'],
        params[f'{color}_s_high'],
        params[f'{color}_v_high'],
    ], dtype=np.uint8)
    return lo, hi


def detect_cones_in_hsv(hsv_img: np.ndarray, depth_img: np.ndarray,
                         camera_info_k: list,
                         color_name: str, color_id: int,
                         hsv_lo: np.ndarray, hsv_hi: np.ndarray) -> list:
    """Detect cones of a single color in an HSV image.

    Args:
        hsv_img:       H×W×3 HSV image (OpenCV encoding).
        depth_img:     H×W float32 depth in metres (NaN = invalid).
        camera_info_k: Flattened 3×3 camera intrinsic matrix K (9 elements).
        color_name:    Human-readable color label (for logging only).
        color_id:      Cone.COLOR_* constant.
        hsv_lo:        Lower HSV bound (3-element uint8 array).
        hsv_hi:        Upper HSV bound (3-element uint8 array).

    Returns:
        List of dicts with keys: x, y, color, confidence, radius.
        Positions are in base_link frame (metres).
    """
    # Unpack intrinsics
    fx = camera_info_k[0]
    cx = camera_info_k[2]
    fy = camera_info_k[4]
    cy_px = camera_info_k[5]

    # Threshold + denoise
    mask = cv2.inRange(hsv_img, hsv_lo, hsv_hi)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, _MORPH_KERNEL)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)

    cones = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < _AREA_MIN or area > _AREA_MAX:
            continue

        # Centroid
        M = cv2.moments(cnt)
        if M['m00'] == 0.0:
            continue
        u = int(M['m10'] / M['m00'])
        v = int(M['m01'] / M['m00'])

        # Depth validity check
        h, w = depth_img.shape[:2]
        if v < 0 or v >= h or u < 0 or u >= w:
            continue
        depth = float(depth_img[v, u])
        if not math.isfinite(depth) or depth < _DEPTH_MIN or depth > _DEPTH_MAX:
            continue

        # 3-D position in camera optical frame (x=right, y=down, z=forward)
        X_cam = (u - cx) * depth / fx
        Y_cam = (v - cy_px) * depth / fy
        Z_cam = depth

        # Transform to base_link frame (x=forward, y=left, z=up)
        base_x = Z_cam + _CAM_X
        base_y = -X_cam + _CAM_Y
        # base_z = -Y_cam + _CAM_Z  # not used for 2-D interface

        # Confidence from blob area
        confidence = min(1.0, area / _CONF_AREA_SCALE)

        # Radius in metres from bounding-box pixel width and depth
        _, _, bw, _ = cv2.boundingRect(cnt)
        radius = (bw * depth / fx) / 2.0

        cones.append({
            'x': base_x,
            'y': base_y,
            'color': color_id,
            'confidence': confidence,
            'radius': radius,
        })

    return cones


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

class CameraProcessor(Node):
    """ROS 2 node that detects colored cones from ZED 2i stereo camera data.

    Subscribes (approximate-time sync):
        /zed/zed_node/rgb/image_rect_color  (sensor_msgs/Image, bgr8)
        /zed/zed_node/depth/depth_registered (sensor_msgs/Image, 32FC1)
        /zed/zed_node/rgb/camera_info       (sensor_msgs/CameraInfo)

    Publishes:
        /perception/camera_cones  (roboracer_perception/ConeArray)
    """

    def __init__(self):
        super().__init__('camera_processor')

        # Topic parameters
        self.declare_parameter('image_topic',
                               '/zed/zed_node/rgb/image_rect_color')
        self.declare_parameter('depth_topic',
                               '/zed/zed_node/depth/depth_registered')
        self.declare_parameter('info_topic',
                               '/zed/zed_node/rgb/camera_info')

        # HSV threshold parameters — blue
        self.declare_parameter('blue_h_low',   100)
        self.declare_parameter('blue_s_low',   120)
        self.declare_parameter('blue_v_low',    50)
        self.declare_parameter('blue_h_high',  130)
        self.declare_parameter('blue_s_high',  255)
        self.declare_parameter('blue_v_high',  255)

        # HSV threshold parameters — yellow
        self.declare_parameter('yellow_h_low',   20)
        self.declare_parameter('yellow_s_low',  120)
        self.declare_parameter('yellow_v_low',   80)
        self.declare_parameter('yellow_h_high',  40)
        self.declare_parameter('yellow_s_high', 255)
        self.declare_parameter('yellow_v_high', 255)

        # HSV threshold parameters — orange
        self.declare_parameter('orange_h_low',    5)
        self.declare_parameter('orange_s_low',  150)
        self.declare_parameter('orange_v_low',   80)
        self.declare_parameter('orange_h_high',  20)
        self.declare_parameter('orange_s_high', 255)
        self.declare_parameter('orange_v_high', 255)

        self._bridge = CvBridge()

        # Cache the most recent CameraInfo; it's nearly static after start-up.
        self._camera_info: CameraInfo | None = None

        # BEST_EFFORT QoS to match ZED wrapper default
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        image_topic = self.get_parameter('image_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        info_topic  = self.get_parameter('info_topic').value

        # Subscribers for approximate-time sync
        self._image_sub = message_filters.Subscriber(
            self, Image, image_topic, qos_profile=sensor_qos)
        self._depth_sub = message_filters.Subscriber(
            self, Image, depth_topic, qos_profile=sensor_qos)

        self._sync = message_filters.ApproximateTimeSynchronizer(
            [self._image_sub, self._depth_sub],
            queue_size=5,
            slop=0.05,
        )
        self._sync.registerCallback(self._image_depth_callback)

        # CameraInfo subscriber (separate — arrives at ~1 Hz after init)
        self._info_sub = self.create_subscription(
            CameraInfo, info_topic, self._info_callback, 10)

        self._cone_pub = self.create_publisher(
            ConeArray, '/perception/camera_cones', 10)

        self.get_logger().info(
            f'CameraProcessor started — syncing {image_topic} + {depth_topic}')

    # ------------------------------------------------------------------
    def _info_callback(self, msg: CameraInfo) -> None:
        if self._camera_info is None:
            self.get_logger().info('CameraInfo received — intrinsics ready.')
        self._camera_info = msg

    # ------------------------------------------------------------------
    def _image_depth_callback(self, image_msg: Image,
                               depth_msg: Image) -> None:
        if self._camera_info is None:
            self.get_logger().warn(
                'Skipping frame — CameraInfo not yet received.')
            return

        # Convert ROS images to OpenCV
        try:
            bgr = self._bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
            depth = self._bridge.imgmsg_to_cv2(
                depth_msg, desired_encoding='passthrough')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'cv_bridge conversion failed: {exc}')
            return

        if depth.dtype != np.float32:
            depth = depth.astype(np.float32)

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        K = list(self._camera_info.k)  # 9-element flattened 3×3

        p = self._params()
        all_cones: list = []

        for color_name, color_id in [
            ('blue',   Cone.COLOR_BLUE),
            ('yellow', Cone.COLOR_YELLOW),
            ('orange', Cone.COLOR_ORANGE),
        ]:
            lo, hi = _hsv_params(color_name, p)
            cones = detect_cones_in_hsv(
                hsv, depth, K, color_name, color_id, lo, hi)
            all_cones.extend(cones)

        header = Header()
        header.stamp = image_msg.header.stamp
        header.frame_id = 'base_link'

        self._cone_pub.publish(build_cone_array(header, all_cones))
        self.get_logger().debug(f'Published {len(all_cones)} camera cones.')

    # ------------------------------------------------------------------
    def _params(self) -> dict:
        names = [
            'blue_h_low', 'blue_s_low', 'blue_v_low',
            'blue_h_high', 'blue_s_high', 'blue_v_high',
            'yellow_h_low', 'yellow_s_low', 'yellow_v_low',
            'yellow_h_high', 'yellow_s_high', 'yellow_v_high',
            'orange_h_low', 'orange_s_low', 'orange_v_low',
            'orange_h_high', 'orange_s_high', 'orange_v_high',
        ]
        return {n: self.get_parameter(n).value for n in names}


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = CameraProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
