#!/usr/bin/env python3
"""Synthetic camera node for RoboRacer simulation.

f1tenth_gym_ros provides no camera feed. This node renders a fake ZED 2i
output so that camera_processor can run its full HSV detection pipeline in sim.

Publishes on the exact same topics as the real ZED 2i wrapper:
    /zed/zed_node/rgb/image_rect_color   (sensor_msgs/Image, bgr8)
    /zed/zed_node/depth/depth_registered (sensor_msgs/Image, 32FC1)
    /zed/zed_node/rgb/camera_info        (sensor_msgs/CameraInfo)

Edit the CONES list below to adjust cone positions for your test scenario.
Positions are in the world frame (metres). Colors: 1=blue, 2=yellow, 3=orange.
"""

import math
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from cv_bridge import CvBridge

# ---------------------------------------------------------------------------
# Camera intrinsics — ZED 2i, 720p (approximate)
# ---------------------------------------------------------------------------
IMG_W = 1280
IMG_H = 720
FX = FY = 700.0
CX = IMG_W / 2.0   # 640.0
CY = IMG_H / 2.0   # 360.0

# Camera position relative to base_link (metres) — from CLAUDE.md
CAM_X =  0.116
CAM_Y = -0.005
CAM_Z =  0.133

# ---------------------------------------------------------------------------
# Cone geometry (standard F1Tenth traffic cone)
# ---------------------------------------------------------------------------
CONE_HEIGHT = 0.228   # m
CONE_RADIUS = 0.055   # m (base radius)

# BGR colours chosen to match camera_processor HSV thresholds
CONE_BGR = {
    1: (255,   0,   0),   # blue   (H~115)
    2: (  0, 255, 255),   # yellow (H~30)
    3: (  0, 128, 255),   # orange (H~15)
}

MAX_DIST = 8.0   # m — don't render cones beyond this distance

# ---------------------------------------------------------------------------
# Cone layout for the oval track (world frame, metres).
# Blue  cones = left  boundary (positive Y)
# Yellow cones = right boundary (negative Y)
# Orange cones = start / finish markers
# ---------------------------------------------------------------------------
CONES = [
    # Left boundary — blue  (y = +1.2 m, every 3 m)
    ( 3.0,  1.2, 1), ( 6.0,  1.2, 1), ( 9.0,  1.2, 1), (12.0,  1.2, 1),
    (15.0,  1.2, 1), (18.0,  1.2, 1), (21.0,  1.2, 1), (24.0,  1.2, 1),
    (27.0,  1.2, 1), (30.0,  1.2, 1), (33.0,  1.2, 1), (36.0,  1.2, 1),
    # Right boundary — yellow (y = -1.2 m, every 3 m)
    ( 3.0, -1.2, 2), ( 6.0, -1.2, 2), ( 9.0, -1.2, 2), (12.0, -1.2, 2),
    (15.0, -1.2, 2), (18.0, -1.2, 2), (21.0, -1.2, 2), (24.0, -1.2, 2),
    (27.0, -1.2, 2), (30.0, -1.2, 2), (33.0, -1.2, 2), (36.0, -1.2, 2),
    # Start / finish — orange
    ( 1.0,  1.2, 3), ( 1.0, -1.2, 3),
]


def _yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


class SimCamera(Node):
    """Publishes synthetic ZED 2i images rendered from known cone positions."""

    def __init__(self):
        super().__init__('sim_camera')

        self._bridge  = CvBridge()
        self._car_x   = 0.0
        self._car_y   = 0.0
        self._car_yaw = 0.0

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._odom_sub = self.create_subscription(
            Odometry, '/ego_racecar/odom', self._odom_cb, sensor_qos)

        self._img_pub   = self.create_publisher(
            Image,      '/zed/zed_node/rgb/image_rect_color',   10)
        self._depth_pub = self.create_publisher(
            Image,      '/zed/zed_node/depth/depth_registered', 10)
        self._info_pub  = self.create_publisher(
            CameraInfo, '/zed/zed_node/rgb/camera_info',        10)

        self._camera_info = self._build_camera_info()

        self.create_timer(0.1, self._publish)   # 10 Hz
        self.get_logger().info(
            f'SimCamera ready — rendering {len(CONES)} cones on ZED topics.')

    # ------------------------------------------------------------------
    def _odom_cb(self, msg: Odometry) -> None:
        self._car_x   = msg.pose.pose.position.x
        self._car_y   = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self._car_yaw = _yaw_from_quaternion(q.x, q.y, q.z, q.w)

    # ------------------------------------------------------------------
    def _publish(self) -> None:
        img, depth = self._render()
        ts = self.get_clock().now().to_msg()

        img_msg   = self._bridge.cv2_to_imgmsg(img,   encoding='bgr8')
        depth_msg = self._bridge.cv2_to_imgmsg(depth, encoding='passthrough')

        img_msg.header.stamp        = ts
        img_msg.header.frame_id     = 'zed_camera_link'
        depth_msg.header.stamp      = ts
        depth_msg.header.frame_id   = 'zed_camera_link'
        self._camera_info.header.stamp    = ts
        self._camera_info.header.frame_id = 'zed_camera_link'

        self._img_pub.publish(img_msg)
        self._depth_pub.publish(depth_msg)
        self._info_pub.publish(self._camera_info)

    # ------------------------------------------------------------------
    def _render(self):
        """Return (bgr_image, depth_image) with cones projected into camera view."""
        img   = np.full((IMG_H, IMG_W, 3), 180, dtype=np.uint8)   # grey background
        depth = np.full((IMG_H, IMG_W), np.nan, dtype=np.float32)

        cos_y = math.cos(self._car_yaw)
        sin_y = math.sin(self._car_yaw)

        for world_x, world_y, color_id in CONES:
            # World frame → base_link frame
            dx = world_x - self._car_x
            dy = world_y - self._car_y
            bl_x =  dx * cos_y + dy * sin_y   # forward
            bl_y = -dx * sin_y + dy * cos_y   # left

            # base_link → camera optical frame (x=right, y=down, z=forward)
            Z = bl_x - CAM_X
            if Z < 0.3 or Z > MAX_DIST:
                continue

            X = -(bl_y - CAM_Y)   # right is negative-left

            # Vertical extent: project cone bottom (z=0) and top (z=CONE_HEIGHT)
            Y_bot = CAM_Z - 0.0
            Y_top = CAM_Z - CONE_HEIGHT

            u_c   = int(FX * X       / Z + CX)
            v_bot = int(FY * Y_bot   / Z + CY)
            v_top = int(FY * Y_top   / Z + CY)
            half_w = max(3, int(FX * CONE_RADIUS / Z))

            # Skip if completely outside image
            if u_c + half_w < 0 or u_c - half_w >= IMG_W:
                continue
            v_top = max(0, min(IMG_H - 1, v_top))
            v_bot = max(0, min(IMG_H - 1, v_bot))

            cone_h  = max(4, v_bot - v_top)
            cx_img  = u_c
            cy_img  = (v_top + v_bot) // 2
            bgr     = CONE_BGR.get(color_id, (128, 128, 128))

            # Draw filled ellipse (taller than wide — realistic cone silhouette)
            cv2.ellipse(img,
                        (cx_img, cy_img), (half_w, cone_h // 2),
                        0, 0, 360, bgr, -1)

            # Write depth at cone pixels
            dist = math.sqrt(X ** 2 + (CAM_Z - CONE_HEIGHT / 2.0) ** 2 + Z ** 2)
            mask = np.zeros((IMG_H, IMG_W), dtype=np.uint8)
            cv2.ellipse(mask,
                        (cx_img, cy_img), (half_w, cone_h // 2),
                        0, 0, 360, 255, -1)
            depth[mask > 0] = float(dist)

        return img, depth

    # ------------------------------------------------------------------
    @staticmethod
    def _build_camera_info() -> CameraInfo:
        info = CameraInfo()
        info.width  = IMG_W
        info.height = IMG_H
        info.distortion_model = 'plumb_bob'
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [FX,  0.0, CX,
                  0.0, FY,  CY,
                  0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0]
        info.p = [FX,  0.0, CX,  0.0,
                  0.0, FY,  CY,  0.0,
                  0.0, 0.0, 1.0, 0.0]
        return info


def main(args=None):
    rclpy.init(args=args)
    node = SimCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
