#!/usr/bin/env python3
"""Synthetic ZED camera for the solid-wall RoboRacer scenario.

Renders the wall cylinders from solid_wall_geometry as colored tubes in a
synthetic ZED image so camera_processor's HSV pipeline can detect them.
Outer walls are yellow (matches camera_processor color 2), inner walls are
blue (color 1). The opponent car body is grey in the sim; not drawn here.

Publishes on the same topics as the real ZED 2i wrapper:
    /zed/zed_node/rgb/image_rect_color   (sensor_msgs/Image, bgr8)
    /zed/zed_node/depth/depth_registered (sensor_msgs/Image, 32FC1)
    /zed/zed_node/rgb/camera_info        (sensor_msgs/CameraInfo)
"""

import math
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

from roboracer_perception.solid_wall_geometry import (
    build_wall_segments, INNER_R, OUTER_R, LEFT_CENTER, RIGHT_CENTER,
)


IMG_W = 1280
IMG_H = 720
FX = FY = 700.0
CX = IMG_W / 2.0
CY = IMG_H / 2.0

CAM_X = 0.270
CAM_Y = -0.005
CAM_Z = 0.155

WALL_DIAMETER = 0.30
TUBE_HALF_HEIGHT = WALL_DIAMETER / 2.0

MAX_DIST = 12.0
ARC_SEGMENT_LENGTH = 0.30

COLOR_OUTER_BGR = (0, 255, 255)   # yellow
COLOR_INNER_BGR = (255, 0, 0)     # blue
COLOR_OPP_BGR   = (0, 0, 255)     # red — opponent body

# Opponent car footprint (f1tenth standard)
OPP_LENGTH = 0.52
OPP_WIDTH  = 0.23
OPP_HEIGHT = 0.20


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _is_outer_segment(x1: float, y1: float, x2: float, y2: float) -> bool:
    """Outer wall = farther from track centerline / curve center."""
    mx = 0.5 * (x1 + x2)
    my = 0.5 * (y1 + y2)
    if mx < LEFT_CENTER[0]:
        r = math.hypot(mx - LEFT_CENTER[0], my - LEFT_CENTER[1])
        return r > 0.5 * (INNER_R + OUTER_R)
    if mx > RIGHT_CENTER[0]:
        r = math.hypot(mx - RIGHT_CENTER[0], my - RIGHT_CENTER[1])
        return r > 0.5 * (INNER_R + OUTER_R)
    return abs(my - 2.5) > 2.0


class SimCameraWalls(Node):
    def __init__(self) -> None:
        super().__init__('sim_camera_walls')

        self._bridge = CvBridge()
        self._car_x = 0.0
        self._car_y = 0.0
        self._car_yaw = 0.0

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._odom_sub = self.create_subscription(
            Odometry, '/ego_racecar/odom', self._odom_cb, sensor_qos)
        self._opp_odom_sub = self.create_subscription(
            Odometry, '/opp_racecar/odom', self._opp_odom_cb, sensor_qos)
        self._opp_pose = None  # (x, y, yaw) or None if no opp seen yet

        self._img_pub = self.create_publisher(
            Image, '/zed/zed_node/rgb/image_rect_color', 10)
        self._depth_pub = self.create_publisher(
            Image, '/zed/zed_node/depth/depth_registered', 10)
        self._info_pub = self.create_publisher(
            CameraInfo, '/zed/zed_node/rgb/camera_info', 10)

        raw_segments = build_wall_segments(ARC_SEGMENT_LENGTH)
        self._segments = [
            (seg, COLOR_OUTER_BGR if _is_outer_segment(*seg) else COLOR_INNER_BGR)
            for seg in raw_segments
        ]

        self._camera_info = self._build_camera_info()
        self.create_timer(0.1, self._publish)   # 10 Hz target — actual rate varies with WSL2 load; real ZED on Jetson is unrelated

        self.get_logger().info(
            f'SimCameraWalls ready — rendering {len(self._segments)} wall segments on ZED topics.'
        )

    def _odom_cb(self, msg: Odometry) -> None:
        self._car_x = msg.pose.pose.position.x
        self._car_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self._car_yaw = _yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def _opp_odom_cb(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        self._opp_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            _yaw_from_quaternion(q.x, q.y, q.z, q.w),
        )

    def _publish(self) -> None:
        img, depth = self._render()
        ts = self.get_clock().now().to_msg()

        img_msg = self._bridge.cv2_to_imgmsg(img, encoding='bgr8')
        depth_msg = self._bridge.cv2_to_imgmsg(depth, encoding='passthrough')

        img_msg.header.stamp = ts
        img_msg.header.frame_id = 'zed_camera_link'
        depth_msg.header.stamp = ts
        depth_msg.header.frame_id = 'zed_camera_link'
        self._camera_info.header.stamp = ts
        self._camera_info.header.frame_id = 'zed_camera_link'

        self._img_pub.publish(img_msg)
        self._depth_pub.publish(depth_msg)
        self._info_pub.publish(self._camera_info)

    def _render(self) -> tuple[np.ndarray, np.ndarray]:
        img = np.full((IMG_H, IMG_W, 3), 180, dtype=np.uint8)
        depth = np.full((IMG_H, IMG_W), np.nan, dtype=np.float32)

        cos_y = math.cos(self._car_yaw)
        sin_y = math.sin(self._car_yaw)
        Y_center = CAM_Z - TUBE_HALF_HEIGHT

        # Horizontal half-FOV ≈ atan(IMG_W/2 / FX) ≈ 42.4°. Cull segments whose
        # midpoint sits outside the FOV (with a small margin) before doing any
        # per-sample projection. This is the biggest perf win — eliminates
        # ~60% of the 106 segments on most frames.
        FOV_HALF_TAN = (IMG_W / 2.0 + 60.0) / FX   # margin for partial visibility

        order = []
        for (x1, y1, x2, y2), color in self._segments:
            # --- midpoint cull (cheap) ---
            mx = 0.5 * (x1 + x2)
            my = 0.5 * (y1 + y2)
            mdx = mx - self._car_x
            mdy = my - self._car_y
            m_bl_x = mdx * cos_y + mdy * sin_y
            m_bl_y = -mdx * sin_y + mdy * cos_y
            m_Z = m_bl_x - CAM_X
            seg_length = math.hypot(x2 - x1, y2 - y1)
            # If midpoint is far behind/past, AND segment can't reach into view
            if m_Z + seg_length < 0.3:
                continue
            if m_Z - seg_length > MAX_DIST:
                continue
            # If clearly off-axis, skip
            if m_Z > 0.5 and abs(m_bl_y - CAM_Y) > FOV_HALF_TAN * m_Z + seg_length:
                continue

            # --- per-sample projection (coarser sampling: 0.15m instead of 0.08m) ---
            n_samples = max(2, int(seg_length / 0.15) + 1)
            inv_n = 1.0 / (n_samples - 1) if n_samples > 1 else 0.0
            for i in range(n_samples):
                t = i * inv_n
                wx = x1 + t * (x2 - x1)
                wy = y1 + t * (y2 - y1)

                dx = wx - self._car_x
                dy = wy - self._car_y
                bl_x = dx * cos_y + dy * sin_y
                bl_y = -dx * sin_y + dy * cos_y

                Z = bl_x - CAM_X
                if Z < 0.3 or Z > MAX_DIST:
                    continue

                X = -(bl_y - CAM_Y)
                u = int(FX * X / Z + CX)
                v = int(FY * Y_center / Z + CY)
                r_px = max(3, int(FX * (WALL_DIAMETER / 2.0) / Z))

                if u + r_px < 0 or u - r_px >= IMG_W:
                    continue
                if v + r_px < 0 or v - r_px >= IMG_H:
                    continue

                dist = math.sqrt(X * X + Y_center * Y_center + Z * Z)
                order.append((dist, u, v, r_px, color))

        order.sort(key=lambda t: -t[0])
        for dist, u, v, r_px, color in order:
            cv2.circle(img, (u, v), r_px, color, -1)
            cv2.circle(depth, (u, v), r_px, float(dist), -1)

        if self._opp_pose is not None:
            self._draw_opponent(img, depth, cos_y, sin_y)

        return img, depth

    def _draw_opponent(self, img, depth, cos_y, sin_y):
        opp_x, opp_y, opp_yaw = self._opp_pose
        half_l, half_w = OPP_LENGTH / 2.0, OPP_WIDTH / 2.0
        corners_local = [
            ( half_l,  half_w),
            ( half_l, -half_w),
            (-half_l, -half_w),
            (-half_l,  half_w),
        ]
        cyaw, syaw = math.cos(opp_yaw), math.sin(opp_yaw)

        projected_top = []
        projected_bot = []
        depths = []
        for lx, ly in corners_local:
            wx = opp_x + cyaw * lx - syaw * ly
            wy = opp_y + syaw * lx + cyaw * ly
            dx = wx - self._car_x
            dy = wy - self._car_y
            bl_x = dx * cos_y + dy * sin_y
            bl_y = -dx * sin_y + dy * cos_y
            Z = bl_x - CAM_X
            if Z < 0.3:
                return  # opp is behind or straddling the camera plane — skip rendering
            if Z > MAX_DIST + 4.0:
                return  # opp too far away
            X = -(bl_y - CAM_Y)
            Y_bot = CAM_Z - 0.0
            Y_top = CAM_Z - OPP_HEIGHT
            u = int(FX * X / Z + CX)
            v_top = int(FY * Y_top / Z + CY)
            v_bot = int(FY * Y_bot / Z + CY)
            projected_top.append((u, v_top))
            projected_bot.append((u, v_bot))
            depths.append(math.sqrt(X ** 2 + Y_bot ** 2 + Z ** 2))

        avg_depth = sum(depths) / len(depths)
        poly = np.array(
            [projected_top[0], projected_top[1], projected_top[2], projected_top[3],
             projected_bot[3], projected_bot[2], projected_bot[1], projected_bot[0]],
            dtype=np.int32,
        )
        hull = cv2.convexHull(poly)
        cv2.fillPoly(img, [hull], COLOR_OPP_BGR)
        mask = np.zeros(depth.shape, dtype=np.uint8)
        cv2.fillPoly(mask, [hull], 1)
        depth[mask == 1] = float(avg_depth)

    def _build_camera_info(self) -> CameraInfo:
        info = CameraInfo()
        info.width = IMG_W
        info.height = IMG_H
        info.distortion_model = 'plumb_bob'
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [FX, 0.0, CX,
                  0.0, FY, CY,
                  0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0]
        info.p = [FX, 0.0, CX, 0.0,
                  0.0, FY, CY, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        return info


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimCameraWalls()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
