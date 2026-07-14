#!/usr/bin/env python3
"""zed_ground_calibration — one-time HARDWARE helper to calibrate the ZED mount.

The ZED is factory-calibrated for intrinsics (the wrapper publishes them), so we
do NOT need a checkerboard. What we DO need is the extrinsic mount pose — mainly
the pitch and height — because person_detector and depth_to_scan assume a level
mount and a fixed camera height.

Point the car at a flat, clear stretch of floor and run this node. It samples the
lower part of the depth image, fits the ground plane, and prints the calibrated
`cam_z` (height) and `cam_pitch` (rad) to paste into camera_params.yaml. It makes
no changes itself — read-only, hardware-only (needs a real ZED depth stream).
"""

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


def fit_ground(points_cam):
    """Fit a plane to camera-frame floor points; return (pitch_rad, height_m).

    points_cam: N×3 array of (x_right, y_down, z_forward) in the camera optical
    frame. For a level mount over flat ground the plane is y = height; a pitch
    tilts it in the (z, y) plane. We fit y = a*z + b*x + c and read pitch from
    the forward slope a and height from c.  Pure function — unit-testable.
    """
    x = points_cam[:, 0]
    y = points_cam[:, 1]
    z = points_cam[:, 2]
    A = np.column_stack([z, x, np.ones_like(z)])
    coef, *_ = np.linalg.lstsq(A, y, rcond=None)
    a, _b, c = coef            # y ≈ a*z + b*x + c
    pitch = float(np.arctan(a))   # forward tilt of the ground in camera frame
    height = float(c)             # camera height above ground (y-down => +c)
    return pitch, height


class ZedGroundCalibration(Node):
    def __init__(self):
        super().__init__('zed_ground_calibration')
        self.declare_parameter('depth_topic', '/zed/zed_node/depth/depth_registered')
        self.declare_parameter('info_topic', '/zed/zed_node/rgb/camera_info')
        self.declare_parameter('n_frames', 30)
        self.declare_parameter('min_depth', 0.4)
        self.declare_parameter('max_depth', 4.0)
        self.declare_parameter('lower_fraction', 0.5)  # use bottom half of image

        self._bridge = CvBridge()
        self._k = None
        self._acc = []
        self._n = self.get_parameter('n_frames').value

        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(CameraInfo, self.get_parameter('info_topic').value,
                                 self._info_cb, 10)
        self.create_subscription(Image, self.get_parameter('depth_topic').value,
                                 self._depth_cb, qos)
        self.get_logger().info('zed_ground_calibration: point at flat floor; '
                               'collecting %d frames...' % self._n)

    def _info_cb(self, msg):
        self._k = list(msg.k)

    def _depth_cb(self, msg):
        if self._k is None or len(self._acc) >= self._n:
            return
        d = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough').astype(np.float32)
        h, w = d.shape[:2]
        fx, cx, fy, cy = self._k[0], self._k[2], self._k[4], self._k[5]
        y0 = int(h * (1.0 - self.get_parameter('lower_fraction').value))
        mind = self.get_parameter('min_depth').value
        maxd = self.get_parameter('max_depth').value
        us, vs = np.meshgrid(np.arange(0, w, 4), np.arange(y0, h, 4))
        dd = d[vs, us]
        m = np.isfinite(dd) & (dd > mind) & (dd < maxd)
        xr = (us[m] - cx) * dd[m] / fx
        yd = (vs[m] - cy) * dd[m] / fy
        zf = dd[m]
        self._acc.append(np.column_stack([xr, yd, zf]))

        if len(self._acc) >= self._n:
            pts = np.vstack(self._acc)
            pitch, height = fit_ground(pts)
            self.get_logger().info(
                '\n==== ZED ground calibration ====\n'
                '  samples: %d\n  cam_pitch: %.4f rad (%.2f deg)\n  cam_z: %.4f m\n'
                'Paste cam_pitch / cam_z into camera_params.yaml.\n'
                '================================'
                % (pts.shape[0], pitch, np.degrees(pitch), height))
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ZedGroundCalibration()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
