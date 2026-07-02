#!/usr/bin/env python3
"""depth_to_scan_node — ZED depth map -> synthetic LaserScan of standing obstacles.

Purpose
-------
On the real car the 2-D Hokuyo already sees the corridor walls perfectly at its
single scan slice (z = 0.11 m). The camera's ONLY non-redundant job is to catch
things that stand up off the floor but sit ABOVE or BELOW that slice — a box on a
chair, a shelf lip, a person's torso, an obstacle the LiDAR beam passes under.

This node reconstructs every depth pixel's height above the ground plane
(COLOUR-INDEPENDENT — it never reads RGB, which is the right call for white walls
and a dark floor with no coloured markers), keeps the pixels that "stand up"
between ground_z_min and ceiling_z_max, and collapses them into a horizontal
`sensor_msgs/LaserScan` on `/camera_scan`. Nav2's obstacle_layer already consumes
a LaserScan, so this plugs in as a *second observation source* with a one-block
additive edit to nav2_params_real.yaml — nothing else in the stack changes.

Frames
------
The scan is published in a dedicated `camera_scan` frame (x-forward, y-left,
z-up) whose static transform to base_link we publish ourselves (see
camera.launch.py). This makes the node fully independent of the ZED wrapper's
internal TF tree — we consume only the depth IMAGE and the intrinsics, never the
depth image's frame. Zero chance of a TF-frame collision with the ZED driver.

Assumptions / on-site tuning
----------------------------
* Level camera mount (zero pitch). A pitched mount biases reconstructed height;
  add a pitch term or lower ceiling_z_max / raise ground_z_min to compensate.
* Textureless white walls give the stereo camera sparse/false depth — that's
  fine here, the LiDAR owns the walls; the camera earns its keep on textured
  obstacles. Tune ground_z_min and range_min if the dark floor leaks in.
"""

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CameraInfo, LaserScan
from cv_bridge import CvBridge


# Camera position relative to base_link (rear axle, ground level), metres.
# Same x as the laser (0.270), 45 mm above it, 5 mm right. Matches CLAUDE.md /
# perception.launch.py. Used ONLY to reconstruct pixel height (base_z); the scan
# frame's location is set by the static TF in camera.launch.py to the same pose.
_CAM_Z = 0.155   # camera height above ground


def depth_to_ranges(depth_img, camera_info_k, params):
    """Collapse a depth map into per-bearing minimum ranges of standing obstacles.

    Pure function (no ROS) — unit-testable.

    Args:
        depth_img:     H×W float32 depth in metres (NaN/0/inf = invalid).
        camera_info_k: Flattened 3×3 intrinsic matrix K (9 elements).
        params:        dict with keys depth_min, depth_max, ground_z_min,
                       ceiling_z_max, range_min, range_max, angle_min,
                       angle_max, angle_increment, stride, min_pts.

    Returns:
        1-D float32 array of length N = round((angle_max-angle_min)/inc)+1.
        Empty bearings are +inf (Nav2 ignores ranges outside [range_min,max]).
    """
    fx = camera_info_k[0]
    cx = camera_info_k[2]
    fy = camera_info_k[4]
    cy = camera_info_k[5]

    stride = max(1, int(params['stride']))
    d = depth_img[::stride, ::stride]
    h, w = d.shape

    # Pixel grids (subsampled coordinates map back via *stride).
    us = (np.arange(w, dtype=np.float32) * stride).reshape(1, w)
    vs = (np.arange(h, dtype=np.float32) * stride).reshape(h, 1)

    with np.errstate(invalid='ignore', divide='ignore'):
        # Optical frame: x=right, y=down, z=forward(=depth).
        y_cam = (vs - cy) * d / fy           # down
        base_z = _CAM_Z - y_cam              # height above ground
        x_cam = (us - cx) * d / fx           # right

    valid = (np.isfinite(d) & (d > params['depth_min']) & (d < params['depth_max']))
    foreground = (valid
                  & (base_z > params['ground_z_min'])
                  & (base_z < params['ceiling_z_max']))

    # Horizontal (top-down) geometry in a x-forward / y-left frame at the camera.
    x_f = d                                   # forward
    y_l = -x_cam                              # left
    rng = np.hypot(x_f, y_l)                  # horizontal range
    theta = np.arctan2(y_l, x_f)             # bearing, +left/CCW

    sel = foreground & (rng > params['range_min']) & (rng < params['range_max'])
    if not np.any(sel):
        n = int(round((params['angle_max'] - params['angle_min'])
                      / params['angle_increment'])) + 1
        return np.full(n, np.inf, dtype=np.float32)

    theta_s = theta[sel]
    rng_s = rng[sel].astype(np.float32)

    inc = params['angle_increment']
    n = int(round((params['angle_max'] - params['angle_min']) / inc)) + 1
    bin_idx = np.round((theta_s - params['angle_min']) / inc).astype(np.int64)
    in_fov = (bin_idx >= 0) & (bin_idx < n)
    bin_idx = bin_idx[in_fov]
    rng_s = rng_s[in_fov]

    ranges = np.full(n, np.inf, dtype=np.float32)
    # Nearest obstacle per bearing.
    np.minimum.at(ranges, bin_idx, rng_s)

    # Noise gate: require at least min_pts hits in a bearing to trust it.
    min_pts = int(params.get('min_pts', 1))
    if min_pts > 1:
        counts = np.bincount(bin_idx, minlength=n)
        ranges[counts < min_pts] = np.inf

    return ranges


class DepthToScan(Node):
    """ROS 2 node: ZED depth -> /camera_scan LaserScan for the Nav2 costmap."""

    def __init__(self):
        super().__init__('depth_to_scan')

        # Topics (confirm against the installed ZED wrapper with `ros2 topic list`).
        self.declare_parameter('depth_topic',
                               '/zed/zed_node/depth/depth_registered')
        self.declare_parameter('info_topic',
                               '/zed/zed_node/rgb/camera_info')
        self.declare_parameter('scan_topic', '/camera_scan')
        self.declare_parameter('scan_frame', 'camera_scan')

        # Height band that counts as a standing obstacle (metres, base_link z).
        self.declare_parameter('ground_z_min', 0.08)   # reject floor / floor noise
        self.declare_parameter('ceiling_z_max', 1.50)  # reject ceiling / lights

        # Depth validity band (metres).
        self.declare_parameter('depth_min', 0.30)
        self.declare_parameter('depth_max', 8.00)

        # Output LaserScan geometry (radians / metres). FOV ~ ZED 2i H-FOV ≈ 110°.
        self.declare_parameter('angle_min', -0.95)
        self.declare_parameter('angle_max', 0.95)
        self.declare_parameter('angle_increment', 0.005)
        self.declare_parameter('range_min', 0.30)
        self.declare_parameter('range_max', 8.00)

        # Performance / noise.
        self.declare_parameter('stride', 2)     # pixel subsample factor
        self.declare_parameter('min_pts', 2)    # min hits per bearing to trust

        self._bridge = CvBridge()
        self._k = None  # cached intrinsics

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(
            CameraInfo, self.get_parameter('info_topic').value,
            self._info_cb, 10)
        self.create_subscription(
            Image, self.get_parameter('depth_topic').value,
            self._depth_cb, sensor_qos)
        self._pub = self.create_publisher(
            LaserScan, self.get_parameter('scan_topic').value, 10)

        self.get_logger().info(
            'depth_to_scan started — %s -> %s'
            % (self.get_parameter('depth_topic').value,
               self.get_parameter('scan_topic').value))

    def _info_cb(self, msg):
        if self._k is None:
            self.get_logger().info('CameraInfo received — intrinsics ready.')
        self._k = list(msg.k)

    def _params(self):
        names = ['ground_z_min', 'ceiling_z_max', 'depth_min', 'depth_max',
                 'angle_min', 'angle_max', 'angle_increment', 'range_min',
                 'range_max', 'stride', 'min_pts']
        return {n: self.get_parameter(n).value for n in names}

    def _depth_cb(self, msg):
        if self._k is None:
            self.get_logger().warn('Skipping frame — CameraInfo not yet received.',
                                   throttle_duration_sec=5.0)
            return
        try:
            depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('cv_bridge conversion failed: %s' % exc)
            return
        if depth.dtype != np.float32:
            depth = depth.astype(np.float32)

        p = self._params()
        ranges = depth_to_ranges(depth, self._k, p)

        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = self.get_parameter('scan_frame').value
        scan.angle_min = float(p['angle_min'])
        scan.angle_max = float(p['angle_max'])
        scan.angle_increment = float(p['angle_increment'])
        scan.range_min = float(p['range_min'])
        scan.range_max = float(p['range_max'])
        scan.ranges = ranges.tolist()
        self._pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = DepthToScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
