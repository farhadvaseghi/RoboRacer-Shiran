#!/usr/bin/env python3
"""person_detector — YOLO person detection + depth-based 3-D localization.

Detects people in the ZED RGB image with a pretrained YOLOv8-nano (COCO class
0 = person — no custom training), then reads the ZED depth map at each detection
to place the person in 3-D. Publishes their positions for the emergency_brake
node (and RViz).

This is the ONLY place a neural net runs. It is hardware-only (needs the ZED +
ultralytics + a GPU). In simulation there is no camera, so `sim_person_publisher`
stands in for this node on the same `/perception/persons` topic — the downstream
AEB is identical in sim and on the car.

Frames
------
People are published in **base_link** (x forward, y left) — a safety reflex is a
*local* reaction ("is someone in front of me"), so we keep it TF-light and
camera-relative rather than routing a safety-critical signal through map-frame
localization. (The opponent estimate, by contrast, is published in map.)

Subscribes: RGB image, depth image (32FC1 m), camera_info.
Publishes:  /perception/persons        (geometry_msgs/PoseArray, base_link)
            /perception/person_markers  (visualization_msgs/MarkerArray)
"""

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge


def project_to_base(u, v, depth, fx, fy, cx, cy, extr):
    """Project a pixel + depth into base_link coordinates (metres).

    Pure function (no ROS) — unit-testable.

    Optical frame: x=right, y=down, z=forward(=depth). base_link: x=fwd,y=left,z=up.
    A camera pitch (rad, nose-down positive) is applied as a rotation in the
    forward/up plane so a tilted mount does not bias the height/range.
    """
    x_cam = (u - cx) * depth / fx      # right
    y_cam = (v - cy) * depth / fy      # down
    z_cam = depth                      # forward

    # base_link before pitch (level mount)
    fwd = z_cam
    up = -y_cam
    left = -x_cam

    pitch = extr.get('cam_pitch', 0.0)
    if pitch != 0.0:
        c, s = np.cos(pitch), np.sin(pitch)
        # rotate (fwd, up) about the left axis; +pitch = camera nosed down
        fwd, up = c * fwd + s * up, -s * fwd + c * up

    bx = fwd + extr['cam_x']
    by = left + extr['cam_y']
    bz = up + extr['cam_z']
    return bx, by, bz


def median_depth_in_box(depth_img, x1, y1, x2, y2, min_d, max_d):
    """Robust depth for a detection box: median of valid depths in its middle."""
    h, w = depth_img.shape[:2]
    # shrink to the central 50% of the box to avoid background bleed at edges
    bw, bh = x2 - x1, y2 - y1
    cx1 = max(0, int(x1 + 0.25 * bw)); cx2 = min(w, int(x2 - 0.25 * bw))
    cy1 = max(0, int(y1 + 0.25 * bh)); cy2 = min(h, int(y2 - 0.25 * bh))
    if cx2 <= cx1 or cy2 <= cy1:
        return None
    patch = depth_img[cy1:cy2, cx1:cx2].astype(np.float32)
    valid = patch[np.isfinite(patch) & (patch > min_d) & (patch < max_d)]
    if valid.size < 10:
        return None
    return float(np.median(valid))


class PersonDetector(Node):
    def __init__(self):
        super().__init__('person_detector')
        self.declare_parameter('rgb_topic', '/zed/zed_node/rgb/image_rect_color')
        self.declare_parameter('depth_topic', '/zed/zed_node/depth/depth_registered')
        self.declare_parameter('info_topic', '/zed/zed_node/rgb/camera_info')
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('conf_threshold', 0.35)
        self.declare_parameter('person_class_id', 0)
        self.declare_parameter('output_frame', 'base_link')
        self.declare_parameter('min_depth', 0.3)
        self.declare_parameter('max_depth', 8.0)
        # calibrated extrinsics (see zed_ground_calibration)
        self.declare_parameter('cam_x', 0.270)
        self.declare_parameter('cam_y', -0.005)
        self.declare_parameter('cam_z', 0.155)
        self.declare_parameter('cam_pitch', 0.0)

        self._bridge = CvBridge()
        self._depth = None
        self._k = None

        # Lazy import so the module still imports (and other nodes/tests run)
        # without ultralytics installed. Fails loudly only when this node starts.
        from ultralytics import YOLO  # noqa: PLC0415
        self._model = YOLO(self.get_parameter('model_path').value)
        self.get_logger().info('YOLO model loaded: %s'
                               % self.get_parameter('model_path').value)

        sensor_qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(CameraInfo, self.get_parameter('info_topic').value,
                                 self._info_cb, 10)
        self.create_subscription(Image, self.get_parameter('depth_topic').value,
                                 self._depth_cb, sensor_qos)
        self.create_subscription(Image, self.get_parameter('rgb_topic').value,
                                 self._rgb_cb, sensor_qos)
        self._pub = self.create_publisher(PoseArray, '/perception/persons', 10)
        self._mpub = self.create_publisher(MarkerArray, '/perception/person_markers', 10)
        self.get_logger().info('person_detector started.')

    def _info_cb(self, msg):
        self._k = list(msg.k)

    def _depth_cb(self, msg):
        d = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self._depth = d.astype(np.float32) if d.dtype != np.float32 else d

    def _extr(self):
        return {n: self.get_parameter(n).value
                for n in ('cam_x', 'cam_y', 'cam_z', 'cam_pitch')}

    def _rgb_cb(self, msg):
        if self._depth is None or self._k is None:
            return
        rgb = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        conf = self.get_parameter('conf_threshold').value
        pid = self.get_parameter('person_class_id').value
        res = self._model.predict(rgb, conf=conf, classes=[pid], verbose=False)

        fx, cx = self._k[0], self._k[2]
        fy, cy = self._k[4], self._k[5]
        mind = self.get_parameter('min_depth').value
        maxd = self.get_parameter('max_depth').value
        frame = self.get_parameter('output_frame').value
        extr = self._extr()

        parr = PoseArray()
        parr.header.stamp = msg.header.stamp
        parr.header.frame_id = frame
        markers = MarkerArray()
        idx = 0
        for r in res:
            for box in r.boxes.xyxy.cpu().numpy():
                x1, y1, x2, y2 = box[:4]
                depth = median_depth_in_box(self._depth, x1, y1, x2, y2, mind, maxd)
                if depth is None:
                    continue
                u = 0.5 * (x1 + x2)
                v = 0.5 * (y1 + y2)
                bx, by, bz = project_to_base(u, v, depth, fx, fy, cx, cy, extr)
                pose = Pose()
                pose.position.x = float(bx)
                pose.position.y = float(by)
                pose.position.z = float(bz)
                pose.orientation.w = 1.0
                parr.poses.append(pose)
                markers.markers.append(self._marker(idx, frame, bx, by, msg.header.stamp))
                idx += 1

        self._pub.publish(parr)
        self._mpub.publish(markers)

    def _marker(self, i, frame, x, y, stamp):
        m = Marker()
        m.header.frame_id = frame
        m.header.stamp = stamp
        m.ns = 'persons'
        m.id = i
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.position.z = 0.5
        m.pose.orientation.w = 1.0
        m.scale.x = 0.4; m.scale.y = 0.4; m.scale.z = 1.0
        m.color.r = 1.0; m.color.g = 0.1; m.color.b = 0.1; m.color.a = 0.8
        m.lifetime.nanosec = 300_000_000
        return m


def main(args=None):
    rclpy.init(args=args)
    node = PersonDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
