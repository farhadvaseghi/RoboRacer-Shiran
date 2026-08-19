#!/usr/bin/env python3
"""opponent_odom_adapter — robust bridge from YOUR perception to the MPC.

The custom pure_pursuit_controller's overtaking layer subscribes to a
nav_msgs/Odometry topic (opponent_odom_topic). wall_opponent_detector instead
publishes roboracer_perception/DetectionArray on /perception/detections
(opponent + wall shapes, positions in the ego base_link frame). This node ONLY
reformats and gates that output — it runs NO detection logic of its own:

    /perception/detections  (DetectionArray, base_link)   ─┐
                                                            ├─►  /perception/opp_odom
    /odometry/filtered      (Odometry, ego pose in world) ─┘     (Odometry, world frame)

Why the gating matters
----------------------
In this sim the opponent car ray-casts into /scan as a thin line, so
wall_opponent_detector rarely tags it as an opponent; meanwhile WALL CORNERS
fragment into short clusters that *do* pass its opponent gate. Forwarding the
nearest such cluster put a PHANTOM opponent on the wall and the MPC dodged into
it. So before publishing we apply defence-in-depth, all on the detector's own
output (no detector/controller changes):

  1. car-like shape   — width in [width_min, width_max] (kills thin wall arcs)
                        and length in [length_min, length_max]
  2. forward corridor — ahead of the ego and within a lateral band (an actual
                        blocker, not clutter off to the side near a wall)
  3. wall veto        — rejected if it sits within wall_clearance of any WALL
                        detection (confidence < opponent_confidence_min) in the
                        same message
  4. persistence      — must be seen at a consistent world position for
                        persist_frames before it drives the MPC; expires after
                        track_timeout_sec of silence

Opponent world pose is composed from the same ego odom the controller reads
(/odometry/filtered), so frames match exactly — no TF lookup. Twist stays zero:
single-frame detections carry no reliable velocity, and the MPC treating the
opponent as a static blocker to route around is the safe default. Every gate is
a parameter, so nothing here is hard-coded.
"""

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

from roboracer_perception.msg import DetectionArray


def _yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _point_segment_dist(px, py, cx, cy, yaw, length) -> float:
    """Distance from point (px,py) to a segment centred at (cx,cy), heading yaw,
    total length `length` — used to test proximity to a wall detection."""
    hx, hy = math.cos(yaw) * length / 2.0, math.sin(yaw) * length / 2.0
    ax, ay = cx - hx, cy - hy
    bx, by = cx + hx, cy + hy
    dx, dy = bx - ax, by - ay
    seg2 = dx * dx + dy * dy
    if seg2 < 1e-9:
        return math.hypot(px - cx, py - cy)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / seg2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


class OpponentOdomAdapter(Node):
    def __init__(self):
        super().__init__('opponent_odom_adapter')

        self.declare_parameter('detections_topic', '/perception/detections')
        self.declare_parameter('ego_odom_topic', '/odometry/filtered')
        self.declare_parameter('opp_odom_topic', '/perception/opp_odom')
        self.declare_parameter('world_frame', 'ego_racecar/odom')
        # wall_opponent_detector: opponent confidence 0.9, wall 0.75.
        self.declare_parameter('opponent_confidence_min', 0.85)
        # (1) car-like shape
        self.declare_parameter('width_min', 0.06)
        self.declare_parameter('width_max', 0.50)
        self.declare_parameter('length_min', 0.15)
        self.declare_parameter('length_max', 0.70)
        # (2) forward corridor (base_link metres)
        self.declare_parameter('forward_min', 0.20)
        self.declare_parameter('forward_max', 6.0)
        self.declare_parameter('lateral_max', 1.0)
        # (3) wall veto
        self.declare_parameter('wall_clearance', 0.40)
        # (4) persistence
        self.declare_parameter('persist_frames', 3)
        self.declare_parameter('gate_radius', 0.7)
        self.declare_parameter('track_timeout_sec', 0.5)

        g = self.get_parameter
        self._conf_min = float(g('opponent_confidence_min').value)
        self._wmin = float(g('width_min').value)
        self._wmax = float(g('width_max').value)
        self._lmin = float(g('length_min').value)
        self._lmax = float(g('length_max').value)
        self._fwd_min = float(g('forward_min').value)
        self._fwd_max = float(g('forward_max').value)
        self._lat_max = float(g('lateral_max').value)
        self._wall_clear = float(g('wall_clearance').value)
        self._persist = int(g('persist_frames').value)
        self._gate = float(g('gate_radius').value)
        self._timeout = float(g('track_timeout_sec').value)
        self._world_frame = g('world_frame').value

        self._ego = None            # (x, y, yaw) latest ego pose
        self._track = None          # (wx, wy) tracked opponent world position
        self._hits = 0
        self._last_seen = None      # rclpy Time of last track update

        self.create_subscription(
            Odometry, g('ego_odom_topic').value, self._ego_cb, 10)
        self.create_subscription(
            DetectionArray, g('detections_topic').value, self._det_cb, 10)
        self._pub = self.create_publisher(Odometry, g('opp_odom_topic').value, 10)

        self.get_logger().info(
            'opponent_odom_adapter: %s (+ ego %s) -> %s  [phantom-gated]' % (
                g('detections_topic').value, g('ego_odom_topic').value,
                g('opp_odom_topic').value))

    def _ego_cb(self, msg: Odometry):
        p = msg.pose.pose
        self._ego = (p.position.x, p.position.y, _yaw_from_quat(p.orientation))

    def _det_cb(self, msg: DetectionArray):
        if self._ego is None:
            return  # no ego pose yet — can't place the opponent in the world

        now = self.get_clock().now()

        # Split the detector's own output into walls and opponent candidates.
        walls = [d for d in msg.detections if d.confidence < self._conf_min]
        cand = None
        best_r2 = float('inf')
        for d in msg.detections:
            if d.confidence < self._conf_min:
                continue
            # (1) car-like shape — thin wall arcs have ~zero width.
            if not (self._wmin <= d.width <= self._wmax):
                continue
            if not (self._lmin <= d.length <= self._lmax):
                continue
            # (2) forward corridor — an actual blocker ahead, not side clutter.
            if not (self._fwd_min < d.x <= self._fwd_max) or abs(d.y) > self._lat_max:
                continue
            # (3) wall veto — drop candidates hugging a detected wall (corner phantoms).
            if any(_point_segment_dist(d.x, d.y, w.x, w.y, w.yaw, w.length)
                   < self._wall_clear for w in walls):
                continue
            r2 = d.x * d.x + d.y * d.y
            if r2 < best_r2:
                best_r2 = r2
                cand = d

        # Expire a stale track (opponent gone / all candidates gated out).
        if self._last_seen is not None and \
                (now - self._last_seen).nanoseconds * 1e-9 > self._timeout:
            self._track, self._hits, self._last_seen = None, 0, None

        if cand is None:
            return

        ex, ey, eyaw = self._ego
        c, s = math.cos(eyaw), math.sin(eyaw)
        wx = ex + cand.x * c - cand.y * s
        wy = ey + cand.x * s + cand.y * c
        wyaw = eyaw + cand.yaw

        # (4) persistence — associate to the running track before trusting it.
        if self._track is not None and math.hypot(
                wx - self._track[0], wy - self._track[1]) <= self._gate:
            self._hits += 1
        else:
            self._hits = 1
        self._track = (wx, wy)
        self._last_seen = now
        if self._hits < self._persist:
            return  # not yet confirmed — don't let it steer the MPC

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._world_frame
        out.child_frame_id = 'opponent'
        out.pose.pose.position.x = wx
        out.pose.pose.position.y = wy
        out.pose.pose.orientation.z = math.sin(wyaw / 2.0)
        out.pose.pose.orientation.w = math.cos(wyaw / 2.0)
        # twist stays zero — see module docstring.
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = OpponentOdomAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
