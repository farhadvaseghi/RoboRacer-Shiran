#!/usr/bin/env python3
"""sim_person_publisher — stand-in for person_detector in simulation.

The simulator has no camera, so YOLO can't run. This node publishes the SAME
`/perception/persons` topic the real person_detector does, letting us validate
the emergency_brake end-to-end in sim with zero downstream changes.

Modes:
  * 'opponent'  — treat the sim opponent (/opp_racecar/odom) as a person, in the
                  ego's base_link frame. Drive the ego at the opponent and watch
                  the AEB stop it.
  * 'scripted'  — a single person that starts far ahead and approaches at a fixed
                  closing speed (no other actors needed).
"""

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def world_to_base(ex, ey, eyaw, ox, oy):
    """Opponent world (map) position -> ego base_link (x fwd, y left)."""
    dx, dy = ox - ex, oy - ey
    c, s = math.cos(eyaw), math.sin(eyaw)
    px = c * dx + s * dy
    py = -s * dx + c * dy
    return px, py


class SimPersonPublisher(Node):
    def __init__(self):
        super().__init__('sim_person_publisher')
        self.declare_parameter('mode', 'opponent')
        self.declare_parameter('ego_odom_topic', '/ego_racecar/odom')
        self.declare_parameter('opp_odom_topic', '/opp_racecar/odom')
        self.declare_parameter('output_frame', 'base_link')
        self.declare_parameter('publish_rate', 15.0)
        # scripted mode
        self.declare_parameter('start_distance', 4.0)
        self.declare_parameter('closing_speed', 0.6)

        self._mode = self.get_parameter('mode').value
        self._frame = self.get_parameter('output_frame').value
        self._ego = None
        self._opp = None
        self._script_x = self.get_parameter('start_distance').value

        self.create_subscription(Odometry, self.get_parameter('ego_odom_topic').value,
                                 self._ego_cb, 10)
        self.create_subscription(Odometry, self.get_parameter('opp_odom_topic').value,
                                 self._opp_cb, 10)
        self._pub = self.create_publisher(PoseArray, '/perception/persons', 10)
        rate = self.get_parameter('publish_rate').value
        self._dt = 1.0 / rate
        self.create_timer(self._dt, self._tick)
        self.get_logger().info('sim_person_publisher started (mode=%s)' % self._mode)

    def _ego_cb(self, msg):
        self._ego = msg

    def _opp_cb(self, msg):
        self._opp = msg

    def _tick(self):
        parr = PoseArray()
        parr.header.stamp = self.get_clock().now().to_msg()
        parr.header.frame_id = self._frame

        if self._mode == 'scripted':
            self._script_x = max(0.0, self._script_x -
                                 self.get_parameter('closing_speed').value * self._dt)
            parr.poses.append(self._pose(self._script_x, 0.0))
        elif self._ego is not None and self._opp is not None:
            ep = self._ego.pose.pose
            op = self._opp.pose.pose
            px, py = world_to_base(ep.position.x, ep.position.y,
                                   _yaw(ep.orientation),
                                   op.position.x, op.position.y)
            if px > 0.0:  # only if ahead of us
                parr.poses.append(self._pose(px, py))
        self._pub.publish(parr)

    def _pose(self, x, y):
        p = Pose()
        p.position.x = float(x)
        p.position.y = float(y)
        p.orientation.w = 1.0
        return p


def main(args=None):
    rclpy.init(args=args)
    node = SimPersonPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
