#!/usr/bin/env python3
"""map_odom_relay — publish the ego pose in the MAP frame for the custom controller.

The custom pure_pursuit reads its pose straight from the odom message with no TF
transform, and it follows a path (/control/plan) that is in the map frame. On the
real car the raw /odom is in the drifting odom frame, so feeding it directly would
mismatch the plan. This node bridges that gap (guide.md §9):

    TF map->base_link (from AMCL)  +  velocity from /odom  ->  /odometry/map

So the controller can consume /odometry/map as a map-frame pose. Small, generic
localization glue — needs AMCL (map->odom) + VESC (odom->base_link) running.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

import tf2_ros


class MapOdomRelay(Node):
    def __init__(self):
        super().__init__('map_odom_relay')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_topic', '/odom')          # velocity source
        self.declare_parameter('output_topic', '/odometry/map')
        self.declare_parameter('rate', 50.0)

        self._map = self.get_parameter('map_frame').value
        self._base = self.get_parameter('base_frame').value

        self._buf = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._buf, self)
        self._twist = None  # latest velocity from /odom

        self.create_subscription(Odometry, self.get_parameter('odom_topic').value,
                                 self._odom_cb, 10)
        self._pub = self.create_publisher(
            Odometry, self.get_parameter('output_topic').value, 10)
        self.create_timer(1.0 / self.get_parameter('rate').value, self._tick)
        self.get_logger().info('map_odom_relay: %s<-%s + %s -> %s'
                               % (self._map, self._base,
                                  self.get_parameter('odom_topic').value,
                                  self.get_parameter('output_topic').value))

    def _odom_cb(self, msg):
        self._twist = msg.twist.twist

    def _tick(self):
        try:
            tf = self._buf.lookup_transform(self._map, self._base,
                                            rclpy.time.Time())
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('no %s<-%s TF yet: %s'
                                   % (self._map, self._base, exc),
                                   throttle_duration_sec=3.0)
            return
        od = Odometry()
        od.header.stamp = self.get_clock().now().to_msg()
        od.header.frame_id = self._map
        od.child_frame_id = self._base
        t = tf.transform.translation
        od.pose.pose.position.x = t.x
        od.pose.pose.position.y = t.y
        od.pose.pose.position.z = t.z
        od.pose.pose.orientation = tf.transform.rotation
        if self._twist is not None:
            od.twist.twist = self._twist
        self._pub.publish(od)


def main(args=None):
    rclpy.init(args=args)
    node = MapOdomRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
