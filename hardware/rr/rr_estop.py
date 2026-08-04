#!/usr/bin/env python3
"""rr_estop.py — software EMERGENCY STOP for the car (Foxglove button).

A manual pass-through /drive filter you toggle from Foxglove:

    controller ── /drive_raw ──► [rr_estop] ──► /drive ──► mux ──► VESC

Publish std_msgs/Bool on /estop:
    data: true   -> STOP : forces speed 0 at 20 Hz and HOLDS it until released
    data: false  -> RUN  : passes the controller through unchanged

Fail-safe: if this node dies, nothing forwards /drive -> the car stops.
Steering is zeroed while stopped. The gamepad LB (mux priority 100) still
overrides everything as the independent hardware backup.
"""
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool


class EStop(Node):
    def __init__(self):
        super().__init__('rr_estop')
        self.declare_parameter('drive_in_topic', '/drive_raw')
        self.declare_parameter('drive_out_topic', '/drive')
        self.declare_parameter('estop_topic', '/estop')
        self.stopped = False
        self.pub = self.create_publisher(
            AckermannDriveStamped, self.get_parameter('drive_out_topic').value, 10)
        self.create_subscription(
            AckermannDriveStamped, self.get_parameter('drive_in_topic').value, self.drive_cb, 10)
        self.create_subscription(
            Bool, self.get_parameter('estop_topic').value, self.estop_cb, 10)
        self.create_timer(0.05, self.heartbeat)   # 20 Hz hold
        self.get_logger().info(
            'rr_estop: RUN (pass-through). Publish Bool data:true on %s to STOP.'
            % self.get_parameter('estop_topic').value)

    def estop_cb(self, m):
        if m.data and not self.stopped:
            self.stopped = True
            self.get_logger().warn('E-STOP ENGAGED -> forcing /drive = 0 (publish data:false to release)')
        elif not m.data and self.stopped:
            self.stopped = False
            self.get_logger().info('E-stop RELEASED -> RUN (passing controller through)')

    def drive_cb(self, msg):
        if self.stopped:
            self._zero(msg.header)
        else:
            self.pub.publish(msg)

    def heartbeat(self):
        if self.stopped:
            self._zero(None)

    def _zero(self, header):
        out = AckermannDriveStamped()
        if header is not None:
            out.header = header
        else:
            out.header.stamp = self.get_clock().now().to_msg()
        out.drive.speed = 0.0
        out.drive.steering_angle = 0.0
        self.pub.publish(out)


def main():
    rclpy.init()
    try:
        rclpy.spin(EStop())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
