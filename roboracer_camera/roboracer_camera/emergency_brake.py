#!/usr/bin/env python3
"""emergency_brake — dedicated AEB (Automatic Emergency Braking), our side only.

A safety reflex that is INDEPENDENT of the main controller, so it still stops the
car if navigation/control misbehaves. It is a pass-through drive filter:

    controller ── /drive_nav ──► [emergency_brake] ──► /drive ──► mux/sim

Normally it republishes the incoming drive command unchanged. When a person is
inside the forward danger corridor it forces speed to 0 (steering held) until the
person clears (with hysteresis to avoid chatter). It also raises
`/perception/aeb_active` (Bool) for logging/RViz.

Insertion is a launch-level remap: the controller's `/drive` is remapped to
`/drive_nav`, and this node owns the real `/drive`. No teammate code changes.
The joystick still overrides everything (mux priority 100), so a human can always
take over — AEB does not fight the driver.

Subscribes: /drive_nav (ackermann), /perception/persons (PoseArray, base_link)
Publishes:  /drive (ackermann), /perception/aeb_active (Bool)
"""

import math

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Bool


def person_in_zone(px, py, stop_dist, half_width, min_forward):
    """True if a person at (px,py) base_link is in the forward danger corridor."""
    return (min_forward <= px <= stop_dist) and (abs(py) <= half_width)


def nearest_forward_person(persons, half_width, min_forward, max_dist):
    """Nearest forward distance among persons inside the corridor, else +inf.

    persons: iterable of (px, py). Pure function — unit-testable.
    """
    best = math.inf
    for px, py in persons:
        if min_forward <= px <= max_dist and abs(py) <= half_width:
            best = min(best, px)
    return best


class EmergencyBrake(Node):
    def __init__(self):
        super().__init__('emergency_brake')
        self.declare_parameter('drive_in_topic', '/drive_nav')
        self.declare_parameter('drive_out_topic', '/drive')
        self.declare_parameter('persons_topic', '/perception/persons')
        self.declare_parameter('stop_distance', 1.5)     # brake if closer (m)
        self.declare_parameter('release_distance', 1.9)  # clear only beyond (m)
        self.declare_parameter('half_width', 0.35)       # corridor half-width (m)
        self.declare_parameter('min_forward', 0.0)
        self.declare_parameter('persons_timeout', 0.5)   # stale persons = no data

        self._half = self.get_parameter('half_width').value
        self._minf = self.get_parameter('min_forward').value
        self._stop = self.get_parameter('stop_distance').value
        self._release = self.get_parameter('release_distance').value
        self._timeout = self.get_parameter('persons_timeout').value

        self._braking = False
        self._nearest = math.inf
        self._last_persons = self.get_clock().now()

        self.create_subscription(PoseArray, self.get_parameter('persons_topic').value,
                                 self._persons_cb, 10)
        self.create_subscription(
            AckermannDriveStamped, self.get_parameter('drive_in_topic').value,
            self._drive_cb, 10)
        self._pub = self.create_publisher(
            AckermannDriveStamped, self.get_parameter('drive_out_topic').value, 10)
        self._apub = self.create_publisher(Bool, '/perception/aeb_active', 10)
        # Safety heartbeat: even if the controller stops publishing, keep asserting
        # a stop while braking so the car cannot creep on a stale command.
        self.create_timer(0.05, self._heartbeat)
        self.get_logger().info(
            'emergency_brake started: stop<%.2fm release>%.2fm halfwidth=%.2fm'
            % (self._stop, self._release, self._half))

    def _persons_cb(self, msg):
        persons = [(p.position.x, p.position.y) for p in msg.poses]
        self._nearest = nearest_forward_person(
            persons, self._half, self._minf, self._release)
        self._last_persons = self.get_clock().now()
        self._update_state()

    def _update_state(self):
        # Hysteresis: engage under stop_distance, release only beyond release_distance.
        if not self._braking and self._nearest <= self._stop:
            self._braking = True
            self.get_logger().warn('AEB ENGAGED — person at %.2f m' % self._nearest)
        elif self._braking and self._nearest > self._release:
            self._braking = False
            self.get_logger().info('AEB released — corridor clear')
        a = Bool(); a.data = self._braking
        self._apub.publish(a)

    def _persons_fresh(self):
        dt = (self.get_clock().now() - self._last_persons).nanoseconds * 1e-9
        return dt <= self._timeout

    def _drive_cb(self, msg):
        # Pass through, or zero speed if braking.
        out = msg
        if self._braking and self._persons_fresh():
            out = AckermannDriveStamped()
            out.header = msg.header
            out.drive.steering_angle = msg.drive.steering_angle
            out.drive.speed = 0.0
        self._pub.publish(out)

    def _heartbeat(self):
        # If persons went stale, don't hold a brake on old data — let the normal
        # command flow (and the mux fail-safe) take over.
        if self._braking and not self._persons_fresh():
            self._braking = False
            self._update_state()
        if self._braking:
            stop = AckermannDriveStamped()
            stop.header.stamp = self.get_clock().now().to_msg()
            stop.drive.speed = 0.0
            self._pub.publish(stop)


def main(args=None):
    rclpy.init(args=args)
    node = EmergencyBrake()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
