#!/usr/bin/env python3
"""rr_wall_aeb.py -- LiDAR emergency braking with reverse-and-replan recovery.

A pass-through /drive filter, independent of planner and controller:

    controller -- /drive_nav --> [rr_wall_aeb] --> /drive --> mux --> VESC

It reads the RAW /scan, so it stops for anything: walls, boxes, people. No
classification, no dependence on the map or on localization being correct.

WHY THIS IS NOT THE 2026-07-23 VERSION
--------------------------------------
That one tested a STRAIGHT box (half-width 0.20 m, 0.45 m ahead) and was
unusable in this corridor: on every curve the box pointed straight into the
outer wall, so it braked continuously and the car had to be turned by hand.

This version tests the swath the car will ACTUALLY sweep, following the arc
implied by the current steering angle. Driving down the middle of a 1.0-1.5 m
corridor, the walls sit outside that swath and nothing triggers; only an
obstacle genuinely on the car's path does.

RECOVERY
--------
On a trigger the node runs a state machine, ignoring controller commands until
it finishes:

  BRAKE    hold speed 0 for brake_hold_s, so the car settles before moving
  REVERSE  back up reverse_distance at reverse_speed, holding the SAME steering
           angle it went in with (backing along an arc retraces it)
  SETTLE   sit still for settle_s while nav2 replans from the new pose

Nav2 replans on its own while a goal is active, so the fresh path appears
without asking. After SETTLE the node returns to PASS and the controller drives
the new plan.

REVERSING IS PARTLY BLIND. The Hokuyo covers 270 deg (+-135 deg), so the rear
90 deg is unseen. The node checks the rear-most beams it does have and refuses
to reverse if they are blocked, but it CANNOT see directly behind. Reverse is
therefore short and slow, and capped: after max_recoveries in
recovery_window_s it stops recovering and just holds the brake, rather than
shuffling back and forth into something it cannot see.

DISABLE: run ~/rr/rr_aeb_off.sh (puts the controller back on /drive directly).
"""

import math

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

# How long the input may go quiet before this node stops trusting it. The
# controller publishes far faster than this, so 0.3 s only trips when the
# upstream has actually stopped.
CMD_TIMEOUT_S = 0.3


PASS, BRAKE, REVERSE, SETTLE, HOLD = 'PASS', 'BRAKE', 'REVERSE', 'SETTLE', 'HOLD'


def swath_distance(x, y, steering, half_width, wheelbase):
    """Along-path distance to the nearest obstacle inside the car's swath.

    Points (x, y) are in base_link. Rather than a straight box ahead, this
    tests the ARC the car will follow at this steering angle -- which is what
    stops a corridor wall on a curve from reading as an imminent collision.

    Returns inf when nothing lies on the swath ahead.
    """
    x = np.asarray(x, dtype=np.float64)
    y = np.asarray(y, dtype=np.float64)
    if x.size == 0:
        return float('inf')

    if abs(steering) < 1e-3:
        ahead = (x > 0.0) & (np.abs(y) < half_width)
        return float(np.min(x[ahead])) if ahead.any() else float('inf')

    radius = wheelbase / math.tan(steering)        # signed: + left, - right
    # Turn centre sits at (0, radius) in base_link. A point is on the swath if
    # its distance from that centre is within half_width of the turn radius.
    on_swath = np.abs(np.hypot(x, y - radius) - abs(radius)) < half_width
    # Angle swept from the car (at the origin) round to the point; positive is
    # ahead along the direction of travel for either turn direction.
    theta = np.arctan2(x, abs(radius) - np.sign(radius) * y)
    ahead = on_swath & (theta > 0.0)
    if not ahead.any():
        return float('inf')
    return float(np.min(abs(radius) * theta[ahead]))


class WallAEB(Node):
    def __init__(self):
        super().__init__('rr_wall_aeb')

        self.declare_parameter('drive_in_topic', '/drive_nav')
        self.declare_parameter('drive_out_topic', '/drive')
        self.declare_parameter('scan_topic', '/scan')

        # Danger swath. half_width is the car's half-width plus a margin: the
        # body is ~0.26 m wide, so 0.13 + 0.04.
        self.declare_parameter('half_width', 0.17)
        # Stop distance grows with speed: base + speed * headway.
        self.declare_parameter('stop_base', 0.30)
        self.declare_parameter('stop_headway', 0.25)
        self.declare_parameter('release_extra', 0.15)   # hysteresis margin
        self.declare_parameter('min_range', 0.06)       # ignore self-returns
        self.declare_parameter('scan_timeout', 0.4)
        # A single LiDAR beam is not enough evidence to stop a race car. Keep
        # only spatially continuous runs of adjacent beams, then require a
        # normal-distance hazard to persist across several control cycles.
        self.declare_parameter('min_cluster_points', 3)
        self.declare_parameter('cluster_neighbor_distance', 0.08)
        self.declare_parameter('hazard_confirm_cycles', 3)
        self.declare_parameter('immediate_stop_distance', 0.25)

        self.declare_parameter('wheelbase', 0.25)
        self.declare_parameter('brake_hold_s', 0.7)
        self.declare_parameter('reverse_speed', 0.3)
        self.declare_parameter('reverse_distance', 0.25)
        self.declare_parameter('reverse_timeout_s', 4.0)
        self.declare_parameter('rear_clear_distance', 0.30)
        self.declare_parameter('settle_s', 2.0)
        self.declare_parameter('max_recoveries', 3)
        self.declare_parameter('recovery_window_s', 40.0)
        # Brake-only for initial validation. The rear 90 degrees are outside
        # the Hokuyo field of view, so automatic reversing requires a separate
        # supervised validation before it can be enabled safely.
        self.declare_parameter('enable_recovery', False)

        g = lambda name: self.get_parameter(name).value  # noqa: E731
        self.half = g('half_width')
        self.stop_base = g('stop_base')
        self.stop_headway = g('stop_headway')
        self.release_extra = g('release_extra')
        self.min_r = g('min_range')
        self.scan_timeout = g('scan_timeout')
        self.min_cluster_points = max(1, int(g('min_cluster_points')))
        self.cluster_neighbor_distance = g('cluster_neighbor_distance')
        self.hazard_confirm_cycles = max(1, int(g('hazard_confirm_cycles')))
        self.immediate_stop_distance = g('immediate_stop_distance')
        self.wheelbase = g('wheelbase')
        self.brake_hold_s = g('brake_hold_s')
        self.reverse_speed = abs(g('reverse_speed'))
        self.reverse_distance = g('reverse_distance')
        self.reverse_timeout_s = g('reverse_timeout_s')
        self.rear_clear = g('rear_clear_distance')
        self.settle_s = g('settle_s')
        self.max_recoveries = g('max_recoveries')
        self.recovery_window_s = g('recovery_window_s')
        self.enable_recovery = g('enable_recovery')

        self.state = PASS
        self.state_since = self.now()
        self.last_cmd = None            # newest controller command
        self.last_cmd_time = 0.0        # when it arrived -- see tick()
        self.last_scan = None
        self.last_scan_time = 0.0
        self.scan_xy = None             # scan points in base_link
        self.range_ahead = float('inf')
        self.rear_min = float('inf')
        self.reverse_steer = 0.0
        self.reverse_origin = None      # odom position when reversing began
        self.position = None
        self.recoveries = []            # timestamps, for the rate cap
        self.hazard_count = 0

        self.create_subscription(LaserScan, g('scan_topic'), self.on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(AckermannDriveStamped, g('drive_in_topic'),
                                 self.on_drive, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.pub = self.create_publisher(AckermannDriveStamped,
                                         g('drive_out_topic'), 10)
        self.active_pub = self.create_publisher(Bool, '/perception/aeb_active', 10)
        self.state_pub = self.create_publisher(String, '/perception/aeb_state', 10)
        self.create_timer(0.05, self.tick)

        self.get_logger().info(
            'rr_wall_aeb: swath half=%.2fm stop=%.2f+%.2f*v release=+%.2f '
            'cluster=%d beams/%.2fm confirm=%d immediate=%.2fm recovery=%s '
            '(reverse %.2fm @ %.2fm/s, max %d per %.0fs)'
            % (self.half, self.stop_base, self.stop_headway, self.release_extra,
               self.min_cluster_points, self.cluster_neighbor_distance,
               self.hazard_confirm_cycles, self.immediate_stop_distance,
               self.enable_recovery, self.reverse_distance, self.reverse_speed,
               self.max_recoveries, self.recovery_window_s)
        )

    # ---------------------------------------------------------------- inputs
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_drive(self, msg):
        self.last_cmd = msg
        self.last_cmd_time = self.now()

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.position = (p.x, p.y)

    def on_scan(self, msg):
        self.last_scan = msg
        self.last_scan_time = self.now()
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        angles = msg.angle_min + np.arange(ranges.size) * msg.angle_increment
        good = np.isfinite(ranges) & (ranges > self.min_r)
        good &= ranges < min(msg.range_max, 12.0)
        beam_idx = np.flatnonzero(good)
        r, a = ranges[good], angles[good]
        # Laser sits 0.27 m ahead of base_link, no yaw offset.
        px, py = 0.27 + r * np.cos(a), r * np.sin(a)
        # Reject returns from the vehicle itself. The Hokuyo's extreme
        # +/-135-degree beams can strike the chassis; those points transform
        # behind the laser and inside the body footprint. Do not raise
        # min_range globally, because that would also hide a genuinely close
        # obstacle directly in front of the laser.
        outside_body = ~(
            (px > -0.20) & (px < 0.35) & (np.abs(py) < 0.17)
        )
        px, py = px[outside_body], py[outside_body]
        beam_idx = beam_idx[outside_body]

        # Retain complete runs of adjacent beams whose neighbouring Cartesian
        # returns are spatially consistent. Solid walls, columns, boxes and
        # people produce runs; isolated edge noise and one/two-beam chassis
        # glints do not. Preserve original beam indices so invalid gaps cannot
        # accidentally join unrelated returns into a cluster.
        if px.size and self.min_cluster_points > 1:
            joins = ((np.diff(beam_idx) == 1) &
                     (np.hypot(np.diff(px), np.diff(py)) <=
                      self.cluster_neighbor_distance))
            starts = np.empty(px.size, dtype=bool)
            starts[0] = True
            starts[1:] = ~joins
            labels = np.cumsum(starts) - 1
            counts = np.bincount(labels)
            clustered = counts[labels] >= self.min_cluster_points
            px, py = px[clustered], py[clustered]
        self.scan_xy = (px, py)
        # Rear clearance by GEOMETRY, not by beam angle: only points actually
        # behind the car and within its width count. Measuring by angle counts
        # the side walls of a narrow corridor as "rear blocked" (seen parked:
        # 0.32 m, which was a wall beside the car, not behind it).
        #
        # The Hokuyo spans 270 deg, so points behind exist only out to roughly
        # 0.35 m within this band -- directly astern is never visible. That is
        # why reverse_distance stays inside what can actually be checked.
        behind = (px < -0.05) & (np.abs(py) < 0.25)
        self.rear_min = float(np.min(-px[behind])) if behind.any() else float('inf')

    # ------------------------------------------------------------- geometry
    def distance_along_path(self, steering):
        if self.scan_xy is None:
            return float('inf')
        x, y = self.scan_xy
        return swath_distance(x, y, steering, self.half, self.wheelbase)

    # ------------------------------------------------------------ machinery
    def set_state(self, state, reason=''):
        if state != self.state:
            self.get_logger().warn('AEB %s -> %s %s' % (self.state, state, reason))
            self.state = state
            self.state_since = self.now()

    def publish(self, speed, steering):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steering)
        self.pub.publish(msg)

    def recoveries_recent(self):
        cutoff = self.now() - self.recovery_window_s
        self.recoveries = [t for t in self.recoveries if t > cutoff]
        return len(self.recoveries)

    def travelled_since_reverse(self):
        if self.reverse_origin is None or self.position is None:
            return 0.0
        return math.hypot(self.position[0] - self.reverse_origin[0],
                          self.position[1] - self.reverse_origin[1])

    def tick(self):
        now = self.now()

        # SAFETY: this node OWNS /drive and ticks at 20 Hz, so whatever it
        # publishes is what the car does. Without this check it re-published
        # last_cmd forever, and when the controller was killed mid-run the car
        # kept driving at its final speed with nothing upstream commanding it.
        # The joystick deadman does not save you: it is a mux override with a
        # 0.2 s timeout, so releasing it hands control straight back to the
        # stale command. A pass-through must fail to ZERO, not to its last
        # input.
        if self.last_cmd is None or now - self.last_cmd_time > CMD_TIMEOUT_S:
            if self.last_cmd is not None:
                self.get_logger().warn(
                    "no command on the input for %.2fs -> zeroing (last was "
                    "%.2f m/s). Upstream controller is gone."
                    % (now - self.last_cmd_time, self.last_cmd.drive.speed))
                self.last_cmd = None
            self.publish(0.0, 0.0)
            self.active_pub.publish(Bool(data=False))
            self.state_pub.publish(String(data=self.state + " (no command)"))
            return

        cmd_speed = self.last_cmd.drive.speed if self.last_cmd else 0.0
        cmd_steer = self.last_cmd.drive.steering_angle if self.last_cmd else 0.0

        # No fresh scan: pass through rather than brake blind, and say so.
        if self.last_scan is None or now - self.last_scan_time > self.scan_timeout:
            if self.state in (BRAKE, REVERSE, SETTLE):
                self.publish(0.0, 0.0)
            elif self.last_cmd is not None:
                self.pub.publish(self.last_cmd)
            self.active_pub.publish(Bool(data=self.state != PASS))
            self.state_pub.publish(String(data=self.state + ' (no scan)'))
            return

        # Only look where the car is actually going: forward commands are
        # checked against the forward swath. While reversing we hold our own
        # steering angle.
        steer_for_check = cmd_steer if self.state == PASS else self.reverse_steer
        self.range_ahead = self.distance_along_path(steer_for_check)
        stop_d = self.stop_base + self.stop_headway * max(0.0, cmd_speed)
        release_d = stop_d + self.release_extra

        if self.state == PASS:
            if cmd_speed > 0.0 and self.range_ahead < stop_d:
                self.hazard_count += 1
                confirmed = self.hazard_count >= self.hazard_confirm_cycles
                immediate = self.range_ahead <= self.immediate_stop_distance
                if confirmed or immediate:
                    self.reverse_steer = cmd_steer
                    self.set_state(
                        BRAKE,
                        'clustered obstacle at %.2fm (limit %.2fm, cycles %d)'
                        % (self.range_ahead, stop_d, self.hazard_count))
                    self.publish(0.0, cmd_steer)
                else:
                    self.pub.publish(self.last_cmd)
            elif self.last_cmd is not None:
                self.hazard_count = 0
                self.pub.publish(self.last_cmd)
            else:
                self.hazard_count = 0
                self.publish(0.0, 0.0)

        elif self.state == BRAKE:
            self.publish(0.0, self.reverse_steer)
            if now - self.state_since >= self.brake_hold_s:
                if not self.enable_recovery:
                    self.set_state(HOLD, '(recovery disabled)')
                elif self.range_ahead >= release_d:
                    self.set_state(PASS, 'path cleared on its own')
                elif self.recoveries_recent() >= self.max_recoveries:
                    self.set_state(HOLD, '- %d recoveries in %.0fs, refusing '
                                   'to keep shuffling; take over with the '
                                   'gamepad' % (self.recoveries_recent(),
                                                self.recovery_window_s))
                elif self.rear_min < self.rear_clear:
                    self.set_state(HOLD, '- rear blocked at %.2fm, will not '
                                   'reverse' % self.rear_min)
                else:
                    self.recoveries.append(now)
                    self.reverse_origin = self.position
                    self.set_state(REVERSE, 'backing %.2fm'
                                   % self.reverse_distance)

        elif self.state == REVERSE:
            travelled = self.travelled_since_reverse()
            timed_out = now - self.state_since > self.reverse_timeout_s
            if self.rear_min < self.rear_clear:
                self.publish(0.0, 0.0)
                self.set_state(HOLD, '- rear became blocked at %.2fm'
                               % self.rear_min)
            elif travelled >= self.reverse_distance or timed_out:
                self.publish(0.0, 0.0)
                self.set_state(SETTLE, '- backed %.2fm%s'
                               % (travelled, ' (timeout)' if timed_out else ''))
            else:
                # Same steering as the approach: backing along an arc retraces
                # it, which puts the car where it can plan a new way round.
                self.publish(-self.reverse_speed, self.reverse_steer)

        elif self.state == SETTLE:
            self.publish(0.0, 0.0)
            if now - self.state_since >= self.settle_s:
                # nav2 replans continuously while the goal is active, so a
                # fresh path is already on the way.
                self.set_state(PASS, '- handing back to the controller')

        elif self.state == HOLD:
            self.publish(0.0, 0.0)
            if self.range_ahead >= release_d and self.recoveries_recent() == 0:
                self.set_state(PASS, 'clear again')

        self.active_pub.publish(Bool(data=self.state != PASS))
        self.state_pub.publish(
            String(data='%s ahead=%.2f rear=%.2f' %
                   (self.state, self.range_ahead, self.rear_min))
        )


def main():
    rclpy.init()
    node = WallAEB()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
