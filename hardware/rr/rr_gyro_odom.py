#!/usr/bin/env python3
"""Publish odom->base_link using the VESC gyro for heading instead of the
steering command.

WHY
---
vesc_to_odom integrates heading from the COMMANDED servo position
(vesc_to_odom.cpp:105-127):

    yaw += (speed * tan(steering_COMMAND) / wheelbase) * dt

Nothing measures that. It assumes the servo reaches the commanded angle
instantly and the tyres never slip. Measured on 2026-08-01 with
rr_odom_ruler.py, driving a loop back onto floor marks so the true answer is
zero: 12.7 m of position error and ~250 deg of heading error over a 39 m lap.
No scan matcher can work from a prior that wrong.

The VESC has a real gyro (44 Hz on /sensors/imu/raw). Substituting it for the
fabricated yaw leaves a bias that is constant and measurable, and noise that
random-walks to well under a degree per lap.

WHAT THIS DOES
--------------
  heading   integrated from the measured gyro yaw rate, bias removed
  distance  from the wheels, via /odom's twist -- the LiDAR cannot observe
            position ALONG the corridor (measured: 0.9 m blind zone), so the
            wheels remain the only source for that axis
  output    TF odom->base_link, plus /odom_gyro for inspection

Set 'publish_tf: false' for vesc_to_odom_node before running this, or two
publishers will fight over the same transform.

UNITS TRAP
----------
vesc_driver copies raw VESC packet fields into sensor_msgs/Imu with NO
conversion (vesc_driver.cpp:234-240): angular_velocity is in DEG/S and
linear_acceleration is in g, despite the message type meaning rad/s and m/s^2.
Verified at rest: |accel| reads 1.070, i.e. 1 g. This node converts.

The VESC also publishes its own AHRS quaternion in 'orientation'. Do not use
it: measured drifting 10.4 deg in 20 s while standing still, because it is
just integrating this same biased gyro with nothing correcting it.

Usage:  ROS_DOMAIN_ID=7 python3 ~/rr/rr_gyro_odom.py
"""

import math
import os
import sys

os.environ.setdefault("ROS_DOMAIN_ID", "7")

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster

DEG_TO_RAD = math.pi / 180.0

# Samples are only used for bias estimation while the wheels report less than
# this. Anything faster means the car was pushed or driven and the average
# would absorb real rotation.
STILL_SPEED_MPS = 0.02

# A dt larger than this means messages were dropped or the clock jumped;
# integrating across it would inject a large bogus rotation.
MAX_DT_S = 0.5

# ZERO-VELOCITY UPDATE. An Ackermann car cannot rotate without its wheels
# turning, so any yaw rate reported while the wheels are stopped is bias by
# definition. Without this the residual bias integrates into heading forever
# while the car is parked: measured 2026-08-01 as 8.5 deg of pose yaw error
# accumulated between two standstill checks, enough to drop the scan fit from
# 90% to 39% on-wall without the car having moved at all.
ZUPT_HOLD_S = 0.5      # how long stopped before the reading is trusted as bias
BIAS_TAU_S = 30.0      # time constant for re-learning bias while stopped
BIAS_LOG_PERIOD_S = 60.0


class GyroOdom(Node):
    def __init__(self):
        super().__init__("rr_gyro_odom")

        self.declare_parameter("imu_topic", "/sensors/imu/raw")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("output_topic", "/odom_gyro")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("calibration_seconds", 4.0)
        self.declare_parameter("publish_tf", True)
        # Set false only to measure the gyro against the old behaviour without
        # taking over the transform.
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.calib_seconds = float(self.get_parameter("calibration_seconds").value)
        self.want_tf = bool(self.get_parameter("publish_tf").value)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.speed = 0.0
        self.last_stamp = None

        self.bias = 0.0
        self.still_time = 0.0
        self.zupt_active = False
        self.last_bias_log = None
        self.calibrated = False
        self.calib_samples = []
        self.calib_start = None
        self.calib_rejected = 0

        sensor_qos = QoSProfile(
            depth=100,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        self.create_subscription(Odometry,
                                 self.get_parameter("odom_topic").value,
                                 self._on_odom, 20)
        self.create_subscription(Imu, self.get_parameter("imu_topic").value,
                                 self._on_imu, sensor_qos)
        self.pub = self.create_publisher(
            Odometry, self.get_parameter("output_topic").value, 20)
        self.tf_broadcaster = TransformBroadcaster(self) if self.want_tf else None

        self.get_logger().info(
            "gyro odom starting: heading from %s (deg/s -> rad/s), speed from "
            "%s, publishing %s%s"
            % (self.get_parameter("imu_topic").value,
               self.get_parameter("odom_topic").value,
               self.get_parameter("output_topic").value,
               " + TF %s->%s" % (self.odom_frame, self.base_frame)
               if self.want_tf else " (NO TF)"))
        self.get_logger().info(
            "hold the car STILL for %.0f s while the gyro bias is measured"
            % self.calib_seconds)

    def _on_odom(self, msg):
        # Only the velocity is taken. This message's POSE is the broken
        # integration this node exists to replace.
        self.speed = msg.twist.twist.linear.x

    def _stamp_seconds(self, stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def _on_imu(self, msg):
        now = self._stamp_seconds(msg.header.stamp)
        rate = msg.angular_velocity.z * DEG_TO_RAD

        if not self.calibrated:
            self._calibrate(now, rate)
            self.last_stamp = now
            return

        if self.last_stamp is None:
            self.last_stamp = now
            return
        dt = now - self.last_stamp
        self.last_stamp = now
        if dt <= 0.0 or dt > MAX_DT_S:
            if dt > MAX_DT_S:
                self.get_logger().warn(
                    "skipped a %.2f s gap in the IMU stream; heading is "
                    "unchanged across it" % dt)
            return

        # Zero-velocity update: while the wheels are stopped, hold the heading
        # and fold the reading back into the bias estimate, so bias drift
        # (temperature, mostly) is tracked instead of accumulating into yaw.
        if abs(self.speed) < STILL_SPEED_MPS:
            self.still_time += dt
        else:
            self.still_time = 0.0

        if self.still_time >= ZUPT_HOLD_S:
            self.bias += (rate - self.bias) * min(dt / BIAS_TAU_S, 1.0)
            omega = 0.0
            if not self.zupt_active:
                self.zupt_active = True
                self.get_logger().info(
                    "stopped -- holding heading, re-learning bias (now %+.4f "
                    "deg/s)" % (self.bias / DEG_TO_RAD))
        else:
            omega = rate - self.bias
            if self.zupt_active:
                self.zupt_active = False
                self.get_logger().info(
                    "moving -- integrating heading again (bias %+.4f deg/s)"
                    % (self.bias / DEG_TO_RAD))

        if self.last_bias_log is None:
            self.last_bias_log = now
        elif now - self.last_bias_log >= BIAS_LOG_PERIOD_S:
            self.last_bias_log = now
            self.get_logger().info("bias now %+.4f deg/s%s"
                                   % (self.bias / DEG_TO_RAD,
                                      " (stopped)" if self.zupt_active else ""))

        # Midpoint heading over the step, so a turn does not bias the position
        # toward the heading at the start of the interval.
        mid_yaw = self.yaw + 0.5 * omega * dt
        self.x += self.speed * math.cos(mid_yaw) * dt
        self.y += self.speed * math.sin(mid_yaw) * dt
        self.yaw = math.atan2(math.sin(self.yaw + omega * dt),
                              math.cos(self.yaw + omega * dt))

        self._publish(msg.header.stamp, omega)

    def _calibrate(self, now, rate):
        if self.calib_start is None:
            self.calib_start = now
            return
        if abs(self.speed) > STILL_SPEED_MPS:
            # Moving: throw the whole batch away rather than average real
            # rotation into the bias, and start the window again.
            if self.calib_samples:
                self.calib_rejected += len(self.calib_samples)
                self.calib_samples = []
            self.calib_start = now
            return

        self.calib_samples.append(rate)
        if now - self.calib_start < self.calib_seconds:
            return
        if len(self.calib_samples) < 20:
            self.calib_start = now
            self.calib_samples = []
            return

        self.bias = sum(self.calib_samples) / len(self.calib_samples)
        spread = max(self.calib_samples) - min(self.calib_samples)
        self.calibrated = True
        self.get_logger().info(
            "gyro bias = %+.5f rad/s (%+.4f deg/s) from %d samples, spread "
            "%.4f deg/s%s"
            % (self.bias, self.bias / DEG_TO_RAD, len(self.calib_samples),
               spread / DEG_TO_RAD,
               "" if not self.calib_rejected
               else "; %d samples rejected for motion" % self.calib_rejected))
        self.get_logger().info("integrating now -- the car may be driven")

    def _publish(self, stamp, omega):
        qz = math.sin(self.yaw * 0.5)
        qw = math.cos(self.yaw * 0.5)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf)

        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = self.odom_frame
        od.child_frame_id = self.base_frame
        od.pose.pose.position.x = self.x
        od.pose.pose.position.y = self.y
        od.pose.pose.orientation.z = qz
        od.pose.pose.orientation.w = qw
        od.twist.twist.linear.x = self.speed
        od.twist.twist.angular.z = omega
        # Heading is now measured rather than assumed, so it is trusted far
        # more than the wheel-derived position.
        od.pose.covariance[0] = 0.05
        od.pose.covariance[7] = 0.05
        od.pose.covariance[35] = 0.01
        od.twist.covariance[0] = 0.05
        od.twist.covariance[35] = 0.01
        self.pub.publish(od)


def main():
    rclpy.init()
    node = GyroOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
