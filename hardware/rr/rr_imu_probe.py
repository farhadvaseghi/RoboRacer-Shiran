#!/usr/bin/env python3
"""Is the VESC's onboard IMU actually producing usable data?

The fix for the fabricated heading is to integrate a real gyro instead of the
steering command. vesc_driver already advertises sensors/imu and
sensors/imu/raw -- but advertising is not publishing, and some VESC hardware
and firmware builds have no IMU populated at all. This settles it before any
work is built on top.

Checks, with the car PARKED AND STILL:
  - does anything arrive, and at what rate
  - is the gyro's yaw rate near zero at rest, and how much does it wobble
    (that wobble is the noise a heading integration would accumulate)
  - does the accelerometer see ~9.8 m/s^2 of gravity, which proves the numbers
    are real engineering units and not an unpopulated struct of zeros

Read-only.

Usage:  ROS_DOMAIN_ID=7 python3 ~/rr/rr_imu_probe.py [seconds]
"""

import math
import os
import sys
import time

os.environ.setdefault("ROS_DOMAIN_ID", "7")

import numpy as np
import rclpy
from rclpy.qos import (QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy)
from sensor_msgs.msg import Imu

# vesc_driver copies raw VESC packet fields straight into sensor_msgs/Imu with
# NO unit conversion (vesc_driver.cpp:234-240). VESC firmware reports
# acceleration in g and angular rate in deg/s, while the message type is
# defined as m/s^2 and rad/s. Anything consuming this topic must convert, or
# it will read a gyro ~57x too small and gravity ~9.8x too small.
G_TO_MS2 = 9.80665

BEST_EFFORT = QoSProfile(
    depth=50,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
)
RELIABLE = QoSProfile(
    depth=50,
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
)

TOPIC = "/sensors/imu/raw"


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0
    rclpy.init()
    node = rclpy.create_node("rr_imu_probe")
    msgs = []
    # Subscribe twice: a best-effort sub matches any publisher, but a reliable
    # publisher paired with only a best-effort sub is still worth confirming.
    node.create_subscription(Imu, TOPIC, msgs.append, BEST_EFFORT)
    node.create_subscription(Imu, TOPIC, lambda _m: None, RELIABLE)

    print("listening to %s for %.0fs -- KEEP THE CAR STILL\n" % (TOPIC, seconds))
    start = time.monotonic()
    while rclpy.ok() and time.monotonic() - start < seconds:
        rclpy.spin_once(node, timeout_sec=0.1)

    pubs = node.count_publishers(TOPIC)
    node.destroy_node()
    rclpy.shutdown()

    print("publishers on %s : %d" % (TOPIC, pubs))
    print("messages received  : %d in %.0fs" % (len(msgs), seconds))

    if not msgs:
        print("")
        print("VERDICT: NO IMU DATA.")
        if pubs == 0:
            print("  Nothing is publishing the topic at all. vesc_driver only")
            print("  creates the publisher, so the node may be down.")
        else:
            print("  The topic is advertised but silent -- the VESC is not")
            print("  returning IMU packets. Either this VESC has no IMU")
            print("  populated, or the firmware does not stream it.")
        print("  Then the gyro fix needs a SEPARATE IMU, which is a hardware")
        print("  change, not a config change.")
        return 1

    rate = len(msgs) / seconds
    gz = np.array([m.angular_velocity.z for m in msgs])
    gx = np.array([m.angular_velocity.x for m in msgs])
    gy = np.array([m.angular_velocity.y for m in msgs])
    ax = np.array([m.linear_acceleration.x for m in msgs])
    ay = np.array([m.linear_acceleration.y for m in msgs])
    az = np.array([m.linear_acceleration.z for m in msgs])
    gravity = np.sqrt(ax ** 2 + ay ** 2 + az ** 2)

    qs = np.array([[m.orientation.w, m.orientation.x, m.orientation.y,
                    m.orientation.z] for m in msgs])
    q_populated = bool(np.abs(qs).sum() > 0)

    print("rate               : %.1f Hz" % rate)
    print("NOTE: raw fields are VESC units (g and deg/s), NOT the m/s^2 and")
    print("      rad/s the message type claims -- see the driver source.")
    print("")
    print("GYRO (deg/s, at rest)")
    print("  yaw rate z : bias %+.4f  noise sd %.4f  min %+.3f  max %+.3f"
          % (gz.mean(), gz.std(), gz.min(), gz.max()))
    print("  roll  x    : bias %+.4f  noise sd %.4f" % (gx.mean(), gx.std()))
    print("  pitch y    : bias %+.4f  noise sd %.4f" % (gy.mean(), gy.std()))
    print("")
    print("ACCELEROMETER (g, at rest)")
    print("  magnitude  : mean %.3f g = %.2f m/s^2   (should be ~1.0 g)"
          % (gravity.mean(), gravity.mean() * G_TO_MS2))
    print("")
    print("VESC AHRS ORIENTATION")
    if q_populated:
        yaws = np.degrees(np.arctan2(
            2 * (qs[:, 0] * qs[:, 3] + qs[:, 1] * qs[:, 2]),
            1 - 2 * (qs[:, 2] ** 2 + qs[:, 3] ** 2)))
        print("  populated: yaw mean %+.2f deg, drift over this window %+.2f deg"
              % (yaws.mean(), yaws[-1] - yaws[0]))
    else:
        print("  all zeros -- the VESC is not running its AHRS fusion, so only")
        print("  the raw gyro is available (which is all we need).")

    # A constant bias integrates straight into heading error, so quantify what
    # it would cost over a lap-length drive.
    drift_deg_per_min = abs(gz.mean()) * 60.0
    print("")
    print("WHAT THIS MEANS FOR HEADING")
    print("  uncorrected bias drift : %.1f deg/min" % drift_deg_per_min)
    print("  after removing the measured bias, what is left is the noise,")
    print("  %.4f deg/s, which random-walks rather than accumulating." % gz.std())
    print("  Compare: the steering-command yaw is ~250 deg wrong per lap.")

    all_zero = (gz.std() == 0.0 and gravity.mean() == 0.0)
    if all_zero:
        print("")
        print("VERDICT: DATA IS ALL ZEROS -- the struct is present but never")
        print("         filled. Treat this as NO IMU.")
        return 1
    if not 0.7 <= gravity.mean() <= 1.3:
        print("")
        print("VERDICT: PUBLISHING, BUT THE SCALE IS WRONG EVEN AS g.")
        print("         Gravity should read ~1.0 g; packet decoding needs")
        print("         checking before the gyro can be trusted.")
        return 0
    print("")
    print("VERDICT: USABLE GYRO (in deg/s -- convert before use).")
    print("  A real measured yaw rate exists. Its bias is constant and can be")
    print("  subtracted; what remains is bounded noise that slam can correct.")
    print("  The steering-command yaw it would replace is ~250 deg wrong per")
    print("  lap, which nothing can correct.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
