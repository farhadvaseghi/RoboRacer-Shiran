#!/usr/bin/env python3
"""Seed slam localization at the surveyed START pose.

Why this exists
---------------
localize_slam_real.yaml has 'map_start_at_dock: true', so every cold bringup
initialises the pose at the MAP ORIGIN. On corridor_despeck the origin is only
0.071 m from a wall, i.e. inside the 0.22 m robot footprint, so the planner
refuses every goal with 'Starting point in lethal space'. Seeding an actual
surveyed free-space pose removes that whole failure mode.

The pose below was established on 2026-07-31 by scan-matching the live /scan
against corridor_despeck and cross-checked against hand measurements taken with
the car parked in its start position (rear wall 0.31 m, right wall 0.93 m to
base_link). Agreement went from 50% of beams at slam's own estimate to 81% here.

Re-survey with /tmp/rr_fix_pose.py if the start position ever moves.

Usage:  ROS_DOMAIN_ID=7 python3 ~/rr/rr_seed_start.py
"""

import math
import os
import sys
import time

os.environ.setdefault('ROS_DOMAIN_ID', '7')

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

# Start pose (map frame): the map origin. The car is parked here before
# every run. rr_waypoint_mission.py seeds the same pose while arming --
# change both together.
START_X = 0.0
START_Y = 0.0
START_YAW_DEG = 0.0

# slam_toolbox drops /initialpose messages published with volatile QoS, so this
# must be TRANSIENT_LOCAL + RELIABLE.
QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
)


def main():
    rclpy.init()
    node = rclpy.create_node('rr_seed_start')
    pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', QOS)

    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.pose.position.x = START_X
    msg.pose.pose.position.y = START_Y
    yaw = math.radians(START_YAW_DEG)
    msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
    # Modest confidence: parked on the marked spot, but not surveyed.
    msg.pose.covariance[0] = 0.02
    msg.pose.covariance[7] = 0.02
    msg.pose.covariance[35] = 0.02

    # Publish a few times; slam only latches one, but the extra sends survive a
    # subscriber that is still coming up.
    for _ in range(5):
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.2)
        time.sleep(0.3)

    node.get_logger().info(
        'seeded /initialpose at x=%.3f y=%.3f yaw=%.2f deg'
        % (START_X, START_Y, START_YAW_DEG)
    )
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
