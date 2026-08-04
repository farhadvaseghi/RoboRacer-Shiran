#!/usr/bin/env python3
"""Cancel every active nav2 goal and stop the controller. Emergency use.

WHY THIS IS NEEDED
------------------
A /initialpose (Foxglove '2D Pose Estimate') re-localizes slam and clears the
costmaps, but it does NOT cancel the running nav2 action goal. bt_navigator
keeps its ACTIVE goal and keeps driving the old plan from the pose that was
just replaced -- so the car carries on toward a goal you thought you had called
off. The joystick deadman does not fix it either: it is a mux override with a
0.2 s timeout, so the car resumes as soon as the button is released.

WHAT IT DOES
------------
  1. Cancels ALL goals on BOTH action servers. navigate_through_poses matters
     as much as navigate_to_pose -- the waypoint mission uses it, and
     cancelling only one leaves the other driving.
  2. Publishes an EMPTY path to /plan and /control/plan. The custom
     pure_pursuit sets have_path_=false on an empty path, which is what
     actually stops the wheels.

An empty CancelGoal request (blank goal_id, zero stamp) means "cancel
everything" to a ROS 2 action server.

The empty path is republished several times: a single one races bt_navigator's
next tick and the old path pops straight back.

Usage:  ROS_DOMAIN_ID=7 python3 ~/rr/rr_cancel_goals.py
"""

import os
import sys
import time

os.environ.setdefault("ROS_DOMAIN_ID", "7")

import rclpy
from action_msgs.srv import CancelGoal
from nav_msgs.msg import Path
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

ACTIONS = ["/navigate_to_pose", "/navigate_through_poses"]

# The controller's path subscription demands RELIABLE + TRANSIENT_LOCAL, and
# the live path is latched -- a VOLATILE publisher cannot overwrite it.
LATCHED = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
)

REPEATS = 8
REPEAT_GAP_S = 0.25


def main():
    rclpy.init()
    node = rclpy.create_node("rr_cancel_goals")

    plan_pub = node.create_publisher(Path, "/plan", LATCHED)
    ctrl_pub = node.create_publisher(Path, "/control/plan", LATCHED)

    cancelled_any = False
    for action in ACTIONS:
        srv = action + "/_action/cancel_goal"
        client = node.create_client(CancelGoal, srv)
        if not client.wait_for_service(timeout_sec=3.0):
            print("  %-28s no cancel service (server not running)" % action)
            continue
        future = client.call_async(CancelGoal.Request())  # blank = cancel all
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        if future.done() and future.result() is not None:
            n = len(future.result().goals_canceling)
            print("  %-28s cancelled %d goal(s)" % (action, n))
            cancelled_any = cancelled_any or n > 0
        else:
            print("  %-28s cancel request timed out" % action)

    print("publishing an empty path %d times to stop the controller ..."
          % REPEATS)
    for _ in range(REPEATS):
        empty = Path()
        empty.header.frame_id = "map"
        empty.header.stamp = node.get_clock().now().to_msg()
        plan_pub.publish(empty)
        ctrl_pub.publish(empty)
        rclpy.spin_once(node, timeout_sec=0.01)
        time.sleep(REPEAT_GAP_S)

    print("")
    if cancelled_any:
        print("DONE: an active goal was cancelled and the path emptied.")
    else:
        print("DONE: no goal was active. The empty path was still published,")
        print("      so if the car was moving it was following a latched path")
        print("      rather than a live goal.")
    print("If it still creeps, the controller itself is the thing to stop:")
    print("  pkill -f 'pure_pursuit_controlle[r]'")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
