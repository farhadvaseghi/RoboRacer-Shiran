#!/usr/bin/env python3
"""Re-publish the latched /map so the costmap static layer re-marks itself.

nav2 "clear entirely" resets the global costmap master grid to NO_INFORMATION.
The static layer only widens its update bounds when a NEW map message arrives,
so after a clear the saved map never comes back and the whole costmap stays
grey (100% unknown) -- the planner then has nothing to plan on.

map_server publishes /map once, at activation, with TRANSIENT_LOCAL durability.
Reading that latched message and publishing it again is enough to make the
static layer re-apply the map on the next costmap update.
"""

import os
import sys

os.environ.setdefault("ROS_DOMAIN_ID", "7")

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

LATCHED = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
)


def main():
    rclpy.init()
    node = rclpy.create_node("rr_map_republish")
    received = []
    node.create_subscription(OccupancyGrid, "/map", received.append, LATCHED)

    deadline = node.get_clock().now().nanoseconds + 10e9
    while rclpy.ok() and not received:
        if node.get_clock().now().nanoseconds > deadline:
            node.get_logger().error("no latched /map within 10s; is map_clean_server up?")
            node.destroy_node()
            rclpy.shutdown()
            return 1
        rclpy.spin_once(node, timeout_sec=0.2)

    grid = received[0]
    pub = node.create_publisher(OccupancyGrid, "/map", LATCHED)
    grid.header.stamp = node.get_clock().now().to_msg()

    # Publish a few times and stay alive briefly: a publisher that exits
    # immediately can be torn down before delivery completes.
    for _ in range(3):
        pub.publish(grid)
        end = node.get_clock().now().nanoseconds + 1e9
        while rclpy.ok() and node.get_clock().now().nanoseconds < end:
            rclpy.spin_once(node, timeout_sec=0.1)

    node.get_logger().info(
        "re-published /map %dx%d @%.3f m/cell"
        % (grid.info.width, grid.info.height, grid.info.resolution)
    )
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
