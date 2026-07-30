#!/usr/bin/env python3
"""Reset navigation state after relocalization or successful navigation.

On every /initialpose:
  1. clear the global and local costmaps,
  2. cancel active navigate_to_pose goals,
  3. publish empty paths so the custom controller drops stale plans.

When a navigate_to_pose goal transitions to SUCCEEDED:
  1. clear the global and local costmaps,
  2. publish empty paths so the completed plan is not retained.

Action statuses already present when this node starts are treated as history and
do not trigger a reset.
"""

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


class NavigationReset(Node):
    def __init__(self):
        super().__init__('rr_costmap_reset')

        self.gclear = self.create_client(
            ClearEntireCostmap,
            '/global_costmap/clear_entirely_global_costmap',
        )
        self.lclear = self.create_client(
            ClearEntireCostmap,
            '/local_costmap/clear_entirely_local_costmap',
        )
        self.cancel = self.create_client(
            CancelGoal,
            '/navigate_to_pose/_action/cancel_goal',
        )

        volatile = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        latched = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        action_status_qos = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.plan_pub = self.create_publisher(Path, '/plan', volatile)
        self.ctrl_plan_pub = self.create_publisher(
            Path,
            '/control/plan',
            latched,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.on_pose,
            10,
        )
        self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self.on_goal_status,
            action_status_qos,
        )

        self.goal_states = {}
        self.status_initialized = False
        self.get_logger().info(
            'rr_costmap_reset: reset on /initialpose and successful '
            'navigate_to_pose goals'
        )

    @staticmethod
    def _goal_id(status):
        return bytes(status.goal_info.goal_id.uuid)

    def _empty_path(self):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        return path

    def _clear_costmaps(self):
        for name, client in (
            ('global', self.gclear),
            ('local', self.lclear),
        ):
            if client.wait_for_service(timeout_sec=1.5):
                client.call_async(ClearEntireCostmap.Request())
            else:
                self.get_logger().warn(
                    f'{name} costmap clear service not available'
                )

    def _empty_paths(self):
        self.plan_pub.publish(self._empty_path())
        self.ctrl_plan_pub.publish(self._empty_path())

    def on_pose(self, _msg):
        self.get_logger().info(
            '/initialpose received -> clearing costmaps, cancelling goal, '
            'emptying paths'
        )
        self._clear_costmaps()

        if self.cancel.wait_for_service(timeout_sec=1.5):
            self.cancel.call_async(CancelGoal.Request())
        else:
            self.get_logger().warn(
                'navigate_to_pose cancel service not available'
            )

        self._empty_paths()

    def on_goal_status(self, msg):
        current_states = {
            self._goal_id(status): status.status
            for status in msg.status_list
        }

        if not self.status_initialized:
            self.goal_states = current_states
            self.status_initialized = True
            self.get_logger().info(
                'navigate_to_pose status initialized; historical goals ignored'
            )
            return

        succeeded_goal_ids = [
            goal_id
            for goal_id, status in current_states.items()
            if status == GoalStatus.STATUS_SUCCEEDED
            and goal_id in self.goal_states
            and self.goal_states[goal_id] != GoalStatus.STATUS_SUCCEEDED
        ]

        self.goal_states = current_states

        for goal_id in succeeded_goal_ids:
            self.get_logger().info(
                f'goal {goal_id.hex()} succeeded -> clearing costmaps and '
                'emptying paths'
            )
            self._clear_costmaps()
            self._empty_paths()


def main():
    rclpy.init()
    try:
        rclpy.spin(NavigationReset())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
