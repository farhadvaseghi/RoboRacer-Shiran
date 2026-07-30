#!/usr/bin/env python3
"""Drive a two-leg RoboRacer mission with Nav2.

The mission is fixed to the poses recorded on 2026-07-30:
  1. initial pose -> Goal 1
  2. Goal 1 -> Goal 2

Each goal is sent only after the previous NavigateToPose action succeeds.
The mission stops immediately if a goal is rejected, canceled, or fails.
"""

import os
import sys
import time

# This robot's complete stack runs on ROS domain 7.
os.environ.setdefault('ROS_DOMAIN_ID', '7')

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions


WAYPOINTS = (
    (
        'Goal 1',
        12.67165385576234,
        1.0165253135270669,
        0.6576101747284752,
        0.7533583862237048,
    ),
    (
        'Goal 2',
        1.6156199176312878,
        4.194904403911615,
        -0.7498062035160866,
        0.6616575074529064,
    ),
)

STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
    GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
    GoalStatus.STATUS_EXECUTING: 'EXECUTING',
    GoalStatus.STATUS_CANCELING: 'CANCELING',
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
}


class WaypointMission(Node):
    def __init__(self):
        super().__init__('rr_waypoint_mission')
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.current_goal = None
        self.current_name = ''
        self.last_feedback_time = 0.0

    def feedback_callback(self, feedback_msg):
        now = time.monotonic()
        if now - self.last_feedback_time < 1.0:
            return

        self.last_feedback_time = now
        distance = feedback_msg.feedback.distance_remaining
        self.get_logger().info(
            f'{self.current_name}: {distance:.2f} m remaining'
        )

    def run_goal(self, waypoint):
        name, x, y, qz, qw = waypoint
        self.current_name = name

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.get_logger().info(f'Sending {name}: x={x:.3f}, y={y:.3f}')
        send_future = self.client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback,
        )
        rclpy.spin_until_future_complete(self, send_future)
        self.current_goal = send_future.result()

        if self.current_goal is None or not self.current_goal.accepted:
            self.get_logger().error(f'{name} was rejected; mission stopped')
            self.current_goal = None
            return False

        self.get_logger().info(f'{name} accepted')
        result_future = self.current_goal.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        self.current_goal = None

        if result is None:
            self.get_logger().error(f'No result received for {name}')
            return False

        status_name = STATUS_NAMES.get(result.status, str(result.status))
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f'{name} finished with {status_name}; mission stopped'
            )
            return False

        self.get_logger().info(f'{name} reached successfully')
        return True

    def cancel_current_goal(self):
        if self.current_goal is None:
            return

        self.get_logger().warn('Canceling the active navigation goal')
        cancel_future = self.current_goal.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)
        self.current_goal = None


def print_mission():
    print('Mission route:')
    for index, (name, x, y, _qz, _qw) in enumerate(WAYPOINTS, start=1):
        print(f'  {index}. {name}: x={x:.3f}, y={y:.3f}')
    print()
    print('Keep the gamepad ready. Holding LB is the emergency override.')


def main():
    print_mission()
    try:
        confirmation = input('Type START to begin this mission: ')
    except (EOFError, KeyboardInterrupt):
        print('No confirmation received; mission not started.')
        return 1

    if confirmation.strip().upper() != 'START':
        print('Mission canceled; robot will not move.')
        return 1

    # Keep Python's normal Ctrl+C behavior so the exception handler below can
    # cancel the server-side action before shutting this client down.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    mission = WaypointMission()

    try:
        mission.get_logger().info('Waiting for /navigate_to_pose action server')
        if not mission.client.wait_for_server(timeout_sec=10.0):
            mission.get_logger().error(
                '/navigate_to_pose action server is unavailable; mission stopped'
            )
            return 1

        for index, waypoint in enumerate(WAYPOINTS):
            if not mission.run_goal(waypoint):
                return 1

            if index < len(WAYPOINTS) - 1:
                mission.get_logger().info(
                    'Waiting 2 seconds for the costmap reset before next goal'
                )
                time.sleep(2.0)

        mission.get_logger().info(
            'MISSION COMPLETE: Goal 1 -> Goal 2'
        )
        return 0
    except KeyboardInterrupt:
        mission.get_logger().warn('Ctrl+C received; stopping mission')
        mission.cancel_current_goal()
        return 130
    finally:
        mission.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
