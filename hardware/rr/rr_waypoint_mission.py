#!/usr/bin/env python3
"""Drive a two-waypoint RoboRacer route with Nav2, without stopping en route.

The route is fixed to the poses recorded on 2026-07-30:
  initial pose -> Goal 1 -> Goal 2

Both poses are sent as ONE NavigateThroughPoses goal, so Nav2 plans a single
continuous path through Goal 1 and only applies the goal checker at Goal 2.
The car drives through Goal 1 without stopping and comes to rest at Goal 2.
The mission stops immediately if the route is rejected, canceled, or fails.

The operator is also asked for a global costmap clear interval. Scan/map
misalignment leaves obstacle marks on the non-rolling global costmap that
raytracing cannot remove (the real wall blocks the clearing rays), so they
accumulate until the planner refuses to start. Clearing the global costmap on
a timer drops that junk; anything real is re-marked on the next costmap update.
The clearing runs only while this mission drives, and only on the global
costmap -- the local costmap is rolling and cleans itself.
"""

import os
import sys
import time

# This robot's complete stack runs on ROS domain 7.
os.environ.setdefault('ROS_DOMAIN_ID', '7')

import rclpy
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from nav2_msgs.srv import ClearEntireCostmap
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
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

DEFAULT_CLEAR_INTERVAL = 3.0

# Seconds between global costmap updates (nav2_params_real.yaml
# global_costmap.update_frequency). Clearing faster than this refill rate is
# allowed but warned about, because a cleared costmap has no static walls in it
# until the next update.
COSTMAP_UPDATE_PERIOD = 1.0

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
    def __init__(self, clear_interval=0.0):
        super().__init__('rr_waypoint_mission')
        self.client = ActionClient(
            self,
            NavigateThroughPoses,
            '/navigate_through_poses',
        )
        self.current_goal = None
        self.last_feedback_time = 0.0
        self.clear_count = 0

        self.gclear = self.create_client(
            ClearEntireCostmap,
            '/global_costmap/clear_entirely_global_costmap',
        )
        self.set_params = self.create_client(
            SetParameters,
            '/global_costmap/global_costmap/set_parameters',
        )
        self.obstacle_layer_disabled = False
        self.stale_cancel_clients = {
            action: self.create_client(
                CancelGoal,
                f'/{action}/_action/cancel_goal',
            )
            for action in ('navigate_to_pose', 'navigate_through_poses')
        }
        if clear_interval > 0.0:
            self.create_timer(clear_interval, self.clear_global_costmap)
            self.get_logger().info(
                f'Clearing the global costmap every {clear_interval:g}s '
                'while this mission runs'
            )
        else:
            self.get_logger().info('Periodic global costmap clearing disabled')

    def cancel_stale_goals(self):
        """Cancel goals left running by an earlier, interrupted mission.

        An empty CancelGoal request cancels every goal on that action. Holding
        LB only overrides the mux, so an aborted run leaves its goal ACTIVE and
        bt_navigator would fight this new route for control of the plan.
        """
        for action, client in self.stale_cancel_clients.items():
            if not client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn(f'{action} cancel service not available')
                continue

            future = client.call_async(CancelGoal.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
            response = future.result()
            if response is not None and response.goals_canceling:
                self.get_logger().warn(
                    f'Cancelled {len(response.goals_canceling)} stale '
                    f'{action} goal(s) before starting'
                )

    def set_obstacle_layer(self, enabled):
        """Enable/disable the global costmap obstacle layer at runtime."""
        if not self.set_params.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(
                'global_costmap set_parameters service not available; '
                'obstacle layer left unchanged'
            )
            return False

        request = SetParameters.Request()
        request.parameters = [
            Parameter(
                name='obstacle_layer.enabled',
                value=ParameterValue(
                    type=ParameterType.PARAMETER_BOOL,
                    bool_value=enabled,
                ),
            )
        ]
        future = self.set_params.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        response = future.result()

        state = 'enabled' if enabled else 'DISABLED'
        if response is None or not response.results[0].successful:
            reason = '' if response is None else response.results[0].reason
            self.get_logger().error(
                f'Failed to set obstacle_layer.enabled={enabled} {reason}'
            )
            return False

        self.obstacle_layer_disabled = not enabled
        self.get_logger().info(f'Global costmap obstacle layer {state}')
        return True

    def clear_global_costmap(self):
        if not self.gclear.service_is_ready():
            self.get_logger().warn('Global costmap clear service not available')
            return

        self.gclear.call_async(ClearEntireCostmap.Request())
        self.clear_count += 1

    def feedback_callback(self, feedback_msg):
        now = time.monotonic()
        if now - self.last_feedback_time < 1.0:
            return

        self.last_feedback_time = now
        feedback = feedback_msg.feedback
        remaining = feedback.number_of_poses_remaining
        self.get_logger().info(
            f'{feedback.distance_remaining:.2f} m remaining, '
            f'{remaining} waypoint(s) still ahead'
        )

    def run_route(self, waypoints):
        """Send every waypoint as one goal so the car never stops en route."""
        goal = NavigateThroughPoses.Goal()
        stamp = self.get_clock().now().to_msg()
        for _name, x, y, qz, qw in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = stamp
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            goal.poses.append(pose)

        names = ' -> '.join(name for name, *_rest in waypoints)
        self.get_logger().info(f'Sending route as one goal: {names}')
        send_future = self.client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback,
        )
        rclpy.spin_until_future_complete(self, send_future)
        self.current_goal = send_future.result()

        if self.current_goal is None or not self.current_goal.accepted:
            self.get_logger().error('Route was rejected; mission stopped')
            self.current_goal = None
            return False

        self.get_logger().info('Route accepted; driving through all waypoints')
        result_future = self.current_goal.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        self.current_goal = None

        if result is None:
            self.get_logger().error('No result received for the route')
            return False

        status_name = STATUS_NAMES.get(result.status, str(result.status))
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f'Route finished with {status_name}; mission stopped'
            )
            return False

        return True

    def cancel_current_goal(self):
        if self.current_goal is None:
            return

        self.get_logger().warn('Canceling the active navigation goal')
        cancel_future = self.current_goal.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)
        self.current_goal = None


def print_mission():
    print('Mission route (driven as one continuous path, no stop en route):')
    for index, (name, x, y, _qz, _qw) in enumerate(WAYPOINTS, start=1):
        print(f'  {index}. {name}: x={x:.3f}, y={y:.3f}')
    print()
    print('Keep the gamepad ready. Holding LB is the emergency override.')


def ask_disable_obstacle_layer():
    """Ask whether to run with the global obstacle layer switched off."""
    print()
    print(
        'Disabling the obstacle layer means the planner sees ONLY the saved '
        'map: no phantom marks from scan/map misalignment, but also no '
        'unmapped obstacles. The layer is restored when this mission exits.'
    )
    answer = input('Disable the global obstacle layer? [y/N]: ').strip().lower()
    return answer in ('y', 'yes')


def ask_clear_interval():
    """Ask how often to clear the global costmap. Returns seconds, 0 = off."""
    prompt = (
        'Global costmap clear interval in seconds '
        f'(0 = off, sub-second allowed) [{DEFAULT_CLEAR_INTERVAL:.0f}]: '
    )
    while True:
        answer = input(prompt).strip()
        if not answer:
            return DEFAULT_CLEAR_INTERVAL

        try:
            interval = float(answer)
        except ValueError:
            print('  Enter a number of seconds, for example 3')
            continue

        if interval < 0.0:
            print('  The interval cannot be negative')
            continue

        if 0.0 < interval < COSTMAP_UPDATE_PERIOD:
            # A clear resets every layer, including the static map, and the
            # layers only refill on the next costmap update. Clearing faster
            # than that leaves the planner looking at a mostly empty costmap
            # -- no phantom marks, but no walls either.
            print(
                f'  WARNING: the global costmap refills every '
                f'{COSTMAP_UPDATE_PERIOD:.1f}s (update_frequency), so it will '
                'be empty most of the time and the planner may route through '
                'walls. Keep LB ready.'
            )

        return interval


def main():
    print_mission()
    try:
        disable_obstacles = ask_disable_obstacle_layer()
        if disable_obstacles:
            # With the layer off there are no obstacle marks to clear.
            clear_interval = 0.0
            print('  Periodic clearing skipped: the layer marks nothing.')
        else:
            clear_interval = ask_clear_interval()
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
    mission = WaypointMission(clear_interval=clear_interval)

    try:
        mission.get_logger().info(
            'Waiting for /navigate_through_poses action server'
        )
        if not mission.client.wait_for_server(timeout_sec=30.0):
            mission.get_logger().error(
                '/navigate_through_poses action server is unavailable; '
                'mission stopped. Nav2 needs ~30s after bring-up to finish '
                'building the planner heuristic table.'
            )
            return 1

        mission.cancel_stale_goals()

        if disable_obstacles and not mission.set_obstacle_layer(False):
            mission.get_logger().error(
                'Could not disable the obstacle layer; mission stopped'
            )
            return 1

        if not mission.run_route(WAYPOINTS):
            return 1

        mission.get_logger().info(
            f'MISSION COMPLETE: drove Goal 1 -> Goal 2 without stopping '
            f'({mission.clear_count} global costmap clears)'
        )
        return 0
    except KeyboardInterrupt:
        mission.get_logger().warn('Ctrl+C received; stopping mission')
        mission.cancel_current_goal()
        return 130
    finally:
        # Never leave the planner blind to unmapped obstacles after we exit.
        if mission.obstacle_layer_disabled and rclpy.ok():
            mission.set_obstacle_layer(True)
        mission.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
