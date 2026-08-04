#!/usr/bin/env python3
"""Full fresh start on a pose estimate, from Foxglove or from a script.

A /initialpose means "the car is here, start over", so EVERYTHING that carries
state between runs is reset:

  1. cancel active goals on BOTH navigate_to_pose and navigate_through_poses,
  2. publish empty paths so the custom controller drops stale plans, and KEEP
     re-publishing them for a couple of seconds,
  3. clear the global and local costmaps, then put the saved map back,
  4. ZERO THE WHEEL ODOMETRY by restarting vesc_to_odom, then re-anchor slam
     against the fresh odom frame.

Step 4 is why runs used to degrade. vesc_to_odom integrates from its own start
and nothing ever reset it, so drift, wheel slip and phantom travel accumulated
across every run of the session and were still there on the next one (measured
2026-08-01: 8.6 m of odom displacement on a car that had not moved). slam hid
this in map->odom, but odom is the motion prior between scan matches, so a bad
odom frame degrades tracking for the whole run.

The re-anchor in step 4 re-publishes the same pose after odom restarts; that
echo comes back to this node and is ignored for SUPPRESS_SECONDS.

The repeat in step 3 closes a race: the cancel is async, so bt_navigator can
tick once more and publish a fresh /plan just after a single empty path went
out -- the goal path would visibly come back a moment after the reset. Emptying
repeatedly until the cancel has taken effect makes the wipe stick.

Cancelling both actions matters: a route sent as NavigateThroughPoses is not
stopped by cancelling NavigateToPose, so bt_navigator would keep running the
route and republish a plan immediately after the reset.

When a goal on either action transitions to SUCCEEDED:
  1. clear the global and local costmaps,
  2. publish empty paths so the completed plan is not retained.

Action statuses already present when this node starts are treated as history and
do not trigger a reset.
"""

import os
import subprocess
import time

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

# --- odom zeroing -------------------------------------------------------------
# vesc_to_odom integrates from ITS OWN start, and nothing in the stack ever
# resets it, so drift and phantom travel carry across runs and accumulate. The
# only way to zero odom -> base_link is to restart that node, which is what a
# /initialpose now does: a pose estimate means "start fresh", so odom starts
# fresh too.
ODOM_NODE_PATTERN = 'vesc_to_odom_node'
VESC_PARAMS = os.path.expanduser(
    '~/f1tenth_ws/install/f1tenth_stack/share/f1tenth_stack/config/vesc.yaml'
)
ODOM_LOG = os.path.expanduser('~/rr_logs/vesc_to_odom.log')
ROS_SETUP = '/opt/ros/humble/setup.bash'
OVERLAY_SETUP = os.path.expanduser('~/f1tenth_ws/install/setup.bash')
DDS_PROFILE = os.path.expanduser('~/rr/fastdds_udp_only.xml')
ODOM_WAIT_SECONDS = 12.0
# Window in which our own re-published /initialpose is recognised as an echo.
SUPPRESS_SECONDS = 8.0


# How long to keep re-publishing empty paths after a reset: bt_navigator ticks
# at up to 100 Hz but the cancel round-trip plus one more planner cycle is well
# under 2 s, so 8 repeats at 0.25 s outlast it without spamming the controller.
EMPTY_PATH_REPEATS = 8
EMPTY_PATH_PERIOD = 0.25

# Re-publications of the saved map after a clear, on top of the one chained to
# the clear service's own response. Two at 0.6 s covers a lost response without
# pushing the 156 kB map over the Wi-Fi link more than necessary.
MAP_RESTORE_REPEATS = 2
MAP_RESTORE_PERIOD = 0.6


class NavigationReset(Node):
    def __init__(self):
        super().__init__('rr_costmap_reset')
        self._empty_timer = None
        self._empty_repeats = 0
        self._restore_timer = None
        self._restore_repeats = 0
        self._suppress_until = 0.0
        self.odom_count = 0
        self._pending_pose = None
        self._odom_before = 0
        self._odom_deadline = 0.0
        # Restarting vesc_to_odom zeroes the odometry, but the respawn is NOT
        # reliable: on 2026-08-01 one of several restarts failed to come back
        # and left the car with no wheel odometry at all -- far worse than
        # carrying odom drift, and it is not obvious until something else
        # fails. A pose estimate must never be able to do that, so this is off
        # by default here. rr_waypoint_mission does the same reset ONCE at
        # arming, where it verifies odom came back and refuses to drive if not.
        self.declare_parameter('reset_odom_on_pose', False)
        self.reset_odom_on_pose = self.get_parameter('reset_odom_on_pose').value

        self.gclear = self.create_client(
            ClearEntireCostmap,
            '/global_costmap/clear_entirely_global_costmap',
        )
        self.lclear = self.create_client(
            ClearEntireCostmap,
            '/local_costmap/clear_entirely_local_costmap',
        )
        # Both navigation actions must be cancelled: a route sent as
        # NavigateThroughPoses is not stopped by cancelling NavigateToPose,
        # and bt_navigator would keep republishing the plan after a reset.
        self.cancel_clients = {
            action: self.create_client(
                CancelGoal,
                f'/{action}/_action/cancel_goal',
            )
            for action in ('navigate_to_pose', 'navigate_through_poses')
        }

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

        # Cache the latched map so it can be re-published after every clear.
        self.last_map = None
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', latched)
        self.create_subscription(
            OccupancyGrid,
            '/map',
            self._on_map,
            latched,
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
        # Watches for odom coming back after the node is restarted.
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        # slam drops /initialpose sent with volatile QoS, so re-anchoring it
        # after the odom reset needs the latched profile.
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', latched
        )
        for action in self.cancel_clients:
            self.create_subscription(
                GoalStatusArray,
                f'/{action}/_action/status',
                lambda msg, action=action: self.on_goal_status(msg, action),
                action_status_qos,
            )

        self.goal_states = {action: {} for action in self.cancel_clients}
        self.status_initialized = {
            action: False for action in self.cancel_clients
        }
        # Drives the second half of the odom reset without blocking a callback.
        self.create_timer(0.25, self._check_odom_restart)
        self.get_logger().info(
            'rr_costmap_reset: FULL fresh start on /initialpose (goals, paths, '
            'costmaps, odom) and reset on successful goals'
        )

    def _on_odom(self, _msg):
        self.odom_count += 1

    def _on_map(self, msg):
        # Ignore our own re-publications: only map_server changes the content,
        # and re-caching our own copy would just churn.
        if self.last_map is None:
            self.get_logger().info(
                'cached /map %dx%d for post-clear restore'
                % (msg.info.width, msg.info.height)
            )
        self.last_map = msg

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
                future = client.call_async(ClearEntireCostmap.Request())
                if name == 'global':
                    # The map must go back AFTER the clear has actually run.
                    # Publishing it straight after call_async races the service
                    # and the clear can land second, wiping the map again.
                    future.add_done_callback(
                        lambda _future: self._republish_map()
                    )
            else:
                self.get_logger().warn(
                    f'{name} costmap clear service not available'
                )
        self._schedule_map_restore()

    def _schedule_map_restore(self):
        """Re-publish the map a couple more times, after the clear settles.

        Belt and braces for the done-callback above: if the clear response is
        lost or the static layer misses the first re-publication, these catch
        it. Without them a single missed restore leaves the costmap grey for
        the rest of the session.
        """
        self._restore_repeats = MAP_RESTORE_REPEATS
        if self._restore_timer is not None:
            self._restore_timer.cancel()
        self._restore_timer = self.create_timer(
            MAP_RESTORE_PERIOD, self._on_restore_timer
        )

    def _on_restore_timer(self):
        self._republish_map()
        self._restore_repeats -= 1
        if self._restore_repeats <= 0:
            self._restore_timer.cancel()
            self._restore_timer = None

    def _republish_map(self):
        """Put the static map back after a clear.

        "Clear entirely" resets the global costmap master grid to
        NO_INFORMATION, and the static layer only widens its update bounds when
        a NEW map message arrives. /map is latched and published once by
        map_server, so without this the saved map never returns and the whole
        costmap stays grey -- the planner is then left with nothing to plan on.
        """
        if self.last_map is None:
            self.get_logger().warn(
                'no /map cached yet; costmap will stay grey until map_server '
                'publishes again'
            )
            return

        self.last_map.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(self.last_map)

    def _empty_paths(self):
        self.plan_pub.publish(self._empty_path())
        self.ctrl_plan_pub.publish(self._empty_path())

    def _keep_emptying_paths(self):
        """Re-publish empty paths for a while, not just once.

        bt_navigator can publish one more /plan after the async cancel is sent
        but before it takes effect, which puts the goal path straight back on
        the screen and back into the controller. Repeating outlasts that.
        """
        self._empty_repeats = EMPTY_PATH_REPEATS
        if self._empty_timer is not None:
            self._empty_timer.cancel()
        self._empty_timer = self.create_timer(
            EMPTY_PATH_PERIOD, self._on_empty_timer
        )

    def _on_empty_timer(self):
        self._empty_paths()
        self._empty_repeats -= 1
        if self._empty_repeats <= 0:
            self._empty_timer.cancel()
            self._empty_timer = None

    def on_pose(self, msg):
        # Our own re-publication (step 6 below) comes back here; do not recurse.
        if self.get_clock().now().nanoseconds * 1e-9 < self._suppress_until:
            self.get_logger().info(
                '/initialpose echo after the odom reset -> costmaps only'
            )
            self._clear_costmaps()
            self._empty_paths()
            self._keep_emptying_paths()
            return

        self.get_logger().warn(
            'RESET: /initialpose received -> cancelling goals, emptying '
            'paths, clearing costmaps'
        )

        # 1. Stop acting on anything from the previous run first.
        for action, client in self.cancel_clients.items():
            if client.wait_for_service(timeout_sec=1.5):
                client.call_async(CancelGoal.Request())
            else:
                self.get_logger().warn(f'{action} cancel service not available')
        self._empty_paths()

        # 2. Costmaps back to just the saved map.
        self._clear_costmaps()

        # 3. Zero the wheel odometry. Nothing else resets it, so every run
        #    inherited the drift and the phantom travel of every run before it
        #    (measured 2026-08-01: 8.6 m of odom displacement on a car that had
        #    not moved). vesc_to_odom integrates from its own start, so the only
        #    way to zero it is to restart the node. This is started here and
        #    finished by _check_odom_restart on a timer -- a callback must never
        #    block waiting for other callbacks, and must never sit long enough
        #    for the next pose in a burst to queue up behind it.
        if self.reset_odom_on_pose:
            self.start_odom_reset(msg)
        else:
            self.get_logger().info(
                'odom left running (reset_odom_on_pose=false): the mission '
                'zeroes odom at arming, where a failed restart is caught '
                'before the car drives'
            )
        self._keep_emptying_paths()

    def start_odom_reset(self, pose):
        """Restart vesc_to_odom_node; _check_odom_restart finishes the job."""
        # Suppress FIRST: seeders publish the pose several times, and every one
        # of those would otherwise kick off its own restart.
        self._suppress_until = (
            self.get_clock().now().nanoseconds * 1e-9 + SUPPRESS_SECONDS
        )
        self._pending_pose = pose
        self._odom_before = self.odom_count
        self._odom_deadline = time.monotonic() + ODOM_WAIT_SECONDS

        subprocess.run(['pkill', '-f', ODOM_NODE_PATTERN], check=False)
        inner = (
            'source %s; [ -f %s ] && source %s; export ROS_DOMAIN_ID=7; '
            'export FASTRTPS_DEFAULT_PROFILES_FILE=%s; '
            'exec ros2 run vesc_ackermann vesc_to_odom_node --ros-args '
            '-r __node:=vesc_to_odom_node --params-file %s'
            % (ROS_SETUP, OVERLAY_SETUP, OVERLAY_SETUP, DDS_PROFILE, VESC_PARAMS)
        )
        with open(ODOM_LOG, 'ab') as log:
            subprocess.Popen(
                ['setsid', 'bash', '-c', inner],
                stdout=log, stderr=log, stdin=subprocess.DEVNULL,
                start_new_session=True,
            )
        self.get_logger().info('restarting vesc_to_odom to zero the odometry')

    def _check_odom_restart(self):
        """Timer: finish the odom reset once /odom is flowing again."""
        if self._pending_pose is None:
            return

        if self.odom_count > self._odom_before + 5:
            pose = self._pending_pose
            self._pending_pose = None
            self.get_logger().info(
                'odom zeroed (vesc_to_odom restarted, /odom flowing again)'
            )
            # slam was anchored against the OLD odom frame; re-send the pose so
            # it re-anchors against the fresh one. The echo lands back in
            # on_pose and is ignored for the rest of the suppress window.
            self._suppress_until = (
                self.get_clock().now().nanoseconds * 1e-9 + SUPPRESS_SECONDS
            )
            self.initialpose_pub.publish(pose)
            self.get_logger().info('re-anchored slam against the fresh odom')
            return

        if time.monotonic() > self._odom_deadline:
            self._pending_pose = None
            self.get_logger().error(
                'odom did NOT come back within %.0fs -- the car has no wheel '
                'odometry; run: bash ~/rr/kill_base.sh && ~/rr/rr_bringup.sh'
                % ODOM_WAIT_SECONDS
            )

    def on_goal_status(self, msg, action):
        current_states = {
            self._goal_id(status): status.status
            for status in msg.status_list
        }
        known_states = self.goal_states[action]

        if not self.status_initialized[action]:
            self.goal_states[action] = current_states
            self.status_initialized[action] = True
            self.get_logger().info(
                f'{action} status initialized; historical goals ignored'
            )
            return

        succeeded_goal_ids = [
            goal_id
            for goal_id, status in current_states.items()
            if status == GoalStatus.STATUS_SUCCEEDED
            and goal_id in known_states
            and known_states[goal_id] != GoalStatus.STATUS_SUCCEEDED
        ]

        self.goal_states[action] = current_states

        for goal_id in succeeded_goal_ids:
            self.get_logger().info(
                f'{action} goal {goal_id.hex()} succeeded -> clearing costmaps '
                'and emptying paths'
            )
            self._clear_costmaps()
            self._empty_paths()
            self._keep_emptying_paths()


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
