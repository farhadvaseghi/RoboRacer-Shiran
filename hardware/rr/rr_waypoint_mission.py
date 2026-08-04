#!/usr/bin/env python3
"""Drive a fixed RoboRacer route with Nav2, without stopping en route.

The route always begins at the map origin (0, 0, 0 deg). Park the car there
before every run: the mission seeds that pose on /initialpose while arming, so
the waypoints below are always driven from the same starting place.

The route is a lap of the ring corridor, from poses picked in Foxglove on
2026-08-03 (they are in the corridor map re-made 2026-07-31 -- poses from any
older map do not carry over):
  start (0, 0) -> Goal 1 (top, turning left) -> Goal 2 (down the left side)
  -> Goal 3 (back along the bottom toward the start).
Each waypoint's heading is the direction of travel through it, not a heading
imposed on it. The previous route asked Goal 1 to face 63 deg in a straight
stretch where the car arrives travelling at 6.5 deg, and the planner paid for
that with a visible swing out and back into the wall.

Every pose is sent as ONE NavigateThroughPoses goal, so Nav2 plans a single
continuous path through the intermediate waypoints and only applies the goal
checker at the last one. The car drives through the waypoints in between
without stopping and comes to rest at the final goal. The mission stops
immediately if the route is rejected, canceled, or fails.

Holding the joystick deadman (LB) calls the run off too. It always gave the
driver the wheels -- ackermann_mux ranks teleop over nav2 -- but nav2 kept its
goal and took the car back 0.2 s after the button was released, which is the
opposite of what a deadman should mean. It now cancels the route as well.

Re-seeding the pose calls the run off. Any /initialpose from another publisher
cancels the route in flight and exits, because the plan in flight was built
from the pose being replaced -- continuing it drives the car from somewhere it
is not. The mission ignores the seed it publishes itself while arming.

The operator is also asked for a global costmap clear interval. Scan/map
misalignment leaves obstacle marks on the non-rolling global costmap that
raytracing cannot remove (the real wall blocks the clearing rays), so they
accumulate until the planner refuses to start. Clearing the global costmap on
a timer drops that junk; anything real is re-marked on the next costmap update.
The clearing runs only while this mission drives, and only on the global
costmap -- the local costmap is rolling and cleans itself.
"""

import math
import os
import re
import shutil
import signal
import subprocess
import sys
import time

# This robot's complete stack runs on ROS domain 7.
os.environ.setdefault('ROS_DOMAIN_ID', '7')

import rclpy
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateThroughPoses
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid, Odometry
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.action import ActionClient
from rclpy.duration import Duration as RclpyDuration
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time as RclpyTime
from sensor_msgs.msg import Joy
from tf2_ros import Buffer, TransformListener


# Start pose (map frame). The car is parked at the map origin before every run
# and this pose is seeded on /initialpose while arming, so the route always
# starts from the same place. ~/rr/rr_seed_start.py holds the same pose for the
# slam-restart path -- change both together.
START_X = 0.0
START_Y = 0.0
START_YAW_DEG = 0.0

# Seconds to wait after seeding: slam scan-matches from the seed and
# rr_costmap_reset clears both costmaps, and neither is instant.
SEED_SETTLE_SECONDS = 3.0

# Holding the deadman gives the driver the WHEELS but not the MISSION:
# ackermann_mux (mux.yaml) ranks 'teleop' at priority 100 over nav2's 'drive'
# at 10, with a 0.2 s timeout -- so 0.2 s after the button is released the mux
# falls back to nav2, which never stopped and still holds the same goal, and
# the car resumes driving to it from wherever it now is. Holding the deadman
# therefore also cancels the route here. Button 4 = LB, matching
# deadman_buttons in ~/rr/joy_teleop_fixed.yaml -- change both together.
DEADMAN_BUTTON = 4

# A press has to last this long to count, so a bumped button does not end a
# run. joy_node republishes at 20 Hz while a button is held (autorepeat_rate),
# so a deliberate press always clears this.
DEADMAN_ABORT_SECONDS = 0.3

# Route replaced 2026-08-03 with poses picked live in Foxglove. This is a lap
# of the ring corridor rather than the old out-and-back: up the right side,
# left along the top (Goal 1), down the left side (Goal 2), then back along the
# bottom toward the start (Goal 3).
#
# Checked against corridor_despeck with ~/rr/rr_goal_check.py before use --
# clearance to the nearest wall 0.45 / 0.81 / 0.91 m, all outside the 0.25 m
# inflation, all quaternions normalised.
#
# A FOURTH pose was offered at (0.448, 4.640) yaw -83.45 and deliberately left
# out: it sits 0.58 m from Goal 3 but laterally offset enough that the path
# between them has to swing ~46 deg, which needs ~0.8 m of arc at
# minimum_turning_radius 1.0. Two waypoints that close are not a finer route,
# they are an impossible one.
#
# Positions are as echoed from /goal_pose to millimetre precision, which is
# well inside the 0.05 m map resolution.
WAYPOINTS = (
    (
        'Goal 1',
        11.776,
        3.755,
        0.71734650427185609,
        0.69671657997276626,
    ),
    (
        'Goal 2',
        4.771,
        6.720,
        -0.99964914484254375,
        0.026487491681373172,
    ),
    (
        'Goal 3',
        -0.010,
        4.989,
        -0.68812920414097267,
        0.72558817411001908,
    ),
)

DEFAULT_CLEAR_INTERVAL = 3.0

# Seconds between global costmap updates (nav2_params_real.yaml
# global_costmap.update_frequency). Clearing faster than this refill rate is
# allowed but warned about, because a cleared costmap has no static walls in it
# until the next update.
COSTMAP_UPDATE_PERIOD = 1.0

# With the obstacle layer off the planner sees only the saved map, so the
# inflation gradient is the ONLY thing keeping a path off the walls. Widen
# it for the run; the original value is restored on exit. robot_radius is
# deliberately not touched - a bigger footprint can make the robot's own
# cell lethal, which is what blocks planning entirely.
#
# Sized for THIS corridor, re-measured for the 2026-08-03 lap route with
# ~/rr/rr_goal_check.py: clearance to the nearest wall is 0.45 m at Goal 1,
# 0.81 m at Goal 2, 0.91 m at Goal 3.
# With robot_radius 0.22 the inscribed band takes 0.22 m off each wall, so the
# usable lane is 0.56 m. Inflating 0.30 m from each wall leaves a 0.40 m
# zero-cost band down the middle to aim for. 0.40 would leave only 0.20 m.
# Goal 1 is the tight one: at 0.45 m clearance it sits only 0.15 m outside a
# 0.30 m inflation. If the planner balks there, lower this to ~0.25 rather
# than moving the waypoint.
OBSTACLE_OFF_INFLATION_RADIUS = 0.30

# --- drive speed -------------------------------------------------------------
# The pure_pursuit controller reads its parameters ONCE, in its constructor, and
# registers no on-set-parameters callback. A live 'ros2 param set' is therefore
# accepted and silently ignored. The only way to change its speed is to rewrite
# the YAML and restart the node, which is what apply_controller_speed() does.
# The original file is restored when this mission exits.
CONTROLLER_PARAMS = os.path.expanduser('~/rr/controller_params_real.yaml')
CONTROLLER_PATTERN = 'pure_pursuit_controller'
CONTROLLER_LOG = os.path.expanduser('~/rr_logs/pure_pursuit_controller.log')
ROS_SETUP = '/opt/ros/humble/setup.bash'
OVERLAY_SETUP = os.path.expanduser('~/f1tenth_ws/install/setup.bash')
WS_SETUP = os.path.expanduser('~/roboracer_ws/install/setup.bash')

# Speed keys that must move together.
SPEED_KEYS = ('speed', 'straight_speed', 'max_speed_command')

# --- slam max_laser_range ----------------------------------------------------
# FIXED at 20 m. slam_toolbox reads this when it first registers the laser and
# caches it per laser frame, so a live 'ros2 param set' is accepted and reads
# back but changes nothing; the only real way to change it is to rewrite the
# YAML and restart slam_toolbox, which DISCARDS the pose. The mission used to
# offer that as a prompt, which meant two runs could be localising differently
# and no comparison between them meant anything. It is now a fixed value the
# mission only checks.
SLAM_MAX_LASER_RANGE = 20.0
SLAM_PARAMS = os.path.expanduser('~/rr/localize_slam_real.yaml')
SLAM_PATTERN = 'localization_slam_toolbox_node'
SLAM_LOG = os.path.expanduser('~/rr_logs/slam_toolbox.log')
SEED_SCRIPT = os.path.expanduser('~/rr/rr_seed_start.py')

# --- odom zeroing at arming ---------------------------------------------------
# The odom frame is zeroed here, at the one moment the car is guaranteed parked
# and still, and the start pose is seeded immediately afterwards.
#
# WHY IT MATTERS (2026-08-03): the pose is map->odom composed with
# odom->base_link. If odom->base_link has grown large, it becomes a LEVER ARM:
# an error of d radians in slam's map->odom rotation lands as
# |odom->base_link| * d of position error. Measured that day: 45 m of
# accumulated odom turned a ~1 degree yaw error into 0.8 m, and the composed
# pose intermittently jumped clean off a 38x10 m map ('Robot is out of bounds
# of the costmap'), so the car tracked a valid plan straight into a wall.
# Zeroed, the same angular error costs centimetres.
#
# rr_gyro_odom owns odom->base_link (since 2026-08-01) and keeps its own x/y/yaw
# across a vesc_to_odom restart, so restarting vesc_to_odom alone does NOT zero
# the transform -- that is what this used to do, and it silently stopped working
# the day the gyro node took over. Both are restarted: vesc_to_odom for the
# wheel speed, rr_gyro_odom for the transform itself.
ODOM_NODE_PATTERN = 'vesc_to_odom_node'
VESC_PARAMS = os.path.expanduser(
    '~/f1tenth_ws/install/f1tenth_stack/share/f1tenth_stack/config/vesc.yaml'
)
ODOM_LOG = os.path.expanduser('~/rr_logs/vesc_to_odom.log')
DDS_PROFILE = os.path.expanduser('~/rr/fastdds_udp_only.xml')
ODOM_WAIT_SECONDS = 15.0

# rr_gyro_odom measures its gyro bias over the first few seconds at standstill
# and only starts publishing the transform once that is done, logging this line.
# The CAR MUST BE STILL for the whole of it, which is why this runs at arming.
GYRO_SCRIPT = os.path.expanduser('~/rr/rr_gyro_odom.py')
GYRO_RESTART_SCRIPT = os.path.expanduser('~/rr/rr_restart_gyro.sh')
GYRO_PATTERN = 'rr_gyro_odo[m]\\.py'
GYRO_LOG = os.path.expanduser('~/rr_logs/rr_gyro_odom.log')
GYRO_READY_MARKER = 'integrating now'
GYRO_WAIT_SECONDS = 30.0

# After zeroing, odom->base_link must be ~0. Anything above this means the
# restart did not take and the lever arm is still there, so do not drive.
ODOM_ZERO_TOLERANCE_M = 0.50

# =============================================================================
# PLANNER MINIMUM TURNING RADIUS  --  EDIT THIS VALUE, THEN RECYCLE THE STACK
# =============================================================================
# Set the radius the planner should use, in metres. On every run the mission
# writes this into nav2_params_real.yaml and CHECKS the running planner against
# it; it never restarts nav2 itself.
#
# Nav2 reads this value only when it STARTS: SmacPlannerHybrid builds its motion
# primitives and heuristic table in configure(), so a live parameter set is
# accepted and silently ignored. After changing the number here:
#
#     bash ~/rr/rr_recycle.sh          # ~90 s, brings the whole stack back
#     python3 ~/rr/rr_seed_start.py    # car parked on the origin
#     python3 ~/rr/rr_waypoint_mission.py
#
# If the running planner disagrees with this value the mission stops before the
# car moves and tells you to recycle -- it will not drive on a stale radius.
MINIMUM_TURNING_RADIUS = 1.0

NAV_PARAMS = os.path.expanduser(
    '~/roboracer_ws/src/RoboRacer-Shiran/roboracer_estimation/config/'
    'nav2_params_real.yaml'
)

# Measured on the car: the tightest circle it can physically drive. Planning
# below this produces corners the steering cannot hold, so the car saturates
# and drifts wide into the outer wall (seen 2026-07-23).
CAR_MIN_TURNING_RADIUS = 0.85

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

        # Used to CONFIRM the odom zeroing actually took, rather than assume it.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.gclear = self.create_client(
            ClearEntireCostmap,
            '/global_costmap/clear_entirely_global_costmap',
        )
        self.set_params = self.create_client(
            SetParameters,
            '/global_costmap/global_costmap/set_parameters',
        )
        self.obstacle_layer_disabled = False
        self.get_params = self.create_client(
            GetParameters,
            '/global_costmap/global_costmap/get_parameters',
        )
        self.saved_inflation_radius = None
        self.route_seconds = None
        latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        # slam_toolbox drops /initialpose messages published with volatile QoS.
        self.initialpose = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', latched
        )
        # Cache the latched map so each costmap clear can restore it.
        self.last_map = None
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', latched)
        self.create_subscription(
            OccupancyGrid, '/map', self._on_map, latched
        )

        self.odom_count = 0
        self.create_subscription(
            Odometry, '/odom', lambda _m: self._count_odom(), 10
        )
        self.stale_cancel_clients = {
            action: self.create_client(
                CancelGoal,
                f'/{action}/_action/cancel_goal',
            )
            for action in ('navigate_to_pose', 'navigate_through_poses')
        }

        # An operator re-seeding the pose (Foxglove "2D Pose Estimate", rviz,
        # or ros2 topic pub) means "forget what you were doing": the route in
        # flight was planned from the OLD pose, so continuing it drives the car
        # from a place it is no longer at. Watch /initialpose and abort.
        #
        # This node publishes its own start-pose seed on the same topic and
        # receives it back, so the callback ignores any message carrying the
        # stamp we published. Nothing is watched until the seed is done, which
        # also swallows a latched pose left over from an earlier run.
        self.abort_reason = None
        self.watch_initialpose = False
        self.route_active = False
        self.deadman_since = None
        self.seed_stamp_ns = None
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self._on_initialpose,
            # VOLATILE so this also matches rviz/Foxglove publishers, which do
            # not offer TRANSIENT_LOCAL; a TRANSIENT_LOCAL publisher (ours)
            # still satisfies a VOLATILE subscription.
            QoSProfile(
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE,
                history=QoSHistoryPolicy.KEEP_LAST,
            ),
        )

        # BEST_EFFORT: a best-effort subscription matches a publisher of either
        # reliability, so this cannot silently fail to connect to joy_node.
        # Losing a message costs nothing here -- a held button arrives 20 times
        # a second.
        self.create_subscription(
            Joy,
            '/joy',
            self._on_joy,
            QoSProfile(
                depth=10,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
                history=QoSHistoryPolicy.KEEP_LAST,
            ),
        )
        if clear_interval > 0.0:
            self.create_timer(clear_interval, self.clear_global_costmap)
            self.get_logger().info(
                f'Clearing the global costmap every {clear_interval:g}s '
                'while this mission runs'
            )
        else:
            self.get_logger().info('Periodic global costmap clearing disabled')

    def _on_joy(self, msg):
        """Cancel the route when the driver takes over with the deadman.

        Only sets a flag; see _on_initialpose for why. Armed only while a route
        is in flight, so holding the stick during arming is harmless.
        """
        held = (
            len(msg.buttons) > DEADMAN_BUTTON
            and msg.buttons[DEADMAN_BUTTON]
        )
        if not held:
            self.deadman_since = None
            return

        if not self.route_active or self.abort_reason is not None:
            return

        now = time.monotonic()
        if self.deadman_since is None:
            self.deadman_since = now
            return

        held_for = now - self.deadman_since
        if held_for < DEADMAN_ABORT_SECONDS:
            return

        self.abort_reason = (
            'the joystick deadman (LB) was held for %.1fs -- the driver has '
            'the car' % held_for
        )
        self.get_logger().warn(
            'Deadman held; revoking the route so the car does not resume '
            'driving to the goal when the button is released'
        )

    def _on_initialpose(self, msg):
        """Flag an external pose reset; the route loop does the aborting.

        Only sets a flag: this runs inside the executor, and canceling a goal
        needs to spin, which cannot be done from a callback.
        """
        stamp_ns = msg.header.stamp.sec * 10 ** 9 + msg.header.stamp.nanosec
        if stamp_ns == self.seed_stamp_ns:
            return

        pose = msg.pose.pose
        yaw = math.degrees(
            2.0 * math.atan2(pose.orientation.z, pose.orientation.w)
        )
        yaw = (yaw + 180.0) % 360.0 - 180.0
        where = 'x=%.3f y=%.3f yaw=%.1f deg' % (
            pose.position.x, pose.position.y, yaw
        )

        if not self.watch_initialpose:
            self.get_logger().info(
                f'/initialpose {where} received before the route started; '
                'nothing to cancel'
            )
            return

        if self.abort_reason is not None:
            return

        self.abort_reason = f'the pose was re-seeded externally to {where}'
        self.get_logger().warn(
            f'/initialpose {where} received from another publisher -- '
            'revoking the route'
        )

    def _on_map(self, msg):
        self.last_map = msg

    def _count_odom(self):
        self.odom_count += 1

    def check_turning_radius(self):
        """Sync MINIMUM_TURNING_RADIUS to the YAML and verify the live planner.

        Returns True only when the running planner is actually using the value
        set at the top of this file. Nothing is restarted here: restarting nav2
        from inside the mission proved fragile, so a mismatch stops the run and
        asks for a recycle instead of driving on a stale radius.
        """
        if MINIMUM_TURNING_RADIUS < CAR_MIN_TURNING_RADIUS:
            self.get_logger().warn(
                'MINIMUM_TURNING_RADIUS %.2f m is below the car\'s physical '
                'minimum %.2f m; the planner will draw corners the car cannot '
                'follow' % (MINIMUM_TURNING_RADIUS, CAR_MIN_TURNING_RADIUS)
            )

        on_file = read_yaml_float('minimum_turning_radius', NAV_PARAMS)
        if on_file is None:
            self.get_logger().error(
                'minimum_turning_radius not found in %s' % NAV_PARAMS
            )
            return False

        if abs(on_file - MINIMUM_TURNING_RADIUS) > 1e-3:
            set_yaml_floats(
                {'minimum_turning_radius': MINIMUM_TURNING_RADIUS}, NAV_PARAMS
            )
            self.get_logger().warn(
                'nav2_params_real.yaml updated %.2f -> %.2f m'
                % (on_file, MINIMUM_TURNING_RADIUS)
            )

        live = self._live_turning_radius()
        if live is None:
            self.get_logger().warn(
                'could not read the planner\'s live turning radius; '
                'continuing on the file value %.2f m' % MINIMUM_TURNING_RADIUS
            )
            return True

        if abs(live - MINIMUM_TURNING_RADIUS) > 1e-3:
            self.get_logger().error(
                'nav2 is RUNNING with minimum_turning_radius %.2f m but this '
                'mission is set to %.2f m. Nav2 only reads it at startup.'
                % (live, MINIMUM_TURNING_RADIUS)
            )
            self.get_logger().error(
                'Apply it with:  bash ~/rr/rr_recycle.sh  then '
                'python3 ~/rr/rr_seed_start.py  then re-run this mission.'
            )
            return False

        self.get_logger().info(
            'Planner minimum_turning_radius %.2f m (matches this mission)'
            % live
        )
        return True

    def _live_turning_radius(self):
        """Read minimum_turning_radius from the running planner_server."""
        client = self.create_client(
            GetParameters, '/planner_server/get_parameters'
        )
        if not client.wait_for_service(timeout_sec=5.0):
            return None
        request = GetParameters.Request()
        request.names = ['GridBased.minimum_turning_radius']
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        response = future.result()
        if response is None or not response.values:
            return None
        value = response.values[0]
        if value.type != ParameterType.PARAMETER_DOUBLE:
            return None
        return value.double_value

    def reset_odom(self):
        """Zero odom -> base_link, then prove it is actually zero.

        Restarts BOTH nodes in the odometry chain: vesc_to_odom for the wheel
        speed, and rr_gyro_odom, which is what actually owns the transform and
        carries its own x/y/yaw. Restarting only the first leaves the lever arm
        in place -- see the ODOM ZEROING note at the top of this file.

        Returns True only when /odom is flowing, rr_gyro_odom has finished its
        standstill bias calibration, and odom -> base_link reads ~0. A False
        here stops the mission before the car moves, which is the whole point
        of doing this at arming rather than mid-run.
        """
        before = self.odom_count
        subprocess.run(['pkill', '-f', ODOM_NODE_PATTERN], check=False)
        time.sleep(1.5)

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

        deadline = time.time() + ODOM_WAIT_SECONDS
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.odom_count > before + 5:
                break
        else:
            self.get_logger().error(
                'Odom did NOT come back within %.0fs. The car has no wheel '
                'odometry, so this mission will not drive. Fix with: '
                'bash ~/rr/kill_base.sh && ~/rr/rr_bringup.sh'
                % ODOM_WAIT_SECONDS
            )
            return False

        self.get_logger().info('Wheel odom restarted (/odom flowing)')
        return self.reset_gyro_odom()

    def reset_gyro_odom(self):
        """Restart rr_gyro_odom so odom -> base_link starts from zero.

        The car must be STANDING STILL for this: the node averages the gyro at
        standstill to measure its bias, and rejects the batch if the wheels
        report motion. It publishes nothing until that finishes, so the wait
        watches its log for the ready marker rather than guessing a sleep.
        """
        if not os.path.exists(GYRO_RESTART_SCRIPT):
            self.get_logger().error(
                '%s is missing; cannot zero odom -> base_link'
                % GYRO_RESTART_SCRIPT
            )
            return False

        try:
            mark = os.path.getsize(GYRO_LOG)
        except OSError:
            mark = 0

        self.get_logger().info(
            'Restarting rr_gyro_odom -- KEEP THE CAR STILL while the gyro '
            'bias is measured'
        )
        subprocess.run(['bash', GYRO_RESTART_SCRIPT], check=False)

        deadline = time.time() + GYRO_WAIT_SECONDS
        ready = False
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            try:
                # rr_restart_gyro.sh redirects with '>', which TRUNCATES the
                # log, so the pre-restart size can be past the new end of file.
                # A shrunk file means the restart happened and everything in it
                # belongs to the new run.
                if os.path.getsize(GYRO_LOG) < mark:
                    mark = 0
                with open(GYRO_LOG) as fh:
                    fh.seek(mark)
                    if GYRO_READY_MARKER in fh.read():
                        ready = True
                        break
            except OSError:
                pass

        if not ready:
            self.get_logger().error(
                'rr_gyro_odom did not finish its bias calibration within '
                '%.0fs. It rejects the batch if the wheels report motion, so '
                'check the car is standing still, then re-run. Log: %s'
                % (GYRO_WAIT_SECONDS, GYRO_LOG)
            )
            return False

        # Prove it rather than trust it: the transform has to read ~0 now.
        offset = self.odom_base_offset()
        if offset is None:
            self.get_logger().error(
                'rr_gyro_odom reports ready but odom -> base_link does not '
                'resolve; this mission will not drive.'
            )
            return False
        if offset > ODOM_ZERO_TOLERANCE_M:
            self.get_logger().error(
                'odom -> base_link is %.2f m after the restart, expected ~0. '
                'The zeroing did not take, and driving on a long lever arm is '
                'what threw the pose off the map on 2026-08-03.' % offset
            )
            return False

        self.get_logger().info(
            'Odom zeroed: odom -> base_link %.3f m, gyro calibrated' % offset
        )
        return True

    def odom_base_offset(self):
        """Distance of odom -> base_link from the origin, or None."""
        deadline = time.time() + 5.0
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            try:
                tf = self.tf_buffer.lookup_transform(
                    'odom', 'base_link', RclpyTime(),
                    timeout=RclpyDuration(seconds=0.0))
            except Exception:
                continue
            return math.hypot(tf.transform.translation.x,
                              tf.transform.translation.y)
        return None
        return False

    def seed_start_pose(self):
        """Declare that the car is parked at the start pose (the map origin).

        slam_toolbox does not self-localize -- it only re-localizes when it is
        given an /initialpose -- so every run starts by telling it where the
        car is. rr_costmap_reset also listens on /initialpose and clears both
        costmaps plus any stale plan, which is why this runs before the route
        is sent rather than after.

        The car MUST physically be at the start pose when this runs; seeding a
        pose the car is not at makes the whole route drive off by that error.
        """
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        # Remembered so _on_initialpose can tell this seed apart from an
        # operator re-seeding the pose to cancel the run.
        self.seed_stamp_ns = (
            msg.header.stamp.sec * 10 ** 9 + msg.header.stamp.nanosec
        )
        msg.pose.pose.position.x = START_X
        msg.pose.pose.position.y = START_Y
        yaw = math.radians(START_YAW_DEG)
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        # Modest confidence: parked on the marked spot, but not surveyed.
        msg.pose.covariance[0] = 0.02
        msg.pose.covariance[7] = 0.02
        msg.pose.covariance[35] = 0.02

        # Publish a few times: slam latches one, but the repeats survive a
        # subscriber that is still coming up.
        for _ in range(5):
            self.initialpose.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.2)
            time.sleep(0.3)

        self.get_logger().info(
            'Seeded the start pose x=%.3f y=%.3f yaw=%.2f deg; waiting %.1fs '
            'for slam to match and the costmaps to clear'
            % (START_X, START_Y, START_YAW_DEG, SEED_SETTLE_SECONDS)
        )
        time.sleep(SEED_SETTLE_SECONDS)

        # From here on, any /initialpose that is not this seed cancels the run.
        self.watch_initialpose = True

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

    def widen_wall_clearance(self, radius=OBSTACLE_OFF_INFLATION_RADIUS):
        """Grow the global inflation so planned paths sit further off walls.

        Only the soft inflation gradient changes. robot_radius is left alone on
        purpose: a larger footprint marks more cells inscribed and can make the
        robot's own cell lethal, which refuses every goal.
        """
        if not self.get_params.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(
                'global_costmap get_parameters unavailable; '
                'wall clearance left unchanged'
            )
            return False

        request = GetParameters.Request()
        request.names = ['inflation_layer.inflation_radius']
        future = self.get_params.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        response = future.result()
        if response is None or not response.values:
            self.get_logger().error(
                'Could not read the current inflation radius'
            )
            return False

        current = response.values[0].double_value
        if not self._set_inflation_radius(radius):
            return False

        self.saved_inflation_radius = current
        self.get_logger().info(
            f'Wall clearance: inflation radius {current:.2f} -> {radius:.2f} m'
        )
        return True

    def restore_wall_clearance(self):
        """Put the original inflation radius back."""
        if self.saved_inflation_radius is None:
            return True
        if self._set_inflation_radius(self.saved_inflation_radius):
            self.get_logger().info(
                f'Wall clearance restored to '
                f'{self.saved_inflation_radius:.2f} m'
            )
            self.saved_inflation_radius = None
            return True
        return False

    def _set_inflation_radius(self, radius):
        if not self.set_params.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(
                'global_costmap set_parameters service not available; '
                'inflation radius left unchanged'
            )
            return False

        request = SetParameters.Request()
        request.parameters = [
            Parameter(
                name='inflation_layer.inflation_radius',
                value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE,
                    double_value=float(radius),
                ),
            )
        ]
        future = self.set_params.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        response = future.result()
        if response is None or not response.results[0].successful:
            reason = '' if response is None else response.results[0].reason
            self.get_logger().error(
                f'Failed to set inflation_radius={radius} {reason}'
            )
            return False
        return True

    def clear_global_costmap(self):
        if not self.gclear.service_is_ready():
            self.get_logger().warn('Global costmap clear service not available')
            return

        # Restore the map only once the clear has actually run: publishing it
        # straight after call_async races the service, and a clear landing
        # second wipes the map back out.
        future = self.gclear.call_async(ClearEntireCostmap.Request())
        future.add_done_callback(lambda _future: self.republish_map())
        self.clear_count += 1

    def republish_map(self):
        """Put the static map back after a clear.

        "Clear entirely" resets the global costmap master grid to
        NO_INFORMATION, and the static layer only re-applies its data when a
        NEW map message arrives. /map is latched and published once, so a clear
        without this leaves the costmap 100% unknown -- grey everywhere, and
        the planner has nothing to plan on for the rest of the run.
        """
        if self.last_map is None:
            self.get_logger().warn('no /map cached yet; costmap stays cleared')
            return

        self.last_map.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(self.last_map)

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
        if self.abort_reason is not None:
            self.get_logger().error(
                f'Not starting the route: {self.abort_reason}'
            )
            return False

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

        self.get_logger().info(
            'Route accepted; driving through all waypoints. Publishing '
            '/initialpose (Foxglove "2D Pose Estimate") cancels it.'
        )
        route_started = time.monotonic()
        self.route_active = True
        result_future = self.current_goal.get_result_async()
        # Spin in slices instead of blocking on the result: an abort flagged by
        # _on_initialpose can only be acted on between spins.
        while rclpy.ok() and not result_future.done():
            rclpy.spin_until_future_complete(self, result_future,
                                             timeout_sec=0.2)
            if self.abort_reason is not None:
                self.route_seconds = time.monotonic() - route_started
                self.get_logger().error(
                    f'Route ABORTED after {self.route_seconds:.1f}s: '
                    f'{self.abort_reason}'
                )
                self.route_active = False
                self.cancel_current_goal()
                # Belt and braces: cancel every goal on both nav actions, so no
                # half-dead goal from this run is left for the next one.
                self.cancel_stale_goals()
                return False

        self.route_active = False
        result = result_future.result()
        self.route_seconds = time.monotonic() - route_started
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
    print('  0. Start: x=%.3f, y=%.3f, yaw=%.1f deg -- PARK THE CAR HERE'
          % (START_X, START_Y, START_YAW_DEG))
    for index, (name, x, y, _qz, _qw) in enumerate(WAYPOINTS, start=1):
        print(f'  {index}. {name}: x={x:.3f}, y={y:.3f}')
    print()
    print('Planner minimum_turning_radius: %.2f m (hard-coded at the top of '
          'this file; a change needs rr_recycle.sh to take effect).'
          % MINIMUM_TURNING_RADIUS)
    print('The start pose is seeded on /initialpose while arming, so the car '
          'has to be standing on it.')
    print('To call the run off: publish /initialpose (Foxglove "2D Pose '
          'Estimate") -- the route is canceled and this mission exits, '
          'instead of driving on from the pose it no longer has.')
    print('Keep the gamepad ready. Holding LB takes the wheels AND cancels '
          'the route -- without that cancel the mux hands control back to '
          'nav2 0.2s after you let go and the car drives on to the goal.')


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


def read_yaml_float(key, path=CONTROLLER_PARAMS):
    """Return the current value of a scalar float key, or None."""
    pattern = re.compile(r'^\s*%s:\s*([-+0-9.eE]+)' % re.escape(key), re.M)
    match = pattern.search(open(path).read())
    return float(match.group(1)) if match else None


def set_yaml_floats(values, path=CONTROLLER_PARAMS):
    """Rewrite scalar float keys in place, preserving indentation and comments."""
    text = open(path).read()
    for key, value in values.items():
        pattern = re.compile(r'^(\s*%s:\s*)([-+0-9.eE]+)' % re.escape(key), re.M)
        if not pattern.search(text):
            continue
        text = pattern.sub(
            lambda m: '%s%.3f' % (m.group(1), value), text, count=1
        )
    open(path, 'w').write(text)


def restart_controller():
    """Stop and respawn the controller so it re-reads its parameter file."""
    subprocess.run(['pkill', '-f', CONTROLLER_PATTERN], check=False)
    time.sleep(2.0)
    inner = (
        'source %s; [ -f %s ] && source %s; [ -f %s ] && source %s; '
        'export ROS_DOMAIN_ID=7; '
        'exec ros2 run roboracer_control pure_pursuit_controller '
        '--ros-args --params-file %s'
        % (ROS_SETUP, OVERLAY_SETUP, OVERLAY_SETUP, WS_SETUP, WS_SETUP,
           CONTROLLER_PARAMS)
    )
    with open(CONTROLLER_LOG, 'ab') as log:
        subprocess.Popen(
            ['setsid', 'bash', '-c', inner],
            stdout=log, stderr=log, stdin=subprocess.DEVNULL,
            start_new_session=True,
        )
    for _ in range(30):
        time.sleep(0.5)
        found = subprocess.run(
            ['pgrep', '-f', CONTROLLER_PATTERN], capture_output=True
        )
        if found.returncode == 0:
            time.sleep(1.5)
            return True
    return False


def apply_controller_speed(speed):
    """Back up the controller params, raise the speed, restart the node.

    Returns the backup path so the caller can restore it on exit.
    """
    backup = CONTROLLER_PARAMS + '.bak_mission_speed'
    shutil.copy2(CONTROLLER_PARAMS, backup)

    straight = read_yaml_float('straight_speed') or speed
    turn = read_yaml_float('turn_speed')
    look_time = read_yaml_float('lookahead_time') or 0.8
    max_look = read_yaml_float('max_lookahead_distance') or 0.7

    values = dict((key, speed) for key in SPEED_KEYS)
    # Keep cornering in proportion to the straights instead of crawling.
    if turn is not None and straight > 0:
        values['turn_speed'] = turn / straight * speed

    # Pure pursuit previews a fixed DISTANCE. If the lookahead cap stays put
    # while speed rises, the preview TIME shrinks and the car starts weaving.
    # Hold the cap at no less than speed * lookahead_time.
    wanted = speed * look_time
    if wanted > max_look:
        values['max_lookahead_distance'] = wanted
        print('  lookahead cap raised %.2f -> %.2f m to hold a %.2f s preview'
              % (max_look, wanted, look_time))

    set_yaml_floats(values)
    print('  controller params updated; restarting controller ...')
    if not restart_controller():
        print('  WARNING: the controller did not come back up!')
        return backup
    print('  controller restarted at %.2f m/s' % speed)
    return backup


def restore_controller_params(backup):
    """Put the original controller parameters back and restart the node."""
    if not backup or not os.path.exists(backup):
        return
    shutil.copy2(backup, CONTROLLER_PARAMS)
    os.remove(backup)
    print('Restoring the original controller speed ...')
    restart_controller()


def ask_speed():
    """Ask for a drive speed. Blank keeps whatever is already configured."""
    current = read_yaml_float('straight_speed')
    top = read_yaml_float('max_speed_command')
    print('')
    print('Current drive speed is %.2f m/s (max_speed_command %.2f).'
          % (current or 0.0, top or 0.0))
    print('Raising it rewrites the controller YAML and restarts the '
          'controller. The original is restored when this mission exits.')
    answer = input(
        'Drive speed in m/s [blank = keep %.2f]: ' % (current or 0.0)
    ).strip()
    if not answer:
        return None
    try:
        speed = float(answer)
    except ValueError:
        print('  Not a number; keeping the current speed.')
        return None
    if speed <= 0.0:
        print('  Speed must be positive; keeping the current speed.')
        return None
    if current and speed > current * 3.0:
        print('  Refusing a jump larger than 3x the current speed.')
        return None
    return speed


def check_slam_max_laser_range():
    """Verify slam is running the fixed max_laser_range. Never changes it.

    This used to be an interactive prompt that rewrote the YAML and restarted
    slam mid-mission. Restarting slam DISCARDS the pose, so a run that answered
    the prompt was localising differently from a run that did not -- which made
    every comparison between runs meaningless. The range is fixed at
    SLAM_MAX_LASER_RANGE now and this only reports disagreement.
    """
    on_file = read_yaml_float('max_laser_range', SLAM_PARAMS)
    if on_file is None:
        print('WARNING: max_laser_range not found in %s' % SLAM_PARAMS)
    elif abs(on_file - SLAM_MAX_LASER_RANGE) > 1e-3:
        print('WARNING: %s has max_laser_range %.2f m, not the fixed %.2f m. '
              'Edit the file and restart slam if that is wrong.'
              % (SLAM_PARAMS, on_file, SLAM_MAX_LASER_RANGE))
    else:
        print('slam max_laser_range fixed at %.2f m' % SLAM_MAX_LASER_RANGE)


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
        speed_choice = ask_speed()
        check_slam_max_laser_range()
        confirmation = input('Type START to begin this mission: ')
    except (EOFError, KeyboardInterrupt):
        print('No confirmation received; mission not started.')
        return 1

    speed_backup = None
    if confirmation.strip().upper() == 'START':
        if speed_choice is not None:
            speed_backup = apply_controller_speed(speed_choice)

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
        if not mission.client.wait_for_server(timeout_sec=120.0):
            mission.get_logger().error(
                '/navigate_through_poses action server is unavailable; '
                'mission stopped. Nav2 needs ~30 s after bring-up to build the '
                'planner heuristic table, longer when the Jetson is loaded.'
            )
            return 1

        if not mission.check_turning_radius():
            return 1

        mission.cancel_stale_goals()

        # Zero odom BEFORE seeding: slam anchors against the odom frame, so the
        # pose must be given after odom restarts, not before.
        if not mission.reset_odom():
            return 1

        mission.seed_start_pose()

        if disable_obstacles and not mission.set_obstacle_layer(False):
            mission.get_logger().error(
                'Could not disable the obstacle layer; mission stopped'
            )
            return 1

        if disable_obstacles and not mission.widen_wall_clearance():
            mission.get_logger().error(
                'Could not widen wall clearance; mission stopped'
            )
            return 1

        if not mission.run_route(WAYPOINTS):
            return 1

        route = ' -> '.join(['start'] + [name for name, *_rest in WAYPOINTS])
        mission.get_logger().info(
            f'MISSION COMPLETE: drove {route} without stopping '
            f'({mission.clear_count} global costmap clears)'
        )
        if mission.route_seconds is not None:
            secs = mission.route_seconds
            mission.get_logger().info(
                'ROUTE DURATION: %.1f s (%d min %04.1f s), goal accepted to '
                'goal reached' % (secs, int(secs // 60), secs % 60)
            )
        return 0
    except KeyboardInterrupt:
        mission.get_logger().warn('Ctrl+C received; stopping mission')
        mission.cancel_current_goal()
        return 130
    finally:
        # Never leave the planner blind to unmapped obstacles after we exit.
        if rclpy.ok():
            mission.restore_wall_clearance()
        restore_controller_params(speed_backup)
        if mission.obstacle_layer_disabled and rclpy.ok():
            mission.set_obstacle_layer(True)
        mission.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
