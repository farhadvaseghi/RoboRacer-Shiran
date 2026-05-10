# MIT License

# Copyright (c) 2020 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Quaternion
from ackermann_msgs.msg import AckermannDriveStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

import gym
import numpy as np
from transforms3d import euler


def _normalize_angle(theta: float) -> float:
    """Wrap yaw to [-pi, pi]."""
    return float(np.arctan2(np.sin(theta), np.cos(theta)))


class GymBridge(Node):
    def __init__(self):
        super().__init__('gym_bridge')

        self.declare_parameter('ego_namespace')
        self.declare_parameter('ego_odom_topic')
        self.declare_parameter('ego_opp_odom_topic')
        self.declare_parameter('ego_scan_topic')
        self.declare_parameter('ego_drive_topic')
        self.declare_parameter('opp_namespace')
        self.declare_parameter('opp_odom_topic')
        self.declare_parameter('opp_ego_odom_topic')
        self.declare_parameter('opp_scan_topic')
        self.declare_parameter('opp_drive_topic')
        self.declare_parameter('scan_distance_to_base_link')
        self.declare_parameter('scan_fov')
        self.declare_parameter('scan_beams')
        self.declare_parameter('map_path')
        self.declare_parameter('map_img_ext')
        self.declare_parameter('num_agent')
        self.declare_parameter('sx')
        self.declare_parameter('sy')
        self.declare_parameter('stheta')
        self.declare_parameter('sx1')
        self.declare_parameter('sy1')
        self.declare_parameter('stheta1')
        self.declare_parameter('kb_teleop')

        # check num_agents
        num_agents = self.get_parameter('num_agent').value
        if num_agents < 1 or num_agents > 2:
            raise ValueError('num_agents should be either 1 or 2.')
        elif type(num_agents) != int:
            raise ValueError('num_agents should be an int.')

        # env backend
        self.env = gym.make('f110_gym:f110-v0',
                            map=self.get_parameter('map_path').value,
                            map_ext=self.get_parameter('map_img_ext').value,
                            num_agents=num_agents,
                            lidar_dist=self.get_parameter("scan_distance_to_base_link").value
                            )

        sx = self.get_parameter('sx').value
        sy = self.get_parameter('sy').value
        stheta = self.get_parameter('stheta').value
        self.ego_pose = [sx, sy, stheta]
        self.ego_start_pose = [sx, sy, stheta]
        self.ego_speed = [0.0, 0.0, 0.0]
        self.ego_requested_speed = 0.0
        self.ego_steer = 0.0
        self.ego_collision = False
        self.collision_grace_steps = 0  # steps remaining where done is ignored
        self._collision_count = 0
        self._last_slowdown_bucket = None
        self._collision_streak = 0
        self._last_collision_pose = None
        self._recovery_steps = 0
        self._recovery_speed = 0.0
        self._recovery_steer = 0.0
        self._recovery_escape_steer = 0.0
        self._recovery_phase = None
        # Rolling buffer of safe poses — used to back up before the wall on collision
        _buf = 250  # 250 steps × 0.01 s = 2.5 s look-back
        self._safe_poses = [[sx, sy, stheta] for _ in range(_buf)]
        self._safe_pose_idx = 0
        ego_scan_topic = self.get_parameter('ego_scan_topic').value
        ego_drive_topic = self.get_parameter('ego_drive_topic').value
        scan_fov = self.get_parameter('scan_fov').value
        scan_beams = self.get_parameter('scan_beams').value
        self.angle_min = -scan_fov / 2.
        self.angle_max = scan_fov / 2.
        self.angle_inc = scan_fov / scan_beams
        self.ego_namespace = self.get_parameter('ego_namespace').value
        ego_odom_topic = self.ego_namespace + '/' + self.get_parameter('ego_odom_topic').value
        self.scan_distance_to_base_link = self.get_parameter('scan_distance_to_base_link').value
        
        if num_agents == 2:
            self.has_opp = True
            self.opp_namespace = self.get_parameter('opp_namespace').value
            sx1 = self.get_parameter('sx1').value
            sy1 = self.get_parameter('sy1').value
            stheta1 = self.get_parameter('stheta1').value
            self.opp_pose = [sx1, sy1, stheta1]
            self.opp_speed = [0.0, 0.0, 0.0]
            self.opp_requested_speed = 0.0
            self.opp_steer = 0.0
            self.opp_collision = False
            self.obs, _ , self.done, _ = self.env.reset(np.array([[sx, sy, stheta], [sx1, sy1, stheta1]]))
            self.ego_scan = list(self.obs['scans'][0])
            self.opp_scan = list(self.obs['scans'][1])

            opp_scan_topic = self.get_parameter('opp_scan_topic').value
            opp_odom_topic = self.opp_namespace + '/' + self.get_parameter('opp_odom_topic').value
            opp_drive_topic = self.get_parameter('opp_drive_topic').value

            ego_opp_odom_topic = self.ego_namespace + '/' + self.get_parameter('ego_opp_odom_topic').value
            opp_ego_odom_topic = self.opp_namespace + '/' + self.get_parameter('opp_ego_odom_topic').value
        else:
            self.has_opp = False
            self.obs, _ , self.done, _ = self.env.reset(np.array([[sx, sy, stheta]]))
            self.ego_scan = list(self.obs['scans'][0])

        # sim physical step timer
        self.drive_timer = self.create_timer(0.01, self.drive_timer_callback)
        # topic publishing timer
        self.timer = self.create_timer(0.004, self.timer_callback)

        # transform broadcaster
        self.br = TransformBroadcaster(self)

        # Static transform: map → <ns>/odom  (identity — no drift in simulation)
        # This completes the REP-105 chain: map → odom → base_link
        self.static_br = StaticTransformBroadcaster(self)
        _ts = self.get_clock().now().to_msg()
        static_tfs = []
        _ego_map_odom = TransformStamped()
        _ego_map_odom.header.stamp = _ts
        _ego_map_odom.header.frame_id = 'map'
        _ego_map_odom.child_frame_id = self.ego_namespace + '/odom'
        _ego_map_odom.transform.rotation.w = 1.0
        static_tfs.append(_ego_map_odom)
        if self.has_opp:
            _opp_map_odom = TransformStamped()
            _opp_map_odom.header.stamp = _ts
            _opp_map_odom.header.frame_id = 'map'
            _opp_map_odom.child_frame_id = self.opp_namespace + '/odom'
            _opp_map_odom.transform.rotation.w = 1.0
            static_tfs.append(_opp_map_odom)
        self.static_br.sendTransform(static_tfs)

        # publishers
        self.ego_scan_pub = self.create_publisher(LaserScan, ego_scan_topic, 10)
        self.ego_odom_pub = self.create_publisher(Odometry, ego_odom_topic, 10)
        self.ego_drive_published = False
        if num_agents == 2:
            self.opp_scan_pub = self.create_publisher(LaserScan, opp_scan_topic, 10)
            self.ego_opp_odom_pub = self.create_publisher(Odometry, ego_opp_odom_topic, 10)
            self.opp_odom_pub = self.create_publisher(Odometry, opp_odom_topic, 10)
            self.opp_ego_odom_pub = self.create_publisher(Odometry, opp_ego_odom_topic, 10)
            self.opp_drive_published = False

        # subscribers
        self.ego_drive_sub = self.create_subscription(
            AckermannDriveStamped,
            ego_drive_topic,
            self.drive_callback,
            10)
        self.ego_reset_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.ego_reset_callback,
            10)
        if num_agents == 2:
            self.opp_drive_sub = self.create_subscription(
                AckermannDriveStamped,
                opp_drive_topic,
                self.opp_drive_callback,
                10)
            self.opp_reset_sub = self.create_subscription(
                PoseStamped,
                '/goal_pose',
                self.opp_reset_callback,
                10)

        if self.get_parameter('kb_teleop').value:
            self.teleop_sub = self.create_subscription(
                Twist,
                '/cmd_vel',
                self.teleop_callback,
                10)

    def _reset_control_state(self) -> None:
        """Clear commanded and recovery state after a simulator reset."""
        self.ego_requested_speed = 0.0
        self.ego_steer = 0.0
        self._last_slowdown_bucket = None
        self._recovery_steps = 0
        self._recovery_speed = 0.0
        self._recovery_steer = 0.0
        self._recovery_escape_steer = 0.0
        self._recovery_phase = None

    def _candidate_pose_is_safe(self, pose) -> bool:
        """Reject clearly bad recovery poses before feeding them back into the simulator."""
        if pose is None or len(pose) != 3:
            return False
        x, y, theta = pose
        if not np.isfinite(x) or not np.isfinite(y) or not np.isfinite(theta):
            return False
        if abs(theta) > 4.0 * np.pi:
            return False
        return True

    def _sanitize_pose(self, pose):
        """Return a pose with finite coordinates and normalized yaw."""
        if not self._candidate_pose_is_safe(pose):
            return list(self.ego_start_pose)
        x, y, theta = pose
        return [float(x), float(y), _normalize_angle(float(theta))]

    def _reset_env_with_pose(self, ego_pose, opp_pose=None) -> None:
        """Reset env and bridge state from a sanitized pose."""
        ego_pose = self._sanitize_pose(ego_pose)
        self._reset_control_state()
        if self.has_opp:
            if opp_pose is None:
                opp_pose = list(self.opp_pose)
            opp_pose = self._sanitize_pose(opp_pose)
            self.obs, _, self.done, _ = self.env.reset(np.array([ego_pose, opp_pose]))
        else:
            self.obs, _, self.done, _ = self.env.reset(np.array([ego_pose]))
        self._update_sim_state()

    def reset_to_start_pose(self) -> None:
        """Deterministically return the ego car to its configured start pose."""
        self._collision_streak = 0
        self._last_collision_pose = None
        self.collision_grace_steps = 0
        self._reset_env_with_pose(self.ego_start_pose, self.opp_pose if self.has_opp else None)


    def drive_callback(self, drive_msg):
        self.ego_requested_speed = drive_msg.drive.speed
        self.ego_steer = drive_msg.drive.steering_angle
        self.ego_drive_published = True

    def opp_drive_callback(self, drive_msg):
        self.opp_requested_speed = drive_msg.drive.speed
        self.opp_steer = drive_msg.drive.steering_angle
        self.opp_drive_published = True

    def ego_reset_callback(self, pose_msg):
        rx = pose_msg.pose.pose.position.x
        ry = pose_msg.pose.pose.position.y
        rqx = pose_msg.pose.pose.orientation.x
        rqy = pose_msg.pose.pose.orientation.y
        rqz = pose_msg.pose.pose.orientation.z
        rqw = pose_msg.pose.pose.orientation.w
        _, _, rtheta = euler.quat2euler([rqw, rqx, rqy, rqz], axes='sxyz')
        if self.has_opp:
            opp_pose = [self.obs['poses_x'][1], self.obs['poses_y'][1], self.obs['poses_theta'][1]]
            self._reset_env_with_pose([rx, ry, rtheta], opp_pose)
        else:
            self._reset_env_with_pose([rx, ry, rtheta])

    def opp_reset_callback(self, pose_msg):
        if self.has_opp:
            rx = pose_msg.pose.position.x
            ry = pose_msg.pose.position.y
            rqx = pose_msg.pose.orientation.x
            rqy = pose_msg.pose.orientation.y
            rqz = pose_msg.pose.orientation.z
            rqw = pose_msg.pose.orientation.w
            _, _, rtheta = euler.quat2euler([rqw, rqx, rqy, rqz], axes='sxyz')
            self._reset_env_with_pose(list(self.ego_pose), [rx, ry, rtheta])

    def teleop_callback(self, twist_msg):
        if not self.ego_drive_published:
            self.ego_drive_published = True

        self.ego_requested_speed = twist_msg.linear.x

        if twist_msg.angular.z > 0.0:
            self.ego_steer = 0.3
        elif twist_msg.angular.z < 0.0:
            self.ego_steer = -0.3
        else:
            self.ego_steer = 0.0

    def _forward_clearance(self) -> float:
        """Return the minimum LiDAR distance in the forward ±30° cone."""
        if not self.ego_scan:
            return 30.0
        n = len(self.ego_scan)
        mid = n // 2
        span = n // 9           # ±30° of 270° total FOV (~120 rays each side)
        lo = max(0, mid - span)
        hi = min(n, mid + span)
        valid = [r for r in self.ego_scan[lo:hi] if 0.05 < r < 29.0]
        return min(valid) if valid else 30.0

    def _forward_path_clearance(self, steer: float) -> float:
        """Return clearance along a narrower forward path biased by steering."""
        steer_ratio = max(-1.0, min(1.0, steer / 0.34))
        center_ratio = 0.5 + 0.10 * steer_ratio
        return self._sector_clearance(center_ratio, half_span_ratio=0.045)

    def _sector_clearance(self, center_ratio: float, half_span_ratio: float = 0.08) -> float:
        """Return the minimum LiDAR distance in a normalized scan sector."""
        if not self.ego_scan:
            return 30.0
        n = len(self.ego_scan)
        mid = int(center_ratio * n)
        span = max(1, int(half_span_ratio * n))
        lo = max(0, mid - span)
        hi = min(n, mid + span)
        valid = [r for r in self.ego_scan[lo:hi] if 0.05 < r < 29.0]
        return min(valid) if valid else 30.0

    def _left_clearance(self) -> float:
        return self._sector_clearance(0.75)

    def _right_clearance(self) -> float:
        return self._sector_clearance(0.25)

    def _rear_left_clearance(self) -> float:
        return self._sector_clearance(0.90, 0.06)

    def _rear_right_clearance(self) -> float:
        return self._sector_clearance(0.10, 0.06)

    def _rear_clearance(self) -> float:
        return min(self._rear_left_clearance(), self._rear_right_clearance())

    def _set_recovery_phase(self, phase: str, steps: int) -> None:
        self._recovery_phase = phase
        self._recovery_steps = steps
        if phase == 'reverse':
            self._recovery_speed = -0.45
            self._recovery_steer = 0.0
        else:
            self._recovery_speed = 0.8
            self._recovery_steer = self._recovery_escape_steer

    def _select_safe_pose(self):
        """Pick an older safe pose, looking further back if collisions repeat."""
        max_lookback = min(len(self._safe_poses) - 1, 40 + self._collision_streak * 35)
        step = 10
        for lookback in range(max_lookback, step - 1, -step):
            idx = (self._safe_pose_idx - lookback) % len(self._safe_poses)
            candidate = self._sanitize_pose(self._safe_poses[idx])
            if self._candidate_pose_is_safe(candidate):
                return candidate, lookback
        return list(self.ego_start_pose), max_lookback

    def _start_recovery(self) -> None:
        """Temporarily override control to escape from the wall after a reset."""
        left_clear = self._left_clearance()
        right_clear = self._right_clearance()
        front_clear = self._forward_clearance()

        # Forward motion steers toward the more open side. Reverse motion must
        # use the opposite steering sign because Ackermann yaw reverses in reverse.
        self._recovery_escape_steer = -0.34 if left_clear < right_clear else 0.34
        if front_clear < 0.45:
            self._set_recovery_phase('reverse', 45)
        else:
            self._set_recovery_phase('forward', 70)

        self.get_logger().info(
            'Recovery: phase={} speed={:.3f} steer={:.3f} front={:.3f} left={:.3f} right={:.3f} steps={}'.format(
                self._recovery_phase,
                self._recovery_speed,
                self._recovery_steer,
                front_clear,
                left_clear,
                right_clear,
                self._recovery_steps,
            )
        )

    def _recovery_control(self):
        """Return recovery override control until the car is back in safe space."""
        left_clear = self._left_clearance()
        right_clear = self._right_clearance()
        front_clear = self._forward_clearance()
        near_side = min(left_clear, right_clear)

        if self._recovery_phase == 'reverse':
            self._recovery_steps -= 1
            if self._recovery_steps <= 0 or front_clear > 0.60:
                self._set_recovery_phase('forward', 90)
            return self._recovery_speed, self._recovery_steer

        if self._recovery_phase == 'forward':
            self._recovery_steps -= 1
            if front_clear > 1.20 and near_side > 0.90:
                self._recovery_phase = None
                self._recovery_steps = 0
                self._collision_streak = 0
                self._last_slowdown_bucket = None
                self.get_logger().info(
                    'Recovery complete: front={:.3f} left={:.3f} right={:.3f}'.format(
                        front_clear, left_clear, right_clear
                    )
                )
                return None

            if self._recovery_steps <= 0:
                # If we still are not clear, retreat again and keep ownership.
                self._set_recovery_phase('reverse', 35)
                self.get_logger().info(
                    'Recovery reattempt: front={:.3f} left={:.3f} right={:.3f}'.format(
                        front_clear, left_clear, right_clear
                    )
                )
                return self._recovery_speed, self._recovery_steer

            self._recovery_speed = 0.8
            return self._recovery_speed, self._recovery_steer

        return None

    def _log_collision_debug(self, safe_pose) -> None:
        self._collision_count += 1
        clearance = self._forward_clearance()
        self.get_logger().warn(
            'Collision #{count}: pose=({px:.3f}, {py:.3f}, {pt:.3f}) '
            'speed=({vx:.3f}, {vy:.3f}, {wz:.3f}) '
            'cmd=(speed={cmd_v:.3f}, steer={cmd_s:.3f}) '
            'clearance={clr:.3f} streak={streak} -> reset_pose=({sx:.3f}, {sy:.3f}, {st:.3f})'.format(
                count=self._collision_count,
                px=self.ego_pose[0],
                py=self.ego_pose[1],
                pt=self.ego_pose[2],
                vx=self.ego_speed[0],
                vy=self.ego_speed[1],
                wz=self.ego_speed[2],
                cmd_v=self.ego_requested_speed,
                cmd_s=self.ego_steer,
                clr=clearance,
                streak=self._collision_streak,
                sx=safe_pose[0],
                sy=safe_pose[1],
                st=safe_pose[2],
            )
        )

    def drive_timer_callback(self):
        if self.collision_grace_steps > 0:
            self.collision_grace_steps -= 1
            self.done = False
        else:
            # Safe — record pose for look-back on future collision
            self._safe_poses[self._safe_pose_idx] = list(self.ego_pose)
            self._safe_pose_idx = (self._safe_pose_idx + 1) % len(self._safe_poses)

        if self.done:
            collision_xy = np.array(self.ego_pose[:2], dtype=float)
            if self._last_collision_pose is not None and \
               np.linalg.norm(collision_xy - self._last_collision_pose) < 0.75:
                self._collision_streak += 1
            else:
                self._collision_streak = 0
            self._last_collision_pose = collision_xy

            safe_pose, lookback = self._select_safe_pose()
            self._log_collision_debug(safe_pose)
            if self.has_opp:
                self.opp_requested_speed = 0.0
                self.opp_steer = 0.0
            self._reset_env_with_pose(safe_pose, self.opp_pose if self.has_opp else None)
            self.collision_grace_steps = 60  # ignore done for 60 steps while recovery runs
            self.get_logger().info(
                'Collision — reset to safe pose {:.2f} s before crash.'.format(lookback * 0.01)
            )
            self._start_recovery()
            return
        # Soft wall avoidance: reduce speed automatically when close to a forward obstacle.
        # This prevents full-speed wall hits that cause the rapid re-collision loop.
        # No hard stop — car always retains some forward mobility so it doesn't freeze near walls.
        recovery_cmd = None
        if self._recovery_phase is not None:
            recovery_cmd = self._recovery_control()

        if recovery_cmd is not None:
            applied_speed, applied_steer = recovery_cmd
        else:
            applied_speed = self.ego_requested_speed
            applied_steer = self.ego_steer

        if recovery_cmd is None and self.ego_requested_speed < 0:
            # Reverse should stay calmer than forward driving, but not feel
            # artificially stuck. Reverse steering is intentionally tiny because
            # the simulator dynamics become unstable with larger reverse turns.
            applied_steer = max(-0.030, min(0.030, applied_steer))
            reverse_turning = abs(applied_steer) > 0.001
            applied_speed = max(applied_speed, -0.25 if reverse_turning else -0.80)

        if recovery_cmd is None and self.ego_requested_speed != 0:
            left_clear = self._left_clearance()
            right_clear = self._right_clearance()
            command_is_reverse = self.ego_requested_speed < 0
            clearance = self._rear_clearance() if command_is_reverse else self._forward_path_clearance(applied_steer)
            _SLOW_DIST = 1.5 if command_is_reverse else 1.0
            _STOP_DIST = 0.55 if command_is_reverse else 0.28
            if clearance < _SLOW_DIST:
                span = max(0.05, _SLOW_DIST - _STOP_DIST)
                ratio = max(0.0, min(1.0, (clearance - _STOP_DIST) / span))
                factor = 0.15 + 0.85 * ratio
                bucket = int(clearance * 5.0)
                if not command_is_reverse and abs(applied_steer) < 0.05:
                    side_delta = right_clear - left_clear
                    near_side = min(left_clear, right_clear)
                    if near_side < 0.75 and abs(side_delta) > 0.08:
                        # Bias the car away from the nearest wall when the user
                        # is asking to go straight but the corridor is closing.
                        assist = -0.55 * side_delta
                        applied_steer = max(-0.22, min(0.22, assist))
                side_delta = right_clear - left_clear
                if bucket != self._last_slowdown_bucket:
                    self.get_logger().info(
                        'Wall slowdown: mode={} clearance={:.3f} m factor={:.3f} cmd_speed={:.3f} '
                        'left={:.3f} right={:.3f} pose=({:.3f}, {:.3f}, {:.3f}) steer={:.3f}'.format(
                            'reverse' if command_is_reverse else 'forward',
                            clearance,
                            factor,
                            self.ego_requested_speed,
                            left_clear,
                            right_clear,
                            self.ego_pose[0],
                            self.ego_pose[1],
                            self.ego_pose[2],
                            applied_steer,
                        )
                    )
                    self._last_slowdown_bucket = bucket
                if not command_is_reverse and abs(applied_steer) > 0.08 and clearance > 0.45:
                    steering_away_from_near_side = applied_steer * (right_clear - left_clear) < 0.0
                    if steering_away_from_near_side:
                        factor = max(factor, 0.78)
                    else:
                        factor = max(factor, 0.68)
                applied_speed = self.ego_requested_speed * factor
                if command_is_reverse:
                    applied_speed = max(applied_speed, -0.35)
                    if clearance < _STOP_DIST:
                        applied_speed = 0.0
                        applied_steer = 0.0
                if clearance < 0.22:
                    if command_is_reverse:
                        applied_speed = 0.0
                        applied_steer = 0.0
                    elif abs(applied_steer) < 0.08:
                        applied_speed = 0.0
                        if abs(side_delta) > 0.04:
                            assist = -0.80 * side_delta
                            applied_steer = max(-0.26, min(0.26, assist))
                        else:
                            applied_speed = -0.18
            else:
                self._last_slowdown_bucket = None

        if self.ego_drive_published and not self.has_opp:
            self.obs, _, self.done, _ = self.env.step(np.array([[applied_steer, applied_speed]]))
        elif self.has_opp and (self.ego_drive_published or self.opp_drive_published):
            ego_steer = applied_steer if self.ego_drive_published else 0.0
            ego_speed = applied_speed if self.ego_drive_published else 0.0
            opp_steer = self.opp_steer if self.opp_drive_published else 0.0
            opp_speed = self.opp_requested_speed if self.opp_drive_published else 0.0
            self.obs, _, self.done, _ = self.env.step(
                np.array([[ego_steer, ego_speed], [opp_steer, opp_speed]])
            )
        if not (
            np.all(np.isfinite(self.obs['poses_x']))
            and np.all(np.isfinite(self.obs['poses_y']))
            and np.all(np.isfinite(self.obs['poses_theta']))
        ):
            self.get_logger().warn('Simulator returned non-finite pose; resetting to start pose.')
            self._reset_env_with_pose(self.ego_start_pose, self.opp_pose if self.has_opp else None)
            return
        self._update_sim_state()

    def timer_callback(self):
        ts = self.get_clock().now().to_msg()

        # pub scans
        scan = LaserScan()
        scan.header.stamp = ts
        scan.header.frame_id = self.ego_namespace + '/laser'
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_inc
        scan.range_min = 0.
        scan.range_max = 30.
        scan.ranges = self.ego_scan
        self.ego_scan_pub.publish(scan)

        if self.has_opp:
            opp_scan = LaserScan()
            opp_scan.header.stamp = ts
            opp_scan.header.frame_id = self.opp_namespace + '/laser'
            opp_scan.angle_min = self.angle_min
            opp_scan.angle_max = self.angle_max
            opp_scan.angle_increment = self.angle_inc
            opp_scan.range_min = 0.
            opp_scan.range_max = 30.
            opp_scan.ranges = self.opp_scan
            self.opp_scan_pub.publish(opp_scan)

        # pub tf
        self._publish_odom(ts)
        self._publish_transforms(ts)
        self._publish_laser_transforms(ts)
        self._publish_wheel_transforms(ts)

    def _update_sim_state(self):
        self.ego_scan = list(self.obs['scans'][0])
        if self.has_opp:
            self.opp_scan = list(self.obs['scans'][1])
            self.opp_pose[0] = self.obs['poses_x'][1]
            self.opp_pose[1] = self.obs['poses_y'][1]
            self.opp_pose[2] = _normalize_angle(self.obs['poses_theta'][1])
            self.opp_speed[0] = self.obs['linear_vels_x'][1]
            self.opp_speed[1] = self.obs['linear_vels_y'][1]
            self.opp_speed[2] = self.obs['ang_vels_z'][1]

        self.ego_pose[0] = self.obs['poses_x'][0]
        self.ego_pose[1] = self.obs['poses_y'][0]
        self.ego_pose[2] = _normalize_angle(self.obs['poses_theta'][0])
        self.ego_speed[0] = self.obs['linear_vels_x'][0]
        self.ego_speed[1] = self.obs['linear_vels_y'][0]
        self.ego_speed[2] = self.obs['ang_vels_z'][0]

        

    def _publish_odom(self, ts):
        ego_odom = Odometry()
        ego_odom.header.stamp = ts
        ego_odom.header.frame_id = self.ego_namespace + '/odom'
        ego_odom.child_frame_id = self.ego_namespace + '/base_link'
        ego_odom.pose.pose.position.x = self.ego_pose[0]
        ego_odom.pose.pose.position.y = self.ego_pose[1]
        ego_quat = euler.euler2quat(0., 0., self.ego_pose[2], axes='sxyz')
        ego_odom.pose.pose.orientation.x = ego_quat[1]
        ego_odom.pose.pose.orientation.y = ego_quat[2]
        ego_odom.pose.pose.orientation.z = ego_quat[3]
        ego_odom.pose.pose.orientation.w = ego_quat[0]
        ego_odom.twist.twist.linear.x = self.ego_speed[0]
        ego_odom.twist.twist.linear.y = self.ego_speed[1]
        ego_odom.twist.twist.angular.z = self.ego_speed[2]
        self.ego_odom_pub.publish(ego_odom)

        if self.has_opp:
            opp_odom = Odometry()
            opp_odom.header.stamp = ts
            opp_odom.header.frame_id = self.opp_namespace + '/odom'
            opp_odom.child_frame_id = self.opp_namespace + '/base_link'
            opp_odom.pose.pose.position.x = self.opp_pose[0]
            opp_odom.pose.pose.position.y = self.opp_pose[1]
            opp_quat = euler.euler2quat(0., 0., self.opp_pose[2], axes='sxyz')
            opp_odom.pose.pose.orientation.x = opp_quat[1]
            opp_odom.pose.pose.orientation.y = opp_quat[2]
            opp_odom.pose.pose.orientation.z = opp_quat[3]
            opp_odom.pose.pose.orientation.w = opp_quat[0]
            opp_odom.twist.twist.linear.x = self.opp_speed[0]
            opp_odom.twist.twist.linear.y = self.opp_speed[1]
            opp_odom.twist.twist.angular.z = self.opp_speed[2]
            self.opp_odom_pub.publish(opp_odom)
            self.opp_ego_odom_pub.publish(ego_odom)
            self.ego_opp_odom_pub.publish(opp_odom)

    def _publish_transforms(self, ts):
        ego_t = Transform()
        ego_t.translation.x = self.ego_pose[0]
        ego_t.translation.y = self.ego_pose[1]
        ego_t.translation.z = 0.0
        ego_quat = euler.euler2quat(0.0, 0.0, self.ego_pose[2], axes='sxyz')
        ego_t.rotation.x = ego_quat[1]
        ego_t.rotation.y = ego_quat[2]
        ego_t.rotation.z = ego_quat[3]
        ego_t.rotation.w = ego_quat[0]

        ego_ts = TransformStamped()
        ego_ts.transform = ego_t
        ego_ts.header.stamp = ts
        ego_ts.header.frame_id = self.ego_namespace + '/odom'
        ego_ts.child_frame_id = self.ego_namespace + '/base_link'
        self.br.sendTransform(ego_ts)

        if self.has_opp:
            opp_t = Transform()
            opp_t.translation.x = self.opp_pose[0]
            opp_t.translation.y = self.opp_pose[1]
            opp_t.translation.z = 0.0
            opp_quat = euler.euler2quat(0.0, 0.0, self.opp_pose[2], axes='sxyz')
            opp_t.rotation.x = opp_quat[1]
            opp_t.rotation.y = opp_quat[2]
            opp_t.rotation.z = opp_quat[3]
            opp_t.rotation.w = opp_quat[0]

            opp_ts = TransformStamped()
            opp_ts.transform = opp_t
            opp_ts.header.stamp = ts
            opp_ts.header.frame_id = self.opp_namespace + '/odom'
            opp_ts.child_frame_id = self.opp_namespace + '/base_link'
            self.br.sendTransform(opp_ts)

    def _publish_wheel_transforms(self, ts):
        ego_wheel_ts = TransformStamped()
        ego_wheel_quat = euler.euler2quat(0., 0., self.ego_steer, axes='sxyz')
        ego_wheel_ts.transform.rotation.x = ego_wheel_quat[1]
        ego_wheel_ts.transform.rotation.y = ego_wheel_quat[2]
        ego_wheel_ts.transform.rotation.z = ego_wheel_quat[3]
        ego_wheel_ts.transform.rotation.w = ego_wheel_quat[0]
        ego_wheel_ts.header.stamp = ts
        ego_wheel_ts.header.frame_id = self.ego_namespace + '/front_left_hinge'
        ego_wheel_ts.child_frame_id = self.ego_namespace + '/front_left_wheel'
        self.br.sendTransform(ego_wheel_ts)
        ego_wheel_ts.header.frame_id = self.ego_namespace + '/front_right_hinge'
        ego_wheel_ts.child_frame_id = self.ego_namespace + '/front_right_wheel'
        self.br.sendTransform(ego_wheel_ts)

        if self.has_opp:
            opp_wheel_ts = TransformStamped()
            opp_wheel_quat = euler.euler2quat(0., 0., self.opp_steer, axes='sxyz')
            opp_wheel_ts.transform.rotation.x = opp_wheel_quat[1]
            opp_wheel_ts.transform.rotation.y = opp_wheel_quat[2]
            opp_wheel_ts.transform.rotation.z = opp_wheel_quat[3]
            opp_wheel_ts.transform.rotation.w = opp_wheel_quat[0]
            opp_wheel_ts.header.stamp = ts
            opp_wheel_ts.header.frame_id = self.opp_namespace + '/front_left_hinge'
            opp_wheel_ts.child_frame_id = self.opp_namespace + '/front_left_wheel'
            self.br.sendTransform(opp_wheel_ts)
            opp_wheel_ts.header.frame_id = self.opp_namespace + '/front_right_hinge'
            opp_wheel_ts.child_frame_id = self.opp_namespace + '/front_right_wheel'
            self.br.sendTransform(opp_wheel_ts)

    def _publish_laser_transforms(self, ts):
        ego_scan_ts = TransformStamped()
        ego_scan_ts.transform.translation.x = self.scan_distance_to_base_link
        # ego_scan_ts.transform.translation.z = 0.04+0.1+0.025
        ego_scan_ts.transform.rotation.w = 1.
        ego_scan_ts.header.stamp = ts
        ego_scan_ts.header.frame_id = self.ego_namespace + '/base_link'
        ego_scan_ts.child_frame_id = self.ego_namespace + '/laser'
        self.br.sendTransform(ego_scan_ts)

        if self.has_opp:
            opp_scan_ts = TransformStamped()
            opp_scan_ts.transform.translation.x = self.scan_distance_to_base_link
            opp_scan_ts.transform.rotation.w = 1.
            opp_scan_ts.header.stamp = ts
            opp_scan_ts.header.frame_id = self.opp_namespace + '/base_link'
            opp_scan_ts.child_frame_id = self.opp_namespace + '/laser'
            self.br.sendTransform(opp_scan_ts)

def main(args=None):
    rclpy.init(args=args)
    gym_bridge = GymBridge()
    rclpy.spin(gym_bridge)

if __name__ == '__main__':
    main()
