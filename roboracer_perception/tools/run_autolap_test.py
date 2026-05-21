#!/usr/bin/env python3
"""Drive one automatic lap on the RoboRacer solid oval track.

Usage:
    source ~/roboracer_ws/install/setup.bash
    python3 ~/roboracer_ws/src/roboracer_perception/tools/run_autolap_test.py

This script assumes the simulator is already running, for example:
    ros2 launch f1tenth_gym_ros gym_bridge_solid.launch.py
"""

import math
import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


WHEELBASE = 0.3302
LOOKAHEAD = 1.2
STRAIGHT_SPEED = 1.0
TURN_SPEED = 0.8
STEER_LIMIT = 0.34
CONTROL_DT = 0.05
RETURN_DIST_THRESH = 1.0
RETURN_YAW_THRESH = 0.5
MIN_LAP_TIME = 20.0
MAX_LAP_TIME = 90.0


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class AutoLapDriver(Node):
    def __init__(self):
        super().__init__('auto_lap_driver')
        self._drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self._reset_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self._odom_sub = self.create_subscription(
            Odometry, '/ego_racecar/odom', self._odom_callback, 10
        )
        self.pose = None

    def _odom_callback(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw,
        )

    def publish_drive(self, speed: float, steer: float) -> None:
        msg = AckermannDriveStamped()
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steer)
        self._drive_pub.publish(msg)

    def stop(self, repeats: int = 20) -> None:
        for _ in range(repeats):
            self.publish_drive(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.02)

    def reset_to_start(self) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose.orientation.w = 1.0
        for _ in range(25):
            self._reset_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.02)
        self.stop(repeats=10)


def pure_pursuit_command(x: float, y: float, yaw: float) -> tuple[float, float]:
    """Track the oval centerline with a simple piecewise pure-pursuit policy."""
    if x < 20.0 and y < 2.5:
        tx = min(20.0, x + LOOKAHEAD)
        ty = 0.0
        target_speed = STRAIGHT_SPEED
    elif x >= 20.0 and y <= 5.0:
        cx, cy, radius = 20.0, 2.5, 2.5
        ang = math.atan2(y - cy, x - cx)
        ang = max(-math.pi / 2.0, min(math.pi / 2.0, ang + LOOKAHEAD / radius))
        tx = cx + radius * math.cos(ang)
        ty = cy + radius * math.sin(ang)
        target_speed = TURN_SPEED
    elif x > 0.0 and y >= 2.5:
        tx = max(0.0, x - LOOKAHEAD)
        ty = 5.0
        target_speed = STRAIGHT_SPEED
    else:
        cx, cy, radius = 0.0, 2.5, 2.5
        ang = math.atan2(y - cy, x - cx)
        if ang < 0.0:
            ang += 2.0 * math.pi
        ang = max(math.pi / 2.0, min(3.0 * math.pi / 2.0, ang + LOOKAHEAD / radius))
        tx = cx + radius * math.cos(ang)
        ty = cy + radius * math.sin(ang)
        target_speed = TURN_SPEED

    dx = tx - x
    dy = ty - y
    local_x = math.cos(-yaw) * dx - math.sin(-yaw) * dy
    local_y = math.sin(-yaw) * dx + math.cos(-yaw) * dy

    if local_x <= 1e-6:
        return target_speed, 0.0

    curvature = 2.0 * local_y / (LOOKAHEAD ** 2)
    steer = math.atan(WHEELBASE * curvature)
    steer = max(-STEER_LIMIT, min(STEER_LIMIT, steer))
    return target_speed, steer


def wait_for_odom(node: AutoLapDriver, timeout_sec: float = 8.0) -> None:
    deadline = time.time() + timeout_sec
    while node.pose is None and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.pose is None:
        raise RuntimeError('No odometry received from /ego_racecar/odom')


def main() -> None:
    rclpy.init()
    node = AutoLapDriver()
    try:
        wait_for_odom(node)
        node.reset_to_start()
        time.sleep(0.5)
        wait_for_odom(node)

        start_pose = node.pose
        lap_start = time.time()
        returned = False

        while time.time() - lap_start < MAX_LAP_TIME:
            x, y, yaw = node.pose
            speed, steer = pure_pursuit_command(x, y, yaw)
            node.publish_drive(speed, steer)
            rclpy.spin_once(node, timeout_sec=CONTROL_DT)

            if time.time() - lap_start > MIN_LAP_TIME:
                x, y, yaw = node.pose
                dist = math.hypot(x - start_pose[0], y - start_pose[1])
                yaw_err = abs(normalize_angle(yaw - start_pose[2]))
                if dist < RETURN_DIST_THRESH and yaw_err < RETURN_YAW_THRESH:
                    returned = True
                    break

        node.stop()

        x, y, yaw = node.pose
        dist = math.hypot(x - start_pose[0], y - start_pose[1])
        yaw_err = normalize_angle(yaw - start_pose[2])

        print('Start pose:', tuple(round(v, 3) for v in start_pose))
        print('End pose  :', (round(x, 3), round(y, 3), round(yaw, 3)))
        print('Returned  :', returned)
        print('Error     :', {'distance_m': round(dist, 3), 'yaw_rad': round(yaw_err, 3)})
    finally:
        node.stop(repeats=5)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
