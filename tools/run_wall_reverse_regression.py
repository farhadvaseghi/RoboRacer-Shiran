#!/usr/bin/env python3
"""Exercise near-wall forward driving and reverse behavior on the solid oval.

Usage:
    source ~/roboracer_ws/install/setup.bash
    python3 ~/roboracer_ws/src/roboracer_perception/tools/run_wall_reverse_regression.py

Assumes the simulator is already running.
"""

import math
import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


CONTROL_DT = 0.05
STEER_LIMIT = 0.34


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class RegressionDriver(Node):
    def __init__(self):
        super().__init__('wall_reverse_regression')
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

    def wait_for_pose(self, timeout_sec: float = 8.0) -> None:
        deadline = time.time() + timeout_sec
        while self.pose is None and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.pose is None:
            raise RuntimeError('No odometry received from /ego_racecar/odom')

    def publish_drive(self, speed: float, steer: float) -> None:
        msg = AckermannDriveStamped()
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(max(-STEER_LIMIT, min(STEER_LIMIT, steer)))
        self._drive_pub.publish(msg)

    def hold_command(self, speed: float, steer: float, duration: float) -> tuple[float, float, float]:
        deadline = time.time() + duration
        while time.time() < deadline:
            self.publish_drive(speed, steer)
            rclpy.spin_once(self, timeout_sec=CONTROL_DT)
        return self.pose

    def stop(self, repeats: int = 20) -> None:
        for _ in range(repeats):
            self.publish_drive(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.02)

    def reset_to_start(self) -> tuple[float, float, float]:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose.orientation.w = 1.0
        for _ in range(25):
            self._reset_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.02)
        self.stop(repeats=10)
        time.sleep(0.3)
        self.wait_for_pose()
        return self.pose


def run_near_wall_forward(driver: RegressionDriver) -> dict:
    start = driver.reset_to_start()
    driver.hold_command(1.3, 0.22, 4.7)
    before = driver.hold_command(0.9, 0.0, 1.2)
    after = driver.hold_command(0.9, 0.0, 1.8)
    driver.stop()
    return {
        'start_pose': tuple(round(v, 3) for v in start),
        'pre_wall_pose': tuple(round(v, 3) for v in before),
        'end_pose': tuple(round(v, 3) for v in after),
        'yaw_delta': round(normalize_angle(after[2] - before[2]), 3),
        'distance_m': round(math.hypot(after[0] - before[0], after[1] - before[1]), 3),
    }


def run_reverse_escape(driver: RegressionDriver) -> dict:
    start = driver.reset_to_start()
    driver.hold_command(1.0, 0.24, 4.6)
    tight_pose = driver.hold_command(0.6, 0.0, 0.9)
    reverse_start = driver.pose
    reverse_end = driver.hold_command(-0.8, 0.0, 2.2)
    driver.stop()
    return {
        'start_pose': tuple(round(v, 3) for v in start),
        'tight_pose': tuple(round(v, 3) for v in tight_pose),
        'reverse_end_pose': tuple(round(v, 3) for v in reverse_end),
        'reverse_distance_m': round(
            math.hypot(reverse_end[0] - reverse_start[0], reverse_end[1] - reverse_start[1]), 3
        ),
        'reverse_yaw_delta': round(normalize_angle(reverse_end[2] - reverse_start[2]), 3),
    }


def main() -> None:
    rclpy.init()
    driver = RegressionDriver()
    try:
        driver.wait_for_pose()
        forward_result = run_near_wall_forward(driver)
        reverse_result = run_reverse_escape(driver)
        print('SCENARIO1', forward_result)
        print('SCENARIO_REVERSE', reverse_result)
    finally:
        driver.stop(repeats=5)
        driver.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
