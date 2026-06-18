import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class PathRelay(Node):
    def __init__(self):
        super().__init__('path_relay')

        self.declare_parameter('use_forward_oval_route', True)
        self.declare_parameter('segment_length', 3.0)
        self.declare_parameter('waypoint_spacing', 0.20)
        self.declare_parameter('goal_reached_threshold', 0.35)
        self.declare_parameter('bottom_y', -0.60)
        self.declare_parameter('top_y', 4.40)
        self.declare_parameter('left_x', 0.0)
        self.declare_parameter('right_x', 20.0)
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('raw_plan_topic', '/nav2/plan_raw')

        self._use_forward_oval_route = (
            self.get_parameter('use_forward_oval_route').value)
        self._segment_length = float(self.get_parameter('segment_length').value)
        self._waypoint_spacing = float(self.get_parameter('waypoint_spacing').value)
        self._goal_reached_threshold = float(
            self.get_parameter('goal_reached_threshold').value)
        self._bottom_y = float(self.get_parameter('bottom_y').value)
        self._top_y = float(self.get_parameter('top_y').value)
        self._left_x = float(self.get_parameter('left_x').value)
        self._right_x = float(self.get_parameter('right_x').value)
        self._raw_plan_topic = self.get_parameter('raw_plan_topic').value
        self._center_y = 0.5 * (self._bottom_y + self._top_y)
        self._radius = 0.5 * (self._top_y - self._bottom_y)
        self._straight_length = self._right_x - self._left_x
        self._loop_length = 2.0 * self._straight_length + 2.0 * math.pi * self._radius

        output_qos = QoSProfile(depth=1)
        output_qos.reliability = ReliabilityPolicy.RELIABLE
        output_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._plan_publisher = self.create_publisher(Path, '/plan', output_qos)
        self._publisher = self.create_publisher(Path, '/control/plan', output_qos)
        self._route_publisher = self.create_publisher(
            Path,
            '/control/forward_route',
            output_qos,
        )
        self._plan_sub = self.create_subscription(Path, self._raw_plan_topic, self._path_callback, 10)
        self._odom_sub = self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self._odom_callback,
            10,
        )
        self._timer = self.create_timer(0.10, self._timer_callback)

        self._ego_pose = None
        self._route_points = []
        self._route_frame = 'map'
        self._last_goal = None
        self._progress_idx = 0

        mode = 'forward oval segmenter' if self._use_forward_oval_route else 'retained relay'
        self.get_logger().info(
            f'Relaying {self._raw_plan_topic} to /plan and /control/plan using {mode}; '
            f'publishing full route on /control/forward_route; '
            f'segment_length={self._segment_length:.1f} m')

    def _odom_callback(self, msg: Odometry) -> None:
        self._ego_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _path_callback(self, msg: Path) -> None:
        if not msg.poses:
            self._route_points = []
            self._progress_idx = 0
            self._publish_empty_paths(msg.header.frame_id or 'map')
            self.get_logger().warn('Received empty raw plan; cleared active segmented route')
            return

        if not self._use_forward_oval_route:
            self._plan_publisher.publish(msg)
            self._publisher.publish(msg)
            self._route_publisher.publish(msg)
            self.get_logger().info(
                f'Retained full path with {len(msg.poses)} poses for the controller')
            return

        if self._ego_pose is None:
            self.get_logger().warn('Ignoring /plan until ego odometry is available')
            return

        goal_pose = msg.poses[-1].pose.position
        goal = (goal_pose.x, goal_pose.y)

        if self._last_goal is not None:
            if self._distance(goal, self._last_goal) < 0.15 and self._route_points:
                return

        self._last_goal = goal
        self._route_frame = msg.header.frame_id or 'map'
        self._route_points = self._build_forward_oval_route(self._ego_pose, goal)
        self._progress_idx = 0

        self.get_logger().info(
            f'Built segmented forward route to goal=({goal[0]:.2f}, {goal[1]:.2f}) '
            f'with {len(self._route_points)} points')
        self._publish_full_route()
        self._publish_current_segment()

    def _timer_callback(self) -> None:
        if self._ego_pose is None or self._last_goal is None:
            return

        self._route_frame = self._route_frame or 'map'
        self._route_points = self._build_forward_oval_route(self._ego_pose, self._last_goal)
        self._progress_idx = self._nearest_route_index(self._ego_pose)
        self._publish_full_route()

        if not self._route_points:
            return

        final_goal = self._route_points[-1]
        if self._distance(self._ego_pose, final_goal) <= self._goal_reached_threshold:
            self._route_points = []
            self._progress_idx = 0
            self._publish_empty_paths(self._route_frame)
            self.get_logger().info('Segmented route complete')
            return

        self._publish_current_segment()

    def _publish_current_segment(self) -> None:
        if not self._route_points:
            return

        start_idx = min(self._progress_idx, len(self._route_points) - 1)
        end_idx = start_idx
        distance = 0.0

        while end_idx + 1 < len(self._route_points) and distance < self._segment_length:
            distance += self._distance(self._route_points[end_idx], self._route_points[end_idx + 1])
            end_idx += 1

        if end_idx <= start_idx and start_idx + 1 < len(self._route_points):
            end_idx = start_idx + 1

        path = self._make_path(self._route_points[start_idx:end_idx + 1])
        self._publisher.publish(path)

    def _publish_full_route(self) -> None:
        if self._route_points:
            path = self._make_path(self._route_points)
            self._plan_publisher.publish(path)
            self._route_publisher.publish(path)

    def _publish_empty_paths(self, frame_id: str) -> None:
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = frame_id
        self._plan_publisher.publish(path)
        self._publisher.publish(path)
        self._route_publisher.publish(path)

    def _make_path(self, points: list[tuple[float, float]]) -> Path:
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self._route_frame

        for x, y in points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        return path

    def _build_forward_oval_route(self, start: tuple[float, float], goal: tuple[float, float]):
        start_s = self._project_to_loop_s(*start)
        goal_s = self._project_to_loop_s(*goal)
        forward_distance = (goal_s - start_s) % self._loop_length

        if forward_distance < self._goal_reached_threshold:
            return [start, goal]

        points = []
        count = max(1, int(math.ceil(forward_distance / self._waypoint_spacing)))
        for i in range(count + 1):
            s = start_s + min(forward_distance, i * self._waypoint_spacing)
            points.append(self._point_at_s(s))

        if self._distance(points[-1], goal) > self._waypoint_spacing:
            points.append(goal)
        else:
            points[-1] = goal

        return points

    def _project_to_loop_s(self, x: float, y: float) -> float:
        sample_step = 0.10
        sample_count = int(math.ceil(self._loop_length / sample_step))
        best_s = 0.0
        best_dist = float('inf')

        for i in range(sample_count):
            s = i * sample_step
            px, py = self._point_at_s(s)
            dist = (x - px) ** 2 + (y - py) ** 2
            if dist < best_dist:
                best_dist = dist
                best_s = s

        return best_s

    def _point_at_s(self, s: float) -> tuple[float, float]:
        s = s % self._loop_length
        straight = self._straight_length
        arc = math.pi * self._radius

        if s <= straight:
            return self._left_x + s, self._bottom_y

        s -= straight
        if s <= arc:
            theta = -0.5 * math.pi + s / self._radius
            return (
                self._right_x + self._radius * math.cos(theta),
                self._center_y + self._radius * math.sin(theta),
            )

        s -= arc
        if s <= straight:
            return self._right_x - s, self._top_y

        s -= straight
        theta = 0.5 * math.pi + s / self._radius
        return (
            self._left_x + self._radius * math.cos(theta),
            self._center_y + self._radius * math.sin(theta),
        )

    def _nearest_route_index(self, point: tuple[float, float]) -> int:
        if not self._route_points:
            return 0

        start = max(0, self._progress_idx - 5)
        best_idx = start
        best_dist = float('inf')

        for idx in range(start, len(self._route_points)):
            dist = self._distance(point, self._route_points[idx])
            if dist < best_dist:
                best_dist = dist
                best_idx = idx

        return best_idx


    @staticmethod
    def _distance(a: tuple[float, float], b: tuple[float, float]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])


def main(args=None):
    rclpy.init(args=args)
    node = PathRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
