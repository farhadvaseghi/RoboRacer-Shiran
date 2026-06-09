import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class PathRelay(Node):
    def __init__(self):
        super().__init__('path_relay')

        output_qos = QoSProfile(depth=1)
        output_qos.reliability = ReliabilityPolicy.RELIABLE
        output_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._publisher = self.create_publisher(Path, '/control/plan', output_qos)
        self._subscription = self.create_subscription(
            Path, '/plan', self._path_callback, 10)
        self.get_logger().info('Relaying /plan to retained topic /control/plan')

    def _path_callback(self, msg: Path) -> None:
        self._publisher.publish(msg)
        self.get_logger().info(
            f'Retained path with {len(msg.poses)} poses for the controller')


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
