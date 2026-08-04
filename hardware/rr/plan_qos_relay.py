import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Path

class Relay(Node):
    def __init__(self):
        super().__init__('plan_qos_relay')
        out_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        in_qos = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                            reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.VOLATILE)
        self.pub = self.create_publisher(Path, '/control/plan', out_qos)
        self.create_subscription(Path, '/plan', self.cb, in_qos)
        self.get_logger().info('plan_qos_relay: /plan (volatile) -> /control/plan (transient_local)')
    def cb(self, msg):
        self.pub.publish(msg)

rclpy.init(); rclpy.spin(Relay()); rclpy.shutdown()
