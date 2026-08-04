import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
import time

class Seed(Node):
    def __init__(self):
        super().__init__("seed_origin")
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        qos.history = QoSHistoryPolicy.KEEP_LAST
        self.pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", qos)

def main():
    rclpy.init()
    n = Seed()
    end = time.time() + 2.0
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.1)
    m = PoseWithCovarianceStamped()
    m.header.frame_id = "map"
    m.pose.pose.orientation.w = 1.0
    cov = [0.0]*36
    cov[0] = 0.1; cov[7] = 0.1; cov[35] = 0.05
    m.pose.covariance = cov
    for _ in range(6):
        m.header.stamp = n.get_clock().now().to_msg()
        n.pub.publish(m)
        rclpy.spin_once(n, timeout_sec=0.1)
        time.sleep(0.25)
    print("SEEDED /initialpose at origin (0,0,0 yaw0) TRANSIENT_LOCAL")
    time.sleep(0.5)
    n.destroy_node()
    rclpy.shutdown()

main()
