import rclpy, sys, math
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
import time

x = float(sys.argv[1]) if len(sys.argv)>1 else 0.0
y = float(sys.argv[2]) if len(sys.argv)>2 else 0.0
yaw_deg = float(sys.argv[3]) if len(sys.argv)>3 else 0.0
yaw = math.radians(yaw_deg)

class Seed(Node):
    def __init__(self):
        super().__init__("seed_pose")
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        qos.history = QoSHistoryPolicy.KEEP_LAST
        self.pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", qos)

def main():
    rclpy.init(); n = Seed()
    end = time.time()+2.0
    while time.time()<end: rclpy.spin_once(n, timeout_sec=0.1)
    m = PoseWithCovarianceStamped(); m.header.frame_id="map"
    m.pose.pose.position.x=x; m.pose.pose.position.y=y
    m.pose.pose.orientation.z=math.sin(yaw/2); m.pose.pose.orientation.w=math.cos(yaw/2)
    cov=[0.0]*36; cov[0]=0.05; cov[7]=0.05; cov[35]=0.03; m.pose.covariance=cov
    for _ in range(6):
        m.header.stamp=n.get_clock().now().to_msg(); n.pub.publish(m)
        rclpy.spin_once(n, timeout_sec=0.1); time.sleep(0.25)
    print(f"SEEDED /initialpose at ({x},{y}) yaw={yaw_deg}deg")
    time.sleep(0.5); n.destroy_node(); rclpy.shutdown()
main()
