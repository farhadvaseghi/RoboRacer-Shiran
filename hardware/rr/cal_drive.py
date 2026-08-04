import rclpy, time, math, sys
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

TARGET=float(sys.argv[1]) if len(sys.argv)>1 else 2.0; SPEED=0.3; TIMEOUT=16.0; MAXD=TARGET+0.6
rclpy.init(); n=Node('cal_drive')
pub=n.create_publisher(AckermannDriveStamped,'/drive',10)
st={'x':None,'y':None}
n.create_subscription(Odometry,'/odom',lambda m:st.update(x=m.pose.pose.position.x,y=m.pose.pose.position.y),10)
t=time.time()
while st['x'] is None and time.time()-t<5: rclpy.spin_once(n,timeout_sec=0.1)
if st['x'] is None: print("NO /odom"); raise SystemExit
x0,y0=st['x'],st['y']
print(f"start odom=({x0:.3f},{y0:.3f})  driving forward at {SPEED} m/s until odom=={TARGET} m ...")
def drive(s):
    m=AckermannDriveStamped(); m.header.stamp=n.get_clock().now().to_msg()
    m.drive.speed=float(s); m.drive.steering_angle=0.0; pub.publish(m)
d=0.0; t0=time.time()
try:
    while time.time()-t0<TIMEOUT:
        rclpy.spin_once(n,timeout_sec=0.02)
        d=math.hypot(st['x']-x0, st['y']-y0)
        if d>=TARGET or d>=MAXD: break
        drive(SPEED)
finally:
    for _ in range(20): drive(0.0); time.sleep(0.02)
print(f"STOPPED. ODOM distance traveled = {d:.3f} m   (elapsed {time.time()-t0:.1f}s)")
print(">>> Now measure the ACTUAL distance with the tape and report it. <<<")
rclpy.shutdown()
