import json,time,math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
rclpy.init();n=Node('spec_motion_check',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)])
status={};position=[]
def st(m):status.update(json.loads(m.data))
def od(m):position[:]=[m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z]
n.create_subscription(String,'/morph/status',st,10);n.create_subscription(Odometry,'/morph/ground_truth/odom',od,qos_profile_sensor_data)
p=n.create_publisher(Twist,'/cmd_vel',10)
start=time.monotonic()
while time.monotonic()-start<60 and (not status.get('feedback_ready') or not position):rclpy.spin_once(n,timeout_sec=.05)
if not position:raise RuntimeError('No ground truth')
before=position.copy();initial=n.get_clock().now().nanoseconds*1e-9
while n.get_clock().now().nanoseconds*1e-9-initial<4 and time.monotonic()-start<120:
 rclpy.spin_once(n,timeout_sec=.02);msg=Twist();msg.linear.x=.1;p.publish(msg)
p.publish(Twist());after=position.copy()
result={'before':before,'after':after,'distance_m':math.dist(before,after),'estop':status.get('estop'),'contacts':status.get('wheel_contacts')}
result['passed']=result['distance_m']>.15 and not result['estop'];print(json.dumps(result,indent=2));n.destroy_node();rclpy.shutdown()
raise SystemExit(0 if result['passed'] else 1)
