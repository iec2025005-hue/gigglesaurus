"""Runtime check to run on Ubuntu after launch. Does not certify physics accuracy."""
import json, math, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, Imu
from rosgraph_msgs.msg import Clock
from ros_gz_interfaces.msg import Contacts
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from .control import STAGES, ARMS, LEGS

def main():
 rclpy.init();n=Node('morph_runtime_smoke_test');data={'clock':[],'states':{},'imu':False,'wheel_contacts':set(),'status':{}}
 def js(m):data['states']={name:value for name,value in zip(m.name,m.position)}
 def imu(m):data['imu']=all(math.isfinite(v) for v in (m.orientation.x,m.orientation.y,m.orientation.z,m.orientation.w))
 def clock(m):data['clock'].append(m.clock.sec+m.clock.nanosec*1e-9)
 def status(m):
  try:data['status']=json.loads(m.data)
  except ValueError:pass
 n.create_subscription(JointState,'/joint_states',js,qos_profile_sensor_data);n.create_subscription(Imu,'/morph/imu',imu,qos_profile_sensor_data);n.create_subscription(Clock,'/clock',clock,qos_profile_sensor_data);n.create_subscription(String,'/morph/status',status,10)
 for leg in LEGS:
  def contacts(m,leg=leg):
   if m.contacts:data['wheel_contacts'].add(leg)
  n.create_subscription(Contacts,'/morph/contacts/'+leg,contacts,qos_profile_sensor_data)
 pub=n.create_publisher(Twist,'/cmd_vel',10)
 start=time.monotonic();required=set(STAGES+ARMS+tuple(leg+suffix for leg in LEGS for suffix in ('_height_joint','_spring_joint','_steer_joint','_wheel_joint')))
 passed=False
 try:
  while time.monotonic()-start<60 and rclpy.ok():
   pub.publish(Twist());rclpy.spin_once(n,timeout_sec=.05)
   finite=required.issubset(data['states']) and all(math.isfinite(data['states'][k]) for k in required)
   if len(data['clock'])>10 and data['clock'][-1]>data['clock'][0]+2 and finite and data['imu'] and len(data['wheel_contacts'])==4 and data['status'].get('feedback_ready'):
    passed=True;break
  sag={leg:data['states'].get(leg+'_spring_joint') for leg in LEGS}
  print(json.dumps({'passed':passed,'clock_advancing':len(data['clock'])>1 and data['clock'][-1]>data['clock'][0],'missing_joints':sorted(required-data['states'].keys()),'imu_received':data['imu'],'wheels_seen_in_contact':sorted(data['wheel_contacts']),'spring_positions_m':sag,'scope':'Topic/connectivity/finite-state check; no terrain performance or stability rating.'},indent=2))
 finally:
  pub.publish(Twist());n.destroy_node()
  if rclpy.ok():rclpy.shutdown()
 sys.exit(0 if passed else 1)
