#!/usr/bin/env python3
"""Record response, combined motion, morphology, contact and real-time factor."""
import json,time,math
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
rclpy.init();n=Node('research_motion_check',parameter_overrides=[Parameter('use_sim_time',value=True)])
s={};p={};o=[];heat={};samples=[]
def read(dst,msg):dst.update(json.loads(msg.data))
n.create_subscription(String,'/morph/status',lambda m:read(s,m),1);n.create_subscription(String,'/morph/physics/status',lambda m:read(p,m),1);n.create_subscription(String,'/morph/suspension/telemetry',lambda m:read(heat,m),1)
def odom(m):o[:]=[m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z]
n.create_subscription(Odometry,'/morph/ground_truth/odom',odom,qos_profile_sensor_data)
pub=n.create_publisher(Twist,'/cmd_vel',1);event=n.create_publisher(String,'/morph/event',10)
def sim():return n.get_clock().now().nanoseconds*1e-9
def run(duration,v=(0,0,0),name='settle'):
 start=sim();wall=time.monotonic();response=None;begin=o.copy();beginshape=s.get('stage_offset',0);maxspeed=0.;trajectory=[]
 while sim()-start<duration:
  if time.monotonic()-wall>80:raise RuntimeError('Clock stalled')
  msg=Twist();msg.linear.x=float(v[0]);msg.linear.y=float(v[1]);msg.angular.z=float(v[2]);pub.publish(msg);rclpy.spin_once(n,timeout_sec=.005)
  actual=s.get('actual_speed') or 0.;maxspeed=max(maxspeed,actual)
  if actual>.03 and response is None:response={'wall_s':time.monotonic()-wall,'sim_s':sim()-start}
  if len(trajectory)==0 or sim()-trajectory[-1]['sim_s']>.1:trajectory.append({'sim_s':sim(),'position':o.copy(),'speed':actual,'stage':s.get('stage_offset',0),'contacts':s.get('wheel_contacts',{})})
 result={'phase':name,'duration_sim_s':sim()-start,'rtf':(sim()-start)/(time.monotonic()-wall),'response':response,'start':begin,'end':o.copy(),'stage_start':beginshape,'stage_end':s.get('stage_offset',0),'peak_speed_m_s':maxspeed,'estop':s.get('estop'),'trajectory':trajectory,'power':p.get('power'),'damper':heat.copy()};samples.append(result);print(json.dumps({k:v for k,v in result.items() if k not in ('trajectory','power','damper')}),flush=True)
wall=time.monotonic()
while (not s.get('feedback_ready') or not o) and time.monotonic()-wall<60:rclpy.spin_once(n,timeout_sec=.02)
try:
 if not s.get('feedback_ready'):raise RuntimeError('No feedback')
 run(2.);event.publish(String(data='reset'));run(2.,(.15,0,0),'forward_response');run(1.)
 event.publish(String(data='expand'));event.publish(String(data='raise'));run(7.,(.12,.04,.10),'combined_drive_and_expand');run(2.,name='brake')
 event.publish(String(data='estop'));run(.5,(.2,0,0),'emergency_stop')
 Path('validation/research_motion.json').write_text(json.dumps(samples,indent=2))
finally:
 pub.publish(Twist());n.destroy_node();rclpy.shutdown()
