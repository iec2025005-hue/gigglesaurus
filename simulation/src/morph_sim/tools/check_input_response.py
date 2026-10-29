#!/usr/bin/env python3
"""Paired acceleration-feedforward ablation using fresh 50Hz body odometry."""
import time,json,math
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter as Param,ParameterValue,ParameterType
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
rclpy.init();n=Node('input_response_check',parameter_overrides=[Parameter('use_sim_time',value=True)]);pub=n.create_publisher(Twist,'/cmd_vel',1);event=n.create_publisher(String,'/morph/event',1);start=None;simstart=None;response=None;speed=0.;records=[]
def odom(m):
 global response,speed
 speed=math.hypot(m.twist.twist.linear.x,m.twist.twist.linear.y)
 stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
 if start is not None and response is None and stamp>=simstart and speed>.03:response=(time.monotonic()-start,stamp-simstart)
n.create_subscription(Odometry,'/morph/ground_truth/odom',odom,qos_profile_sensor_data)
client=n.create_client(SetParameters,'/morph_interactions/set_parameters')
if not client.wait_for_service(timeout_sec=30):raise RuntimeError('No motor model service')
def setting(enabled):
 req=SetParameters.Request(parameters=[Param(name='acceleration_feedforward',value=ParameterValue(type=ParameterType.PARAMETER_BOOL,bool_value=enabled))]);f=client.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=3)
 if not f.done() or not all(x.successful for x in f.result().results):raise RuntimeError('Parameter change rejected')
def run(seconds,v=0.):
 wall=time.monotonic()
 while time.monotonic()-wall<seconds:
  m=Twist();m.linear.x=float(v);pub.publish(m);rclpy.spin_once(n,timeout_sec=.003)
try:
 event.publish(String(data='reset'));event.publish(String(data='normal'));run(6)
 for trial in range(12):
  start=None;setting(bool(trial%2));run(.8)
  response=None;simstart=n.get_clock().now().nanoseconds*1e-9;start=time.monotonic();direction=1 if (trial//2)%2==0 else -1
  run(.7,.2*direction);pub.publish(Twist());start=None
  record={'trial':trial,'feedforward':bool(trial%2),'direction':direction,'response_wall_ms':None if response is None else response[0]*1000,'response_sim_ms':None if response is None else response[1]*1000};records.append(record);print(json.dumps(record),flush=True)
 setting(True);run(1.)
 result={'scope':'50 Hz ground-truth speed >0.03 m/s; command publication excludes human key timing','trials':records}
 for enabled in [False,True]:
  values=[r['response_wall_ms'] for r in records if r['feedforward']==enabled and r['response_wall_ms'] is not None];result['feedforward_on' if enabled else 'feedforward_off']={'n':len(values),'median_wall_ms':float(np.median(values)),'max_wall_ms':max(values)}
 Path('validation/input_response_comparison.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
finally:
 pub.publish(Twist());n.destroy_node();rclpy.shutdown()
