#!/usr/bin/env python3
"""Live moving sensor-loss and controlled-stop checks; clears faults afterward."""
import json,time,math
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter as Param,ParameterValue,ParameterType
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
rclpy.init();n=Node('smart_stop_check',parameter_overrides=[Parameter('use_sim_time',value=True)]);state={};speed=0.;pose=[]
def status(m):state.update(json.loads(m.data))
def odom(m):
 global speed
 speed=math.hypot(m.twist.twist.linear.x,m.twist.twist.linear.y);pose[:]=[m.pose.pose.position.x,m.pose.pose.position.y]
n.create_subscription(String,'/morph/status',status,1);n.create_subscription(Odometry,'/morph/ground_truth/odom',odom,qos_profile_sensor_data)
pub=n.create_publisher(Twist,'/cmd_vel',1);ev=n.create_publisher(String,'/morph/event',1);client=n.create_client(SetParameters,'/morph_local_perception/set_parameters')
if not client.wait_for_service(timeout_sec=30):raise RuntimeError('Perception node unavailable')
def setting(enabled):
 f=client.call_async(SetParameters.Request(parameters=[Param(name='processing_enabled',value=ParameterValue(type=ParameterType.PARAMETER_BOOL,bool_value=enabled))]));rclpy.spin_until_future_complete(n,f,timeout_sec=3)
 if not f.done() or not all(r.successful for r in f.result().results):raise RuntimeError('Parameter update failed')
def run(duration,v=0.):
 start=n.get_clock().now().nanoseconds*1e-9;wall=time.monotonic()
 while n.get_clock().now().nanoseconds*1e-9-start<duration:
  if time.monotonic()-wall>duration*5+10:raise RuntimeError('Clock stalled')
  m=Twist();m.linear.x=float(v);pub.publish(m);rclpy.spin_once(n,timeout_sec=.005)
results={}
try:
 run(1);ev.publish(String(data='reset'));ev.publish(String(data='normal'));run(5)
 ev.publish(String(data='perception_toggle'));run(2,.15);results['moving_before_sensor_loss']=speed
 setting(False);run(1.5,.15);results['sensor_loss']={'speed_m_s':speed,'state':dict(state)}
 results['sensor_loss_stops']=speed<.02 and state.get('perception_auto') and not state.get('perception',{}).get('fresh',True)
 setting(True);run(.5);ev.publish(String(data='normal'));run(1.5,.2);results['moving_before_stop']=speed;before=pose.copy();ev.publish(String(data='estop'));run(.8,.2)
 results['controlled_stop']={'speed_m_s':speed,'distance_m':math.dist(before,pose),'latched':state.get('estop')};results['moving_stop_passed']=speed<.02 and state.get('estop') and results['moving_before_stop']>.12
 results['passed']=bool(results['sensor_loss_stops'] and results['moving_stop_passed']);Path('validation/smart_stop.json').write_text(json.dumps(results,indent=2));print(json.dumps(results,indent=2))
 ev.publish(String(data='reset'));run(.5)
finally:
 setting(True);pub.publish(Twist());n.destroy_node();rclpy.shutdown()
raise SystemExit(0 if results.get('passed') else 1)
