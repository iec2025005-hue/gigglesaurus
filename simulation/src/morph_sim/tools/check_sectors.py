#!/usr/bin/env python3
"""Runtime checks: teleport to each safe entrance, settle, then drive 0.1m/s.

Run in the same ROS/Gazebo environment as the launch. This verifies entrances,
not complete obstacle traversal or calibrated soft-soil behavior.
"""
import json,math,subprocess,time
from pathlib import Path
import os,hashlib
os.environ['GZ_PARTITION']=os.environ.get('MORPH_GZ_PARTITION','morph_'+hashlib.sha256(str(Path.cwd()).encode()).hexdigest()[:12])
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
from morph_sim.course import Course

rclpy.init();node=Node('sector_acceptance',parameter_overrides=[Parameter('use_sim_time',value=True)])
status={};pos=[];physics={}
node.create_subscription(String,'/morph/status',lambda m:status.update(json.loads(m.data)),10)
node.create_subscription(String,'/morph/physics/status',lambda m:physics.update(json.loads(m.data)),10)
def odom(m):pos[:]=[m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z]
node.create_subscription(Odometry,'/morph/ground_truth/odom',odom,qos_profile_sensor_data)
commands=node.create_publisher(Twist,'/cmd_vel',10);events=node.create_publisher(String,'/morph/event',10)
def wait_sim(seconds,speed=0):
 start=node.get_clock().now().nanoseconds*1e-9;wall=time.monotonic()
 while node.get_clock().now().nanoseconds*1e-9-start<seconds:
  if time.monotonic()-wall>90:raise RuntimeError('Simulation clock stalled')
  rclpy.spin_once(node,timeout_sec=.02);m=Twist();m.linear.x=float(speed);commands.publish(m)
 commands.publish(Twist())
wall=time.monotonic()
while (not status.get('feedback_ready') or not pos) and time.monotonic()-wall<60:rclpy.spin_once(node,timeout_sec=.05)
results=[]
try:
 for sector in Course(get_package_share_directory('morph_sim')).sectors:
  wait_sim(.5);x,y,z=sector.spawn
  req=f'name: "morph" position: {{x: {x} y: {y} z: {z}}} orientation: {{z: 0.70710678 w: 0.70710678}}'
  result=subprocess.run(['gz','service','-s','/world/leo_marsyard/set_pose','--reqtype','gz.msgs.Pose','--reptype','gz.msgs.Boolean','--timeout','3000','--req',req],capture_output=True,text=True,timeout=6)
  if 'true' not in result.stdout:raise RuntimeError('Pose reset failed: '+result.stdout+result.stderr)
  wait_sim(2.);events.publish(String(data='reset'));wait_sim(.5)
  before=pos.copy();contacts=status.get('wheel_contacts',{}).copy();wait_sim(2.,.10);after=pos.copy();wait_sim(.4)
  travel=math.hypot(after[0]-before[0],after[1]-before[1])
  record={'sector':sector.name,'settled_position_m':before,'contacts':contacts,'travel_m':travel,'estop':status.get('estop'),'detected_sector':physics.get('zone')}
  record['passed']=all(contacts.get(n,False) for n in ['fl','fr','rl','rr']) and travel>.08 and not record['estop'] and abs(before[2]-.29)<.15
  results.append(record);print(json.dumps(record),flush=True)
finally:
 commands.publish(Twist());node.destroy_node();rclpy.shutdown()
Path('validation/sector_checks.json').write_text(json.dumps({'scope':'Safe entrance settling and two-second drive; not full lane traversal','passed':all(r['passed'] for r in results),'sectors':results},indent=2))
raise SystemExit(0 if all(r['passed'] for r in results) else 1)
