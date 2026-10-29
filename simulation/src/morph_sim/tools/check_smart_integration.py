#!/usr/bin/env python3
"""Live range-driven expansion using an imported NIST ramp, then stale-data stop."""
import json,subprocess,time,math,xml.etree.ElementTree as ET
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
rclpy.init();n=Node('smart_integration_check',parameter_overrides=[Parameter('use_sim_time',value=True)]);s={};sense={};odom=[]
def read(target,m):target.update(json.loads(m.data))
n.create_subscription(String,'/morph/status',lambda m:read(s,m),1);n.create_subscription(String,'/morph/perception/decision',lambda m:read(sense,m),1)
def pose(m):odom[:]=[m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z]
n.create_subscription(Odometry,'/morph/ground_truth/odom',pose,qos_profile_sensor_data)
ev=n.create_publisher(String,'/morph/event',1);pub=n.create_publisher(Twist,'/cmd_vel',1)
def wait(seconds):
 start=n.get_clock().now().nanoseconds*1e-9;wall=time.monotonic()
 while n.get_clock().now().nanoseconds*1e-9-start<seconds:
  if time.monotonic()-wall>max(30,seconds*4):raise RuntimeError('No clock')
  rclpy.spin_once(n,timeout_sec=.01);pub.publish(Twist())
def gz(service,reqtype,req):
 r=subprocess.run(['gz','service','-s',service,'--reqtype',reqtype,'--reptype','gz.msgs.Boolean','--timeout','4000','--req',req],capture_output=True,text=True,timeout=6)
 if 'true' not in r.stdout:raise RuntimeError(r.stdout+r.stderr)
wall=time.monotonic()
while (not s.get('feedback_ready') or not sense) and time.monotonic()-wall<60:rclpy.spin_once(n,timeout_sec=.02)
results={}
try:
 if not s.get('feedback_ready'):raise RuntimeError('Controller feedback unavailable')
 ev.publish(String(data='reset'));ev.publish(String(data='normal'));wait(2)
 gz('/world/leo_marsyard/set_pose','gz.msgs.Pose','name: "morph" position: {x: 0 y: -3 z: 0.38} orientation: {w: 1}')
 wait(2);results['flat_before']=sense.copy()
 p=Path(get_package_share_directory('morph_sim'));tree=ET.parse(p/'models/nist_simple_ramp_120/model.sdf');model=tree.find('model');model.set('name','perception_validation_ramp');ET.SubElement(model,'pose').text='1.4 -3 0 0 0 -1.57079632679'
 for mesh in model.findall('.//mesh'):
  el=mesh.find('scale');el.text=f'1 {2/1.18} {2*math.tan(math.radians(12))/1.2}'
 for mat in model.findall('.//visual'):
  m=ET.SubElement(mat,'material');ET.SubElement(m,'diffuse').text='.4 .4 .4 1'
 xml=ET.tostring(tree.getroot(),encoding='unicode');req='sdf: '+json.dumps(xml)+' allow_renaming: false'
 gz('/world/leo_marsyard/create','gz.msgs.EntityFactory',req);wait(1.2);results['ramp_observed']=sense.copy()
 ev.publish(String(data='perception_toggle'));wait(6);results['auto_status']=s.copy();results['expanded_without_shape_command']=s.get('perception_auto') and s.get('target_stage_offset',0)>.015 and s.get('stage_offset',0)>.014
 ev.publish(String(data='normal'));wait(.3);results['manual_override_disables_auto']=not s.get('perception_auto')
 results['passed']=bool(results['ramp_observed'].get('valid') and results['ramp_observed'].get('mode')=='climb' and results['expanded_without_shape_command'] and results['manual_override_disables_auto'])
 print(json.dumps(results,indent=2));Path('validation/smart_perception_integration.json').write_text(json.dumps(results,indent=2))
finally:
 try:gz('/world/leo_marsyard/remove','gz.msgs.Entity','name: "perception_validation_ramp" type: MODEL')
 except Exception:pass
 pub.publish(Twist());n.destroy_node();rclpy.shutdown()
raise SystemExit(0 if results.get('passed') else 1)
