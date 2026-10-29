import json, math, time
import rclpy
from . import lifecycle
from .trajectory_limits import PositionTrajectoryLimiter
from .timing import timed_callback,decode_command
import socket
from .safety import valid_joint_sample,fresh_snapshot,ACTUATED_JOINTS
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64MultiArray, Float64, String
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from ros_gz_interfaces.msg import Contacts
from .control import AdaptationCore, LEGS, STAGES, ARMS, D_MIN, D_MAX, approach, clamp
from .terrain_zones import zone_at
from .course import Course
from .design_physics import load_parameters
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

class TerrainManager(Node):
 def __init__(self):
  super().__init__('terrain_adaptation_manager')
  self.declare_parameter('position_backend','forward');self.position_backend=self.get_parameter('position_backend').value
  self.declare_parameter('force_height_servo',False);self.declare_parameter('robot_urdf_file','');self.declare_parameter('controllers_file','');self.declare_parameter('position_filter',True);self.declare_parameter('trace_positions',False)
  self.position_limiter=PositionTrajectoryLimiter(self.get_parameter('robot_urdf_file').value,self.get_parameter('controllers_file').value,bool(self.get_parameter('force_height_servo').value))
  self.declare_parameter('params_file','')
  self.safety=load_parameters(self.get_parameter('params_file').value or Path(get_package_share_directory('morph_sim'))/'config/params.yaml')['control_safety']
  self.declare_parameter('wheel_radius',.075)
  self.declare_parameter('automatic_terrain',False)
  self.declare_parameter('arena','adaptive')
  self.declare_parameter('perception_auto',False)
  self.perception_enabled=bool(self.get_parameter('perception_auto').value);self.perception={};self.perception_time=-1e9
  self.course=Course(get_package_share_directory('morph_sim')) if self.get_parameter('arena').value=='multisector' else None
  self.core=AdaptationCore(float(self.get_parameter('wheel_radius').value))
  self.automatic_terrain=bool(self.get_parameter('automatic_terrain').value)
  self.core.auto=self.automatic_terrain or self.perception_enabled
  self.zone=None;self.position=None
  self.cmd=[0.,0.,0.];self.cmd_time=-1e9;self.cmd_wall=-1e9;self.joint_time=-1e9;self.joint_wall=-1e9;self.imu_time=0.;self.states={};self.last=self.get_clock().now().nanoseconds*1e-9;self.hit=False
  self.height_cmd=[0.]*4;self.joint_velocities={};self.wheel_velocities={};self.actual_speed=0.;self.odom_time=0.
  self.manual={};self.callback_timing={};self.command_source=None;self.transition_source=None;self.command_sequence=0;self.last_source_sequences={};self.command_receipt_wall_ns=0;self.command_receipt_sim_ns=0
  self.command_publishers={key:self.create_publisher(Float64MultiArray,'/'+key+'_controller/commands',1) for key in ['drive','steering','height','shape']}
  self.trajectory_joint_names={'steering':[n+'_steer_joint' for n in LEGS],'height':[n+'_height_joint' for n in LEGS],'shape':list(STAGES+ARMS)}
  self.trajectory_publishers={key:self.create_publisher(JointTrajectory,'/'+key+'_controller/joint_trajectory',1) for key in self.trajectory_joint_names} if self.position_backend=='trajectory' else {}
  self.controller_discovery_ready=False;self.last_discovery_wall=-1e9
  self.last_sim=None
  self.status=self.create_publisher(String,'/morph/status',1)
  self.last_status=-1e9
  self.kpub=self.create_publisher(Float64,'/morph/suspension/stiffness',10);self.cpub=self.create_publisher(Float64,'/morph/suspension/damping',10)
  self.create_subscription(Twist,'/cmd_vel',self.on_cmd,1)
  self.create_subscription(String,'/morph/cmd_vel_timed',self.on_timed_cmd,1)
  self.create_subscription(Odometry,'/morph/ground_truth/odom',self.on_odom,qos_profile_sensor_data)
  self.create_subscription(String,'/morph/event',self.on_event,10)
  self.create_subscription(String,'/morph/perception/decision',self.on_perception,1)
  self.create_subscription(String,'/morph/manual_joint',self.on_manual,10)
  self.create_subscription(JointState,'/joint_states',self.on_joints,qos_profile_sensor_data)
  self.create_subscription(Imu,'/morph/imu',self.on_imu,qos_profile_sensor_data)
  self.create_subscription(Contacts,'/morph/contacts/underbody',self.on_underbody,qos_profile_sensor_data)
  self.wheel_contact_time={n:0. for n in LEGS}
  for n in LEGS:self.create_subscription(Contacts,'/morph/contacts/'+n,lambda m,n=n:self.on_contact(n,m),qos_profile_sensor_data)
  self.timer=self.create_timer(.01,self.tick)
  self.last_feedback_warning=0.
 def on_odom(self,m):
  v=m.twist.twist.linear
  if all(math.isfinite(x) for x in (v.x,v.y,v.z)):self.actual_speed=math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z);self.odom_time=self.get_clock().now().nanoseconds*1e-9
  p=m.pose.pose.position
  if math.isfinite(p.x) and math.isfinite(p.y):self.position=(p.x,p.y)
 def on_perception(self,m):
  try:
   data=json.loads(m.data)
   if data.get('valid') and (data.get('mode') not in ('track','climb','rocky') or not math.isfinite(data.get('speed_limit_m_s',float('nan')))):return
   self.perception=data;self.perception_time=self.get_clock().now().nanoseconds*1e-9
  except (ValueError,TypeError):pass
 def on_event(self,m):
  if m.data=='perception_toggle':
   self.perception_enabled=not self.perception_enabled
   if self.perception_enabled:self.automatic_terrain=False
   self.core.auto=self.perception_enabled or self.automatic_terrain;return
  if m.data in ('compact','normal','expand','track','grass','mud','rocky','climb','sand','debris','snow','mixed'):
   self.perception_enabled=False;self.core.auto=self.automatic_terrain

  if self.automatic_terrain and m.data in ('compact','normal','expand','track','grass','mud','rocky','climb','sand','debris','snow','mixed','auto'):
   self.get_logger().info('Terrain automation owns morphing; drive using cmd_vel.')
   return
  if m.data=='flip' and (self.get_clock().now().nanoseconds*1e-9-self.joint_time>.3 or len(self.wheel_velocities)!=4 or max([abs(v) for v in self.wheel_velocities.values()]+[0.])>.5):return
  self.core.event(m.data)
 def on_manual(self,m):
  try:
   command=json.loads(m.data);name=command['joint'];value=float(command['value'])
   duration=clamp(float(command.get('duration',10.)),.1,30.)
   if not math.isfinite(value) or not math.isfinite(duration):raise ValueError('non-finite command')
   if name=='shape_offset':
    if not D_MIN<=value<=D_MAX:raise ValueError('shape offset outside limits')
    self.core.target_d=value;return
   if name in STAGES:low,high=D_MIN,D_MAX
   elif name in ARMS:low,high=-.25,.25
   elif name in [n+'_steer_joint' for n in LEGS]:low,high=-math.pi/2,math.pi/2
   elif name in [n+'_height_joint' for n in LEGS]:low,high=-.015,.015
   elif name in [n+'_wheel_joint' for n in LEGS]:low,high=-12.,12.
   else:raise ValueError('joint is absent or passive')
   if not low<=value<=high:raise ValueError('target outside joint limits')
   self.manual[name]=(value,self.get_clock().now().nanoseconds*1e-9+duration)
  except (ValueError,KeyError,TypeError) as exc:
   self.get_logger().warning('Manual joint command rejected: '+str(exc))
 def accept_command(self,values,source=None):
  if not all(math.isfinite(x) for x in values):return
  now=self.get_clock().now().nanoseconds;wall=time.monotonic_ns();self.command_sequence+=1
  stamp={'manager_sequence':self.command_sequence,'source':source,'manager_receipt_wall_ns':wall,'manager_receipt_sim_ns':now}
  if values!=self.cmd:self.transition_source=stamp
  self.command_source=stamp;self.command_receipt_wall_ns=wall;self.command_receipt_sim_ns=now
  self.cmd=values;self.cmd_time=(source['source_sim_ns'] if source else now)*1e-9
  self.cmd_wall=source['source_wall_ns']*1e-9 if source and source.get('host')==socket.gethostname() else wall*1e-9
 @timed_callback
 def on_cmd(self,m):self.accept_command([m.linear.x,m.linear.y,m.angular.z])
 @timed_callback
 def on_timed_cmd(self,m):
  try:
   data=decode_command(m.data,self.get_clock().now().nanoseconds,time.monotonic_ns(),self.safety['command_timeout_s'],self.safety['future_stamp_tolerance_s'])
   if data['sequence']<=self.last_source_sequences.get(data['source_id'],-1):return
   self.last_source_sequences[data['source_id']]=data['sequence'];self.accept_command(data['values'],data)
  except (ValueError,TypeError,KeyError,json.JSONDecodeError):return
 @timed_callback
 def on_joints(self,m):
  now=self.get_clock().now().nanoseconds*1e-9;stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
  if not valid_joint_sample(m.name,m.position,m.velocity,stamp,now,self.safety['state_timeout_s'],self.safety['future_stamp_tolerance_s']):
   self.states={};self.wheel_velocities={};return
  self.states=dict(zip(m.name,m.position));self.joint_time=stamp;self.joint_wall=time.monotonic()
  self.joint_velocities=dict(zip(m.name,m.velocity))
  self.wheel_velocities={n:v for n,v in zip(m.name,m.velocity) if n.endswith('_wheel_joint') and math.isfinite(v)}
 def on_imu(self,m):
  q=m.orientation;norm=math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
  if not math.isfinite(norm) or norm<.5:return
  x,y,z,w=q.x/norm,q.y/norm,q.z/norm,q.w/norm
  self.core.roll=math.atan2(2*(w*x+y*z),1-2*(x*x+y*y));self.core.pitch=math.asin(max(-1.,min(1.,2*(w*y-z*x))));self.imu_time=self.get_clock().now().nanoseconds*1e-9
 def on_underbody(self,m):
  if m.contacts:self.hit=True
 def on_contact(self,n,m):
  if m.contacts:self.wheel_contact_time[n]=self.get_clock().now().nanoseconds*1e-9
 @timed_callback
 def tick(self):
  now=self.get_clock().now().nanoseconds*1e-9;sim=self.get_clock().now().nanoseconds*1e-9
  if self.automatic_terrain and self.position is not None and now-self.odom_time<.5:
   zone=self.course.at(*self.position) if self.course else zone_at(*self.position,self.zone)
   if zone is not self.zone:
    self.zone=zone
    if zone:self.get_logger().info('Entering '+zone.name+' zone; requesting '+zone.mode+' morphology')
   if zone and (self.core.mode!=zone.mode) and (now-self.imu_time>.3 or max(abs(self.core.roll),abs(self.core.pitch))<.20):
    self.core.event(zone.mode)
  dt=0. if self.last_sim is None else min(.05,max(0.,sim-self.last_sim));self.last_sim=sim
  wall=time.monotonic()
  if not self.controller_discovery_ready and wall-self.last_discovery_wall>=.1:
   self.last_discovery_wall=wall
   receivers={key:key+'_controller' for key in ('steering','height','shape')}
   self.controller_discovery_ready=all(any(endpoint.node_name==name for endpoint in self.get_subscriptions_info_by_topic('/'+key+'_controller/'+('joint_trajectory' if self.position_backend=='trajectory' else 'commands'))) for key,name in receivers.items())
  feedback=self.states if self.controller_discovery_ready and fresh_snapshot(self.joint_time,self.joint_wall,now,time.monotonic(),self.safety['state_timeout_s'],self.safety['future_stamp_tolerance_s']) else {}
  if not feedback and now-self.last_feedback_warning>5.:
   self.get_logger().warning('No fresh joint states: wheel commands are held at zero. Check joint_state_broadcaster and the Gazebo controller plugin.')
   self.last_feedback_warning=now
  command=list(self.cmd) if fresh_snapshot(self.cmd_time,self.cmd_wall,now,time.monotonic(),self.safety['command_timeout_s'],self.safety['future_stamp_tolerance_s']) else [0.,0.,0.]
  if self.perception_enabled:
   if now-self.perception_time<.5 and self.perception.get('valid'):
    mode=self.perception['mode']
    if mode!=self.core.mode:self.core.event(mode)
    cap=clamp(self.perception['speed_limit_m_s'],0.,.3);speed=math.hypot(*command[:2])
    if speed>cap:
     command[0]*=cap/speed;command[1]*=cap/speed
   else:command=[0.,0.,0.]
  out=self.core.step(dt,command,feedback,self.hit,now-self.imu_time<.3);self.hit=False
  if out.status['feedback_ready']:
   for i in range(4):self.height_cmd[i]=approach(self.height_cmd[i],out.heights[i],.01*dt)
  commands=dict(drive=out.wheels,steering=out.steering,height=self.height_cmd,shape=out.shape)
  for name,(value,expiry) in list(self.manual.items()):
   if now>=expiry or self.core.estop:
    del self.manual[name];continue
   if name in STAGES+ARMS and max(abs(v) for v in self.core.v)<.03:
    commands['shape'][(STAGES+ARMS).index(name)]=value
   elif name.endswith('_steer_joint'):
    commands['steering'][LEGS.index(name[:2])]=value
   elif name.endswith('_height_joint'):
    commands['height'][LEGS.index(name[:2])]=value
   elif name.endswith('_wheel_joint') and out.status['feedback_ready']:
    commands['drive'][LEGS.index(name[:2])]=value
  trace=[]
  if out.status['feedback_ready']:
   names={'steering':[n+'_steer_joint' for n in LEGS],'height':[n+'_height_joint' for n in LEGS],'shape':list(STAGES+ARMS)}
   requested={name:value for key,joints in names.items() for name,value in zip(joints,commands[key])}
   bounded,trace=self.position_limiter.apply(requested,feedback,dt)
   if self.get_parameter('position_filter').value:
    for key,joints in names.items():commands[key]=[bounded[name] for name in joints]
   else:
    for row in trace:
     row['filtered_candidate_position']=row['published_position'];row['published_position']=row['requested_position']
     row['command_velocity']=(row['published_position']-row['previous_position'])/min(dt,self.position_limiter.period) if dt>0 else 0.
     row['lead_conflict']=abs(row['published_position']-row['measured_position'])>row['lead_limit']
    self.position_limiter.previous.update(requested)
   for row in trace:row['measured_velocity']=self.joint_velocities.get(row['joint'])
   out.status['steering_published_degrees']=[math.degrees(v) for v in commands['steering']]
   out.status['steering_measured_degrees']=[math.degrees(feedback[n+'_steer_joint']) for n in LEGS]
   out.status['position_command_limits']={'filter_enabled':bool(self.get_parameter('position_filter').value),'clamped_axes':sum(row['clamped'] for row in trace),'lead_conflicts':sum(row['lead_conflict'] for row in trace),'max_tracking_error':max([abs(row['tracking_error']) for row in trace]+[0.])}
   if self.get_parameter('trace_positions').value:out.status['position_command_trace']=trace
  actuator_publication_wall_ns=time.monotonic_ns()
  if out.status['feedback_ready'] and self.transition_source is not None and 'first_actuator_publication_wall_ns' not in self.transition_source:
   self.transition_source['first_actuator_publication_wall_ns']=actuator_publication_wall_ns;self.transition_source['first_actuator_publication_sim_ns']=self.get_clock().now().nanoseconds
  for key,values in commands.items():
   if not out.status['feedback_ready']:
    if key=='drive':self.command_publishers[key].publish(Float64MultiArray(data=[0.]*4))
    continue
   self.command_publishers[key].publish(Float64MultiArray(data=values))
   if key in self.trajectory_publishers:
    trajectory=JointTrajectory();trajectory.joint_names=self.trajectory_joint_names[key]
    horizon=self.safety['trajectory_horizon_s'];ns=round(horizon*1e9)
    trajectory.points=[JointTrajectoryPoint(positions=list(values),time_from_start=Duration(sec=ns//10**9,nanosec=ns%10**9))]
    self.trajectory_publishers[key].publish(trajectory)
  if out.status['feedback_ready']:
   self.kpub.publish(Float64(data=float(out.status['stiffness'])));self.cpub.publish(Float64(data=float(out.status['damping'])))
  out.status['position_backend']=self.position_backend
  out.status['timing']={'command':self.command_source,'last_transition':self.transition_source,'actuator_publication_wall_ns':actuator_publication_wall_ns,'actuator_publication_sim_ns':self.get_clock().now().nanoseconds,'joint_sample_sim_s':self.joint_time,'joint_receipt_wall_s':self.joint_wall,'callback_execution':self.callback_timing,'scope':'Application callback durations exclude executor waiting; source monotonic comparisons require the same host'}
  out.status['actual_speed']=self.actual_speed if now-self.odom_time<.3 else None
  out.status['wheel_contacts']={n:now-t<.25 for n,t in self.wheel_contact_time.items()};out.status['roll_deg']=math.degrees(self.core.roll);out.status['pitch_deg']=math.degrees(self.core.pitch);out.status['command_fresh']=fresh_snapshot(self.cmd_time,self.cmd_wall,now,time.monotonic(),self.safety['command_timeout_s'],self.safety['future_stamp_tolerance_s'])
  out.status['input_age_sim_ms']=max(0.,1000*(now-self.cmd_time))
  out.status['manual_joints']=list(self.manual)
  out.status['terrain_zone']=self.zone.name if self.zone else None
  out.status['automatic_terrain']=self.automatic_terrain
  out.status['perception_auto']=self.perception_enabled
  out.status['perception']=dict(self.perception,fresh=now-self.perception_time<.5)
  if self.perception_enabled and now-self.perception_time>=.5:out.status['perception']['reason']='range decision stale; auto drive held'
  if now-self.last_status>=.05:
   self.last_status=now;self.status.publish(String(data=json.dumps(out.status)))

def main():
 lifecycle.initialize();node=TerrainManager()
 try:lifecycle.spin(node)
 except KeyboardInterrupt:pass
 finally:
  if rclpy.ok():node.command_publishers['drive'].publish(Float64MultiArray(data=[0.]*4))
  node.destroy_node();
  if rclpy.ok():rclpy.shutdown()
