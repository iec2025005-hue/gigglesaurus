#!/usr/bin/env python3
"""Repeated command-to-observed-motion trials; all raw timestamps retained.

Observation times include transport/executor delay; they are not hardware latency.
Missing responses are failures, never dropped from acceptance statistics.
"""
import json,math,os,socket,time,uuid
from pathlib import Path
from sequence_gaps import SequenceGaps
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from sensor_msgs.msg import JointState,Image,Imu,CameraInfo
from nav_msgs.msg import Odometry

class Bench(Node):
 def __init__(self):
  super().__init__('response_trials',parameter_overrides=[Parameter('use_sim_time',value=True)])
  self.pub=self.create_publisher(String,'/morph/cmd_vel_timed',1);self.events=self.create_publisher(String,'/morph/event',10)
  self.physics={};self.xy=None;self.feedback_failures=0
  self.source='benchmark_'+uuid.uuid4().hex;self.seq=0;self.status={};self.frames=[];self.camera_images={};self.camera_infos={};self.camera_checks=[];self.samples=[];self.active=None;self.speed=0.;self.wheel_speed=0.;self.rates={};self.sequences={};self.rtf=[];self.next_send=0.;self.last_rtf=None
  self.create_subscription(String,'/morph/gui/frame_timing',lambda m:self.frames.append(json.loads(m.data)),10)
  self.create_subscription(String,'/morph/status',self.status_cb,1)
  self.create_subscription(String,'/morph/physics/status',self.physics_cb,1)
  self.create_subscription(JointState,'/joint_states',self.joints,qos_profile_sensor_data)
  self.create_subscription(Odometry,'/morph/ground_truth/odom',self.odom,qos_profile_sensor_data)
  self.create_subscription(Image,'/morph/camera/image',self.image,qos_profile_sensor_data)
  self.create_subscription(CameraInfo,'/morph/camera/camera_info',self.info,qos_profile_sensor_data)
  self.create_subscription(Imu,'/morph/imu',lambda m,info:self.rate('imu',m,info),qos_profile_sensor_data)
 def image(self,m,info):
  self.rate('camera',m,info);stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
  self.camera_images[stamp]={'width':m.width,'height':m.height,'encoding':m.encoding,'step':m.step,'data_bytes':len(m.data),'frame':m.header.frame_id}
  self.check_camera(stamp)
 def info(self,m,info):
  self.rate('camera_info',m,info);stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
  self.camera_infos[stamp]={'width':m.width,'height':m.height,'k':list(m.k),'frame':m.header.frame_id}
  self.check_camera(stamp)
 def check_camera(self,stamp):
  if stamp in self.camera_images and stamp in self.camera_infos:
   image=self.camera_images.pop(stamp);info=self.camera_infos.pop(stamp)
   expected=image['width']/(2*math.tan(1.15/2));k=info['k']
   valid=image['width']==info['width'] and image['height']==info['height'] and image['frame']==info['frame'] and abs(k[0]-expected)<1 and abs(k[4]-expected)<1 and abs(k[2]-image['width']/2)<=1 and abs(k[5]-image['height']/2)<=1 and image['data_bytes']==image['step']*image['height']
   self.camera_checks.append({'stamp_ns':stamp,'image':image,'info':info,'expected_focal_px':expected,'passed':bool(valid)})
  for values in (self.camera_images,self.camera_infos):
   while len(values)>100:values.pop(next(iter(values)))
 def sim(self):return self.get_clock().now().nanoseconds
 def rate(self,name,m,info=None):
  self.sequences.setdefault(name,SequenceGaps()).observe(info)
  self.rates.setdefault(name,[]).append([time.monotonic_ns(),m.header.stamp.sec*10**9+m.header.stamp.nanosec])
 def observed(self,name):
  if self.active is not None and name not in self.active:self.active[name]={'wall_ns':time.monotonic_ns(),'sim_ns':self.sim()}
 def status_cb(self,m):
  self.status=json.loads(m.data)
  if not self.status.get('feedback_ready',False):self.feedback_failures+=1
  if self.active:
   timing=self.status.get('timing',{});transition=timing.get('last_transition') or {};src=transition.get('source') or {}
   if src.get('source_id')==self.source and src.get('sequence')==self.active['source']['sequence']:
    self.active.setdefault('manager_transition',transition)
 def physics_cb(self,m):
  self.physics=json.loads(m.data)
  transition=self.physics.get('timing',{}).get('last_motion_transition') or {}
  if self.active and transition.get('receipt_wall_ns',0)>=self.active['source']['source_wall_ns'] and max([abs(v) for v in transition.get('target',[])]+[0.])>.1:
   self.active.setdefault('runtime_target_transition',transition)
 def joints(self,m,info):
  self.rate('joint_states',m,info)
  speeds=[abs(v) for n,v in zip(m.name,m.velocity) if n.endswith('_wheel_joint')];self.wheel_speed=max(speeds or [0.])
  if self.wheel_speed>.4:self.observed('joint_response')
 def odom(self,m,info):
  self.rate('odom',m,info);v=m.twist.twist.linear;self.speed=math.hypot(v.x,v.y);self.xy=[m.pose.pose.position.x,m.pose.pose.position.y]
  if self.speed>.03:self.observed('body_response')
 def send(self,value):
  self.seq+=1;data=dict(values=[value,0.,0.],source_id=self.source,sequence=self.seq,host=socket.gethostname(),source_sim_ns=self.sim(),source_wall_ns=time.monotonic_ns())
  self.pub.publish(String(data=json.dumps(data)));return data
 def step(self,value):
  wall=time.monotonic()
  if wall>=self.next_send:self.send(value);self.next_send=wall+.02
  rclpy.spin_once(self,timeout_sec=.002)
  pair=(time.monotonic_ns(),self.sim())
  if self.last_rtf is None:self.last_rtf=pair
  elif pair[0]-self.last_rtf[0]>=10**9:
   self.rtf.append((pair[1]-self.last_rtf[1])/(pair[0]-self.last_rtf[0]));self.last_rtf=pair
 def hold(self,seconds,value):
  start=self.sim();deadline=time.monotonic()+max(20,seconds*5)
  while (self.sim()-start)*1e-9<seconds:
   if time.monotonic()>deadline:raise RuntimeError('Simulation did not advance the requested hold within its wall-time budget')
   self.step(value)

def main():
 rclpy.init();b=Bench();trials=[];output=Path(os.environ['MORPH_BENCH_OUTPUT']);experiment_start_wall=time.monotonic_ns();experiment_start_sim=b.sim()
 try:
  deadline=time.monotonic()+60
  while not b.status.get('feedback_ready') and time.monotonic()<deadline:b.step(0.)
  if not b.status.get('feedback_ready'):raise RuntimeError('No fresh feedback')
  b.events.publish(String(data='reset'));b.hold(2.,0.);b.rtf=[];b.frames=[];b.camera_checks=[];b.last_rtf=None;b.rates={};b.sequences={};start=(time.monotonic_ns(),b.sim())
  for i in range(int(os.environ.get('MORPH_RESPONSE_TRIALS','20'))):
   b.hold(1.,0.)
   baseline={'body_m_s':b.speed,'wheel_rad_s':b.wheel_speed};value=.15 if i%2==0 else -.15
   source=b.send(value);b.next_send=time.monotonic()+.02;b.active={'trial':i,'source':source,'baseline':baseline,'baseline_settled':b.speed<.01 and b.wheel_speed<.2}
   b.hold(.8,value);trial=b.active;b.active=None
   for name in ('joint_response','body_response'):
    event=trial.get(name);trial[name+'_wall_ms']=(event['wall_ns']-source['source_wall_ns'])/1e6 if event else None;trial[name+'_sim_ms']=(event['sim_ns']-source['source_sim_ns'])/1e6 if event else None
   transition=trial.get('manager_transition') or {}
   for name in ('manager_receipt','first_actuator_publication'):
    for clock in ('wall','sim'):
     stamp=transition.get(name+'_'+clock+'_ns');trial[name+'_'+clock+'_ms']=(stamp-source['source_'+clock+'_ns'])/1e6 if stamp is not None else None
   trials.append(trial)
  b.hold(1.,0.)
  soak_start=b.sim();soak_wall=time.monotonic_ns();soak_duration=float(os.environ.get('MORPH_LOADED_SOAK_SECONDS','0'));soak_wall_duration=float(os.environ.get('MORPH_LOADED_SOAK_WALL_SECONDS','0'))
  soak_samples=[];next_soak_checkpoint=0.;next_soak_sample=0.;soak_feedback_baseline=b.feedback_failures
  if soak_duration>0 or soak_wall_duration>0:
   b.events.publish(String(data='expand'));b.events.publish(String(data='raise'))
  while (b.sim()-soak_start)*1e-9<soak_duration or (time.monotonic_ns()-soak_wall)*1e-9<soak_wall_duration:
   phase=(b.sim()-soak_start)*1e-9%4.
   command=.12 if phase<1.5 else 0. if phase<2 else -.12 if phase<3.5 else 0.
   b.step(command)
   if time.monotonic()>=next_soak_sample:
    next_soak_sample=time.monotonic()+1.
    soak_samples.append({'wall_ns':time.monotonic_ns(),'sim_ns':b.sim(),'speed_m_s':b.speed,'xy':b.xy,'command_x_m_s':command,'feedback_ready':b.status.get('feedback_ready'),'stage_offset_m':b.status.get('stage_offset'),'power':b.physics.get('power'),'suspension_thermal':b.physics.get('suspension_thermal')})
   if time.monotonic()>=next_soak_checkpoint:
    next_soak_checkpoint=time.monotonic()+30.
    output.with_name('soak_progress.json').write_text(json.dumps({'status':'in_progress','wall_s':(time.monotonic_ns()-soak_wall)*1e-9,'sim_s':(b.sim()-soak_start)*1e-9,'samples':soak_samples,'trials_completed':len(trials)},indent=2))
   if time.monotonic_ns()-soak_wall>max(30,soak_duration*5,soak_wall_duration+30)*1e9:raise RuntimeError('Soak clock stalled')
  b.hold(1.,0.);end=(time.monotonic_ns(),b.sim());responses=[t['body_response_wall_ms'] for t in trials if t['body_response_wall_ms'] is not None]
  mean=(end[1]-start[1])/(end[0]-start[0]);p05=float(np.percentile(b.rtf,5)) if b.rtf else None;p95=float(np.percentile(responses,95)) if len(responses)==len(trials) else None
  soak_motion_fraction=sum(x['speed_m_s']>.03 for x in soak_samples)/len(soak_samples) if soak_samples else None
  soak_passed=(soak_motion_fraction is not None and soak_motion_fraction>=.5 and b.feedback_failures==soak_feedback_baseline and all(not (x.get('power') or {}).get('brownout',True) and not (x.get('power') or {}).get('needs_home',True) for x in soak_samples)) if soak_duration or soak_wall_duration else None
  rates={}
  for name in ('joint_states','odom','imu','camera','camera_info'):
   values=b.rates.get(name,[])
   if not values:
    rates[name]={'messages':0,'observed_wall_hz':0.,'max_stamp_gap_s':None,'non_increasing_stamps':None,'dropped_samples':None,'availability':'No messages observed; do not claim camera performance for this run'};continue
   a=np.array(values,dtype=np.int64);ds=np.diff(a[:,1])*1e-9
   rates[name]={'messages':len(a),'observed_sim_hz':(len(a)-1)*1e9/(a[-1,1]-a[0,1]) if len(a)>1 and a[-1,1]>a[0,1] else None,'ros_writer_sequence_gaps':b.sequences[name].report(),'observed_wall_hz':(len(a)-1)*1e9/(a[-1,0]-a[0,0]) if len(a)>1 else None,'max_stamp_gap_s':float(max(ds)) if len(ds) else None,'non_increasing_stamps':int(sum(ds<=0)),'dropped_samples':None,'drop_scope':'No producer sequence counter; actual drops cannot be identified from header gaps alone'}
  report={'scope':'20 application command-to-observed-motion trials by default; not physical keyboard or compositor timing','mean_rtf':mean,'one_second_rtf_p05':p05,'body_response_wall_p95_ms':p95,'criteria':{'mean_rtf_min':.95,'one_second_rtf_p05_min':.90,'response_p95_ms_max':250},'trials':trials,'one_second_rtf':b.rtf,'sensor_rates':rates,'sensor_samples':b.rates,'camera_validation':{'matched_pairs':len(b.camera_checks),'passed':bool(b.camera_checks) and all(x['passed'] for x in b.camera_checks),'checks':b.camera_checks},'callback_execution':b.status.get('timing',{}).get('callback_execution'),'graphics_frame_time':b.frames or None,'loaded_soak_wall_s':(time.monotonic_ns()-soak_wall)*1e-9 if soak_duration or soak_wall_duration else 0.,'loaded_soak_requested_wall_s':soak_wall_duration,'loaded_soak_sim_s':(b.sim()-soak_start)*1e-9 if soak_duration or soak_wall_duration else 0.,'loaded_soak_requested_sim_s':soak_duration,'loaded_soak_samples':soak_samples,'loaded_soak_motion_fraction':soak_motion_fraction,'loaded_soak_feedback_failure_messages':b.feedback_failures-soak_feedback_baseline,'loaded_soak_passed':soak_passed,'loaded_soak_criteria':{'motion_fraction_min':.5,'feedback_failure_messages_max':0,'brownout_or_needs_home_allowed':False},'loaded_soak_scope':'Alternating forward/reverse electrical-drive motion at assumed robot mass, expanded and raised; no extra calibrated payload','passed':len(trials)>=20 and all(t['baseline_settled'] for t in trials) and p95 is not None and p95<=250 and mean>=.95 and p05 is not None and p05>=.90 and (soak_passed is not False)}
  output.write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k not in ('trials','sensor_samples')},indent=2));return 0 if report['passed'] else 1
 except Exception as error:
  output.write_text(json.dumps({'passed':False,'error':str(error),'trials_completed':len(trials),'trials':trials,'observed_wall_s':(time.monotonic_ns()-experiment_start_wall)*1e-9,'observed_sim_s':(b.sim()-experiment_start_sim)*1e-9,'sensor_samples':b.rates,'graphics_frame_time':b.frames,'camera_validation':{'matched_pairs':len(b.camera_checks),'checks':b.camera_checks},'scope':'Incomplete protocol; no release RTF or p95 claimed'},indent=2))
  raise
 finally:
  if rclpy.ok():b.send(0.)
  b.destroy_node()
  if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':raise SystemExit(main())
