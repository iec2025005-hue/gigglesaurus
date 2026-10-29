#!/usr/bin/env python3
"""Exercise the actual Tk binding/publisher path with synthetic Tk key/focus events.

This measures application event dispatch to observed motion, excluding physical
keyboard, OS delivery and compositor display latency. It owns its test windows.
"""
import json,math,os,time,tkinter as tk
from pathlib import Path
import numpy as np
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from morph_sim.control_console import main
report={'scope':'Synthetic Tk KeyPress/KeyRelease and owned-window FocusOut through real console bindings; excludes physical keyboard, OS input delivery and compositor','trials':[],'display_updates':[],'passed':False}

def hook(root,node,close):
 state={};rtf_pairs=[];clock_pairs=[]
 data={'phase':'wait','phase_wall':time.monotonic(),'speed':0.,'active':None,'trial':0,'stop_kind':None};other=tk.Toplevel(root);other.title('MORPH focus-loss test target');other.withdraw()
 def status(m):state.update(json.loads(m.data))
 def odom(m):
  v=m.twist.twist.linear;data['speed']=math.hypot(v.x,v.y)
  active=data['active']
  if active is not None and data['speed']>.03 and 'body_response_wall_ns' not in active:
   active['body_response_wall_ns']=time.monotonic_ns();active['body_response_sim_ns']=node.get_clock().now().nanoseconds
 def command(m):
  d=json.loads(m.data)
  if not d['source_id'].startswith('console_'):return
  active=data['active']
  if active is not None and abs(d['values'][0])>.01 and 'first_command' not in active:active['first_command']=d
  if data['stop_kind'] and max(abs(v) for v in d['values'])<1e-6:
   report.setdefault(data['stop_kind']+'_first_zero',dict(wall_ns=time.monotonic_ns(),sim_ns=node.get_clock().now().nanoseconds,command=d))
 node.create_subscription(String,'/morph/status',status,1)
 node.create_subscription(String,'/morph/cmd_vel_timed',command,1)
 node.create_subscription(Odometry,'/morph/ground_truth/odom',odom,qos_profile_sensor_data)
 node.create_subscription(String,'/morph/console/display_timing',lambda m:report['display_updates'].append(json.loads(m.data)),1)
 def end(error=None):
  if error:report['error']=error
  responses=[t.get('response_wall_ms') for t in report['trials']]
  report['response_wall_p95_ms']=float(np.percentile(responses,95)) if len(responses)==20 and all(v is not None for v in responses) else None
  stamps=[v['wall_ns'] for v in report['display_updates']]
  report['widget_update_hz']=(len(stamps)-1)*1e9/(stamps[-1]-stamps[0]) if len(stamps)>1 else None
  report['graphics_frame_time']=None
  if len(clock_pairs)>1:
   report['mean_rtf']=(clock_pairs[-1][1]-clock_pairs[0][1])/(clock_pairs[-1][0]-clock_pairs[0][0]);rtfs=[(b[1]-a[1])/(b[0]-a[0]) for a,b in zip(clock_pairs,clock_pairs[1:])];report['one_second_rtf_p05']=float(np.percentile(rtfs,5));report['rtf_wall_sim_ns']=clock_pairs
  else:report['mean_rtf']=None;report['one_second_rtf_p05']=None
  report['timing_targets_passed']=report['mean_rtf'] is not None and report['mean_rtf']>=.95 and report['one_second_rtf_p05']>=.90
  report['passed']=not error and report['response_wall_p95_ms'] is not None and report['response_wall_p95_ms']<=250 and all(report.get(k+'_passed',False) for k in ('release','focus_loss'))
  Path(os.environ['MORPH_BENCH_OUTPUT']).write_text(json.dumps(report,indent=2));other.destroy();close()
 def enter(phase):data.update(phase=phase,phase_wall=time.monotonic())
 def tick():
  pair=(time.monotonic_ns(),node.get_clock().now().nanoseconds)
  if state.get('feedback_ready'):
   if not clock_pairs or pair[0]-clock_pairs[-1][0]>=10**9:clock_pairs.append(pair)
  wall=time.monotonic();age=wall-data['phase_wall'];phase=data['phase']
  if phase=='wait':
   if state.get('feedback_ready') and age>2:enter('idle')
   elif age>60:return end('No fresh simulation feedback')
  elif phase=='idle' and age>1.2:
   if data['speed']>.01:return end('Robot did not settle before key trial')
   root.focus_force();key='w' if data['trial']%2==0 else 's';data['key']=key
   data['active']=dict(trial=data['trial'],event_wall_ns=time.monotonic_ns(),event_sim_ns=node.get_clock().now().nanoseconds)
   root.event_generate('<KeyPress>',keysym=key);enter('moving')
  elif phase=='moving' and age>.8:
   active=data['active'];event=active.get('body_response_wall_ns');active['response_wall_ms']=(event-active['event_wall_ns'])/1e6 if event else None
   report['trials'].append(active);data['active']=None;data['trial']+=1
   root.event_generate('<KeyRelease>',keysym=data['key']);enter('idle' if data['trial']<20 else 'prepare_release')
  elif phase.startswith('prepare_') and age>1.2:
   if data['speed']>.01:return end('Robot did not settle before stopping check')
   root.focus_force();root.event_generate('<KeyPress>',keysym='w');enter('drive_'+phase[8:])
  elif phase.startswith('drive_') and age>1.5:
   kind=phase[6:]
   if data['speed']<.15:return end('Stop check did not start while moving')
   data['stop_kind']=kind;report[kind+'_trigger']=dict(wall_ns=time.monotonic_ns(),sim_ns=node.get_clock().now().nanoseconds)
   if kind=='release':root.event_generate('<KeyRelease>',keysym='w')
   else:other.deiconify();other.focus_force()
   enter('stop_'+kind)
  elif phase.startswith('stop_'):
   kind=phase[5:]
   if data['speed']<.01 and age>.1:
    report[kind+'_stop_wall_s']=age;report[kind+'_passed']=age<=1 and kind+'_first_zero' in report;data['stop_kind']=None
    if kind=='release':enter('prepare_focus_loss')
    else:return end()
   elif age>3:return end('Stopping failed: '+kind)
  root.after(5,tick)
 root.after(20,tick)

if __name__=='__main__':
 import sys
 sys.argv=[sys.argv[0],'--ros-args','-p','use_sim_time:=true']
 main(test_hook=hook)
 if not report['passed']:raise SystemExit(1)
