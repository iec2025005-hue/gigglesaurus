#!/usr/bin/env python3
"""Trace combined morphology/steering and manual reversals without changing limits."""
import json,os,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from morph_sim.timing import TimedVelocityPublisher
rclpy.init();node=Node('trajectory_limit_check',parameter_overrides=[Parameter('use_sim_time',value=True)])
state={};samples=[];native=[]
def receive(message):
    state.update(json.loads(message.data));samples.append(dict(sim_s=node.get_clock().now().nanoseconds*1e-9,wall_ns=time.monotonic_ns(),status=dict(state)))
node.create_subscription(String,'/morph/status',receive,1)
node.create_subscription(String,'/morph/watchdog/status',lambda m:native.append(json.loads(m.data)),1)
pub=TimedVelocityPublisher(node,'trajectory_protocol');event=node.create_publisher(String,'/morph/event',10);manual=node.create_publisher(String,'/morph/manual_joint',10)
def sim():return node.get_clock().now().nanoseconds*1e-9
def hold(duration,values=(0,0,0)):
    start=sim();deadline=time.monotonic()+max(10,4*duration);next_pub=0
    while sim()-start<duration:
        if time.monotonic()>deadline:raise RuntimeError('Clock stalled')
        if time.monotonic()>=next_pub:
            message=Twist();message.linear.x=float(values[0]);message.linear.y=float(values[1]);message.angular.z=float(values[2]);pub.publish(message);next_pub=time.monotonic()+.02
        rclpy.spin_once(node,timeout_sec=.003)
try:
    deadline=time.monotonic()+60
    while not state.get('feedback_ready') and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.01)
    if not state.get('feedback_ready'):raise RuntimeError('Missing complete feedback')
    begin_sim=sim();begin_wall=time.monotonic();hold(2)
    event.publish(String(data='expand'));event.publish(String(data='raise'));hold(5,(.12,.04,.1))
    event.publish(String(data='compact'));event.publish(String(data='lower'));hold(4,(.08,-.04,-.1));hold(1)
    for target in (-1.2,1.2):
        for name in ('fl','fr','rl','rr'):manual.publish(String(data=json.dumps(dict(joint=name+'_steer_joint',value=target,duration=2))))
        hold(2)
    hold(2);event.publish(String(data='estop'));hold(1)
    rows=[row for sample in samples for row in sample['status'].get('position_command_trace',[])]
    enabled=bool(state.get('position_command_limits',{}).get('filter_enabled'))
    report=dict(scope='Manager outgoing requests/published positions; native limiter messages retained separately in launch log',filter_enabled=enabled,sim_s=sim()-begin_sim,wall_s=time.monotonic()-begin_wall,trace_samples=len(rows),max_published_velocity_ratio=max([abs(row['command_velocity'])/row['velocity_limit'] for row in rows]+[0]),lead_conflicts=sum(row['lead_conflict'] for row in rows),samples=samples,native_samples=native)
    report['mean_rtf']=report['sim_s']/report['wall_s'];report['passed']=bool(rows) and (not enabled or report['max_published_velocity_ratio']<=.951)
    output=Path(os.environ['MORPH_BENCH_OUTPUT']);output.write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k not in ('samples','native_samples')},indent=2))
finally:
    if rclpy.ok():pub.publish(Twist())
    node.destroy_node()
    if rclpy.ok():rclpy.shutdown()
