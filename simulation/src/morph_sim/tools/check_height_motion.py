#!/usr/bin/env python3
"""Measure all ride-height axes under robot load, using existing force/travel limits."""
import json,os,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from morph_sim.timing import TimedVelocityPublisher
rclpy.init();node=Node('height_under_load_check',parameter_overrides=[Parameter('use_sim_time',value=True)])
state={};joints={};servos=[];trajectory=[]
node.create_subscription(String,'/morph/status',lambda m:state.update(json.loads(m.data)),1)
def sample(message):
    joints.update(zip(message.name,message.position));trajectory.append({'sim_s':message.header.stamp.sec+message.header.stamp.nanosec*1e-9,'height_m':{n:joints.get(n+'_height_joint') for n in ('fl','fr','rl','rr')}})
node.create_subscription(JointState,'/joint_states',sample,qos_profile_sensor_data)
node.create_subscription(String,'/morph/height/status',lambda m:servos.append(json.loads(m.data)),1)
pub=TimedVelocityPublisher(node,'height_protocol');manual=node.create_publisher(String,'/morph/manual_joint',10)
def sim():return node.get_clock().now().nanoseconds*1e-9
def hold(duration):
    start=sim();deadline=time.monotonic()+max(15,duration*4);next_pub=0
    while sim()-start<duration:
        if time.monotonic()>deadline:raise RuntimeError('Clock stalled')
        if time.monotonic()>=next_pub:pub.publish(Twist());next_pub=time.monotonic()+.02
        rclpy.spin_once(node,timeout_sec=.003)
try:
    deadline=time.monotonic()+60
    while not state.get('feedback_ready') and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.01)
    if not state.get('feedback_ready'):raise RuntimeError('No complete feedback')
    hold(2);results=[]
    for target in (.010,-.010):
        for name in ('fl','fr','rl','rr'):manual.publish(String(data=json.dumps({'joint':name+'_height_joint','value':target,'duration':6.})))
        hold(5)
        results.append({'target_m':target,'measured_m':{name:joints[name+'_height_joint'] for name in ('fl','fr','rl','rr')}})
    errors=[abs(value-row['target_m']) for row in results for value in row['measured_m'].values()]
    report={'scope':'Simulation-only ride-height tracking under the assumed 10 kg robot load','results':results,'max_error_m':max(errors),'max_force_N':max([abs(axis['force_N']) for sample in servos for axis in sample['axes']]+[0]),'max_speed_m_s':max([abs(axis['velocity_m_s']) for sample in servos for axis in sample['axes']]+[0]),'criteria':{'position_error_m_max':.001,'force_N_max':100.},'trajectory':trajectory,'native_samples':servos}
    report['passed']=bool(servos) and report['max_error_m']<=.001 and report['max_force_N']<=100
    Path(os.environ['MORPH_BENCH_OUTPUT']).write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k not in ('trajectory','native_samples')},indent=2))
    raise SystemExit(0 if report['passed'] else 1)
finally:
    if rclpy.ok():pub.publish(Twist())
    node.destroy_node()
    if rclpy.ok():rclpy.shutdown()
