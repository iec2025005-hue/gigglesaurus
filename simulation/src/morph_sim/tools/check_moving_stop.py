#!/usr/bin/env python3
"""Moving stop/failure experiment; run under run_benchmark_case.py ownership."""
import json,math,os,signal,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String,Float64MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from controller_manager_msgs.srv import SwitchController
from morph_sim.timing import TimedVelocityPublisher


class Experiment(Node):
    def __init__(self):
        super().__init__('moving_stop_check',parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.status={};self.physics={};self.watchdog={};self.trajectory=[];self.positions={};self.position_commands=[]
        self.pub=TimedVelocityPublisher(self,'stop_protocol');self.events=self.create_publisher(String,'/morph/event',10)
        self.create_subscription(String,'/morph/status',lambda m:self.status.update(json.loads(m.data)),1)
        self.create_subscription(String,'/morph/physics/status',lambda m:self.physics.update(json.loads(m.data)),1)
        self.create_subscription(String,'/morph/watchdog/status',lambda m:self.watchdog.update(json.loads(m.data)),1)
        self.create_subscription(Odometry,'/morph/ground_truth/odom',self.odom,qos_profile_sensor_data)
        self.create_subscription(JointState,'/joint_states',lambda m:self.positions.update(zip(m.name,m.position)),qos_profile_sensor_data)
        for kind in ('steering','height','shape'):
            self.create_subscription(Float64MultiArray,'/'+kind+'_controller/commands',lambda m,k=kind:self.position_commands.append((self.now(),k,list(m.data))),1)
        self.next_pub=0.
    def now(self):return self.get_clock().now().nanoseconds*1e-9
    def odom(self,m):
        v=m.twist.twist.linear;p=m.pose.pose.position
        self.trajectory.append({'sim_s':m.header.stamp.sec+m.header.stamp.nanosec*1e-9,'wall_ns':time.monotonic_ns(),'xy':[p.x,p.y],'speed':math.hypot(v.x,v.y)})
    def spin(self,command=None):
        if command is not None and time.monotonic()>=self.next_pub:
            msg=Twist();msg.linear.x=command;self.pub.publish(msg);self.next_pub=time.monotonic()+.02
        rclpy.spin_once(self,timeout_sec=.003)
    def hold(self,duration,command=0.):
        start=self.now();deadline=time.monotonic()+max(10,duration*4)
        while self.now()-start<duration:
            if time.monotonic()>deadline:raise RuntimeError('Simulation clock stalled')
            self.spin(command)


def find_owned_node(name):
    found=[]
    for path in Path('/proc').iterdir():
        if not path.name.isdigit():continue
        try:
            args=(path/'cmdline').read_bytes().split(b'\0')
            if any(Path(arg.decode()).name==name for arg in args if arg) and (path/'cwd').resolve()==Path.cwd():found.append(int(path.name))
        except (OSError,ValueError,UnicodeDecodeError):pass
    if len(found)!=1:raise RuntimeError('Expected one owned '+name+' process; found '+str(found))
    return found[0]


def main():
    case=os.environ.get('MORPH_STOP_CASE','estop')
    if case not in ('estop','release','stale_command','stale_state','manager_crash','actuator_crash'):raise ValueError('Unknown stop case')
    speed=float(os.environ.get('MORPH_STOP_SPEED','0.25'));mode=os.environ.get('MORPH_STOP_MODE','')
    if not .05<=speed<=.25:raise ValueError('Stop speed must be in [0.05,0.25] m/s')
    rclpy.init();node=Experiment();output=Path(os.environ.get('MORPH_BENCH_OUTPUT','validation/moving_stop.json'))
    try:
        deadline=time.monotonic()+60
        while (not node.status.get('feedback_ready') or not node.trajectory) and time.monotonic()<deadline:node.spin()
        if not node.status.get('feedback_ready'):raise RuntimeError('No complete measured joint feedback')
        node.hold(2.,0.);node.events.publish(String(data='reset'))
        if mode:node.events.publish(String(data=mode));node.hold(8.,0.)
        node.hold(2.,speed)
        before=node.trajectory[-1]
        if before['speed']<.6*speed:raise RuntimeError('Robot did not reach the declared moving test speed')
        trigger={'wall_ns':time.monotonic_ns(),'sim_s':node.now(),'xy':before['xy'],'speed_m_s':before['speed']}
        future=None
        if case=='estop':node.events.publish(String(data='estop'))
        elif case=='stale_state':
            client=node.create_client(SwitchController,'/controller_manager/switch_controller')
            if not client.wait_for_service(timeout_sec=5):raise RuntimeError('No controller switch service')
            request=SwitchController.Request();request.strictness=2;request.deactivate_controllers=['joint_state_broadcaster'];future=client.call_async(request)
        elif case in ('manager_crash','actuator_crash'):os.kill(find_owned_node('terrain_manager' if case=='manager_crash' else 'interaction_runtime'),signal.SIGKILL)
        node.pub.mark_input(case)
        command=0. if case=='release' else None if case=='stale_command' else speed
        deadline=time.monotonic()+5;streak=None;previous=before;path=0.;first_low=None;last_index=len(node.trajectory)
        while time.monotonic()<deadline:
            node.spin(command)
            for sample in node.trajectory[last_index:]:
                path+=math.dist(sample['xy'],previous['xy']);previous=sample
                if sample['speed']<.01:
                    if streak is None:streak=sample['sim_s'];first_low=(sample.copy(),path)
                    if sample['sim_s']-streak>=.2:break
                else:streak=None;first_low=None
            last_index=len(node.trajectory)
            if streak is not None and previous['sim_s']-streak>=.2:break
        stable=first_low is not None and previous['sim_s']-streak>=.2 if streak is not None else False
        stopped=first_low[0] if stable else previous;distance=first_low[1] if stable else path
        node.hold(.5,command)
        post=node.trajectory[-1]
        no_position_commands=(not any(t>trigger['sim_s']+.35 for t,_,_ in node.position_commands)) if case=='stale_state' else None
        report={'case':case,'requested_speed_m_s':speed,'mode':mode or 'unchanged','initial_mode':node.status.get('mode'),'parameter_status':'assumed','scope':'Simulation failure test; stop time is source action to first body sample below 0.01 m/s sustained for 0.2 s','criteria':{'stop_wall_s_max':1.,'stop_distance_m_max':.2,'post_hold_speed_m_s_max':.02},'trigger':trigger,'stop_wall_s':(stopped['wall_ns']-trigger['wall_ns'])*1e-9,'stop_sim_s':stopped['sim_s']-trigger['sim_s'],'stop_distance_path_m':distance,'stable_stop':stable,'post_hold_speed_m_s':post['speed'],'native_watchdog':node.watchdog,'no_position_commands_after_stale_deadline':no_position_commands,'switch_ack':bool(future.done() and future.result() and future.result().ok) if future else None,'manager_status':node.status,'physics_status':node.physics,'trajectory':node.trajectory,'position_command_count':len(node.position_commands)}
        report['passed']=stable and report['stop_wall_s']<=1 and distance<=.2 and post['speed']<.02 and (case!='estop' or node.status.get('estop',False)) and (case!='stale_state' or no_position_commands)
        if case in ('stale_state','manager_crash','actuator_crash'):report['passed']=report['passed'] and not node.watchdog.get('heartbeat_fresh',True)
        output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k not in ('trajectory','manager_status','physics_status')},indent=2))
        return 0 if report['passed'] else 1
    finally:
        if rclpy.ok():node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':raise SystemExit(main())
