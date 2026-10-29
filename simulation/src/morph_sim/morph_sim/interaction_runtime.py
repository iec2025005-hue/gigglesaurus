"""Assumed hardware plant + diagnostic sensors; raw Gazebo feedback stays intact."""
import copy
import json
import math
from pathlib import Path
import random
import time
import numpy as np
import rclpy
from . import lifecycle
from .safety import StopPositionHold,valid_joint_sample,fresh_snapshot,ACTUATED_JOINTS
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, Float64, String, Bool
from ros_gz_interfaces.msg import Contacts
from .control import LEGS
from .course import Course
from ament_index_python.packages import get_package_share_directory
from .design_physics import load_parameters, PowerPlant, design_report, contact_patch, traction_prediction, soft_ground_prediction, clamp


class InteractionRuntime(Node):
    def __init__(self):
        super().__init__('morph_interactions')
        self.declare_parameter('resolved_manifest_file','');self.declare_parameter('params_file',''); self.declare_parameter('scenario','indoor')
        self.declare_parameter('log_dir','log/interactions');self.declare_parameter('actuate',True);self.declare_parameter('arena','adaptive')
        manifest_file=Path(self.get_parameter('resolved_manifest_file').value)
        if not manifest_file.is_file():raise ValueError('A resolved run manifest is required for material-consistent diagnostics')
        self.manifest=json.loads(manifest_file.read_text())
        self.materials={record['collision']:record['surface'] for record in self.manifest.get('native_surface_assignments',[])}
        self.contact_materials={}
        self.p=load_parameters(self.get_parameter('params_file').value)
        self.declare_parameter('acceleration_feedforward',True)
        self.scenario=self.get_parameter('scenario').value
        self.env=dict(self.p['environment']['scenarios'][self.scenario])
        self.zone='warehouse' if self.get_parameter('arena').value=='warehouse' else 'marsyard'
        self.surface='concrete' if self.zone=='warehouse' else self.p['environment']['default_surface']
        self.course=Course(get_package_share_directory('morph_sim')) if self.get_parameter('arena').value=='multisector' else None
        self.plant=PowerPlant(self.p,self.env['ambient_C']); self.rng=random.Random(self.p['provenance']['seed'])
        self.actuate=self.get_parameter('actuate').value
        safety=self.p['control_safety'];self.stop_hold=StopPositionHold(safety['hold_capture_speed_rad_s'],safety['hold_position_gain_s_inv'],safety['hold_speed_limit_rad_s']);self.was_braking=False
        self.steer_integral=[0.]*4;self.drive=[0.]*4;self.steer=[0.]*4;self.states={};self.vel={};self.contacts={n:-1e9 for n in LEGS}
        self.motion_transition=None;self.target_transition=None;self.target_time=-1e9;self.target_wall=-1e9;self.state_time=-1e9;self.state_wall=-1e9;self.odom=None;self.status={};self.last=None;self.samples=0
        self.enc_pose=np.zeros(3);self.encoder_previous={};self.counts={};self.missed=0;self.filtered={}
        self.imu_residual=0.;self.last_power={};self.suspension_telemetry={};self.k=self.p['suspension']['stiffness_N_m'];self.c=self.p['suspension']['damping_Ns_m']
        self.logtime=-1e9
        directory=Path(self.get_parameter('log_dir').value)/time.strftime('%Y%m%d_%H%M%S')
        directory.mkdir(parents=True,exist_ok=True)
        (directory/'params.yaml').write_text(Path(self.get_parameter('params_file').value).read_text())
        (directory/'resolved_manifest.json').write_text(json.dumps(self.manifest,indent=2))
        (directory/'design_report.json').write_text(json.dumps(design_report(self.p),indent=2))
        self.log=(directory/'telemetry.jsonl').open('w',buffering=1)
        self.height_targets=[self.create_publisher(Float64,'/morph/height/'+name+'/target',1) for name in LEGS]
        self.create_subscription(Float64MultiArray,'/height_controller/commands',self.height_target,1)
        self.watchdog_pub=self.create_publisher(Bool,'/morph/actuator/watchdog',1)
        self.drive_pub=self.create_publisher(Float64MultiArray,'/drive_controller/commands',10)
        self.steer_pub=self.create_publisher(Float64MultiArray,'/steering_controller/commands',10)
        self.ambient_pub=self.create_publisher(Float64,'/morph/suspension/ambient',1)
        self.create_subscription(String,'/morph/suspension/telemetry',self.suspension_state,1)
        self.telemetry=self.create_publisher(String,'/morph/physics/status',10)
        self.sensor_joints=self.create_publisher(JointState,'/morph/sensors/joint_states',10)
        self.sensor_imu=self.create_publisher(Imu,'/morph/sensors/imu',qos_profile_sensor_data)
        self.encoder_odom=self.create_publisher(Odometry,'/morph/encoder/odom',10)
        self.create_subscription(Float64MultiArray,'/morph/actuator/drive_target',self.target,1)
        self.create_subscription(Float64MultiArray,'/morph/actuator/steer_target',self.steering_target,1)
        self.create_subscription(JointState,'/joint_states',self.joints,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/morph/ground_truth/odom',self.odometry,qos_profile_sensor_data)
        self.create_subscription(Imu,'/morph/imu',self.imu,qos_profile_sensor_data)
        self.create_subscription(String,'/morph/status',self.manager_status,10)
        self.create_subscription(String,'/morph/physics/event',self.event,10)
        for n in LEGS:self.create_subscription(Contacts,'/morph/contacts/'+n,lambda msg,n=n:self.contact(n,msg),qos_profile_sensor_data)
        self.create_timer(.01,self.tick)
        self.get_logger().info('ASSUMED design profile; raw data and degraded sensors are separate. Logs: '+str(directory))

    def suspension_state(self,m):
        try:self.suspension_telemetry=json.loads(m.data)
        except ValueError:pass
    def now(self):return self.get_clock().now().nanoseconds*1e-9
    def target(self,m):
        if len(m.data)==4 and all(math.isfinite(v) for v in m.data):
            moving=max(abs(v) for v in m.data)>1e-6
            if moving and max(abs(v) for v in self.drive)<=1e-6:self.motion_transition={'target':list(m.data),'receipt_wall_ns':time.monotonic_ns(),'receipt_sim_ns':self.get_clock().now().nanoseconds}
            if list(m.data)!=self.drive:self.target_transition={'target':list(m.data),'receipt_wall_ns':time.monotonic_ns(),'receipt_sim_ns':self.get_clock().now().nanoseconds}
            self.drive=list(m.data);self.target_time=self.now();self.target_wall=time.monotonic()
    def height_target(self,m):
        if self.actuate and len(m.data)==4 and all(math.isfinite(v) and -.015<=v<=.015 for v in m.data) and fresh_snapshot(self.state_time,self.state_wall,self.now(),time.monotonic(),self.p['control_safety']['state_timeout_s'],self.p['control_safety']['future_stamp_tolerance_s']):
            for pub,value in zip(self.height_targets,m.data):pub.publish(Float64(data=float(value)))
    def steering_target(self,m):
        if len(m.data)==4 and all(math.isfinite(v) for v in m.data):self.steer=list(m.data)
    def joints(self,m):
        stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        if not valid_joint_sample(m.name,m.position,m.velocity,stamp,self.now(),self.p['control_safety']['state_timeout_s'],self.p['control_safety']['future_stamp_tolerance_s']):
            self.states={};self.vel={};return
        self.state_wall=time.monotonic()
        self.states={n:q for n,q in zip(m.name,m.position) if math.isfinite(q)}
        self.vel={n:q for n,q in zip(m.name,m.velocity) if math.isfinite(q)}
        self.state_time=stamp
    def contact(self,n,m):
        if m.contacts:
            self.contacts[n]=self.now()
            candidates=[]
            for contact in m.contacts:
                for entity in (contact.collision1,contact.collision2):
                    name=entity.name.split('::')[-1]
                    if name in self.materials:candidates.append(self.materials[name])
            if candidates:self.contact_materials[n]=sorted(set(candidates))
    def odometry(self,m):
        if self.odom is None:
            q=m.pose.pose.orientation
            self.enc_pose[:]=[m.pose.pose.position.x,m.pose.pose.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
        self.odom=m
        if self.course:
            sector=self.course.at(m.pose.pose.position.x,m.pose.pose.position.y)
            self.zone=sector.name if sector else 'access_road'
            self.surface=sector.surface if sector else 'tarmac'
    def manager_status(self,m):
        try:self.status=json.loads(m.data);self.k=float(self.status.get('stiffness',self.k));self.c=float(self.status.get('damping',self.c))
        except (ValueError,TypeError):pass
    def event(self,m):
        event=m.data
        if event=='home':
            stopped=max([abs(v) for n,v in self.vel.items() if n.endswith('_wheel_joint')]+[0.])<.1
            self.get_logger().info('Simulated homing accepted' if self.plant.home(stopped) else 'Homing rejected: stop wheels and recover rails')
        elif event in self.p['environment']['scenarios']:
            if self.p['environment']['scenarios'][event]['wetness']!=self.env['wetness']:
                self.get_logger().warning('Wetness changes require relaunch to update native collision materials and diagnostics together');return
            self.scenario=event;self.env=dict(self.p['environment']['scenarios'][event]);self.plant.ambient=self.env['ambient_C']
        elif event.startswith('surface:') and event[8:] in self.p['surfaces']:
            self.get_logger().warning('Surface overrides require schema regeneration and relaunch; native material assignments remain authoritative')
        elif event=='brownout':
            self.plant.brownout=True;self.plant.needs_home=True;self.plant.resets+=1
    def imu(self,m):
        out=copy.deepcopy(m);s=self.p['sensors'];temperature=self.env['ambient_C'];ss=0.
        for key,std,limit in [('angular_velocity',s['imu_gyro_std_rad_s'],s['imu_gyro_limit_rad_s']),('linear_acceleration',s['imu_accel_std_m_s2'],s['imu_accel_limit_m_s2'])]:
            for axis in ('x','y','z'):
                noise=self.rng.gauss(0,std)+(temperature-22)*s['imu_gyro_temp_rad_s_K'] if key=='angular_velocity' else self.rng.gauss(0,std)
                value=getattr(getattr(m,key),axis)
                setattr(getattr(out,key),axis,clamp(value+noise,-limit,limit));ss+=noise*noise
        self.imu_residual=math.sqrt(ss/6)
        # Orientation remains the raw simulator estimate, explicitly not integrated noisy gyro.
        self.sensor_imu.publish(out)
    def tick(self):
        now=self.now();dt=0. if self.last is None else now-self.last;self.last=now
        if dt<=0:return
        dt=min(dt,.05)
        ready=fresh_snapshot(self.state_time,self.state_wall,now,time.monotonic(),self.p['control_safety']['state_timeout_s'],self.p['control_safety']['future_stamp_tolerance_s']) and all(n in self.states and n in self.vel for n in ACTUATED_JOINTS)
        target=self.drive if fresh_snapshot(self.target_time,self.target_wall,now,time.monotonic(),self.p['control_safety']['command_timeout_s'],self.p['control_safety']['future_stamp_tolerance_s']) else [0.]*4
        if self.actuate:self.watchdog_pub.publish(Bool(data=ready and fresh_snapshot(self.target_time,self.target_wall,now,time.monotonic(),self.p['control_safety']['command_timeout_s'],self.p['control_safety']['future_stamp_tolerance_s'])))
        omega=[self.vel.get(n+'_wheel_joint',0.) for n in LEGS]
        self.p['drive']['acceleration_feedforward']=bool(self.get_parameter('acceleration_feedforward').value)
        braking=self.status.get('estop',False)
        if braking:
            target=[0.]*4
            if not self.was_braking:self.plant.integral=[0.]*4
        self.was_braking=braking
        previous_hold=list(self.stop_hold.positions)
        target=self.stop_hold.apply(target,[self.states.get(n+'_wheel_joint',0.) for n in LEGS],omega,ready)
        for i,(old,new) in enumerate(zip(previous_hold,self.stop_hold.positions)):
            if old is None and new is not None:
                # Do not carry braking PI integral into captured-position holding.
                self.plant.integral[i]=0.;self.plant.previous_targets[i]=target[i]
        power=self.plant.step(dt,target,omega,enabled=ready)
        power['controlled_stop_braking']=bool(braking)
        self.last_power=power
        r=self.p['robot'];st=self.p['steering'];surface=self.p['surfaces'][self.surface]
        torques=[];wheels={};matrix=[];observations=[]
        sensor=JointState();sensor.header.stamp=self.get_clock().now().to_msg()
        vx=vy=wz=roll=pitch=0.
        if self.odom:
            v=self.odom.twist.twist;vx,vy,wz=v.linear.x,v.linear.y,v.angular.z
            q=self.odom.pose.pose.orientation
            roll=math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y));pitch=math.asin(clamp(2*(q.w*q.y-q.z*q.x),-1,1))
        for i,n in enumerate(LEGS):
            materials=self.contact_materials.get(n,[])
            material=min(materials,key=lambda name:self.p['surfaces'][name]['mu_kinetic_x']) if materials else self.surface
            surface=self.p['surfaces'][material]
            contact=now-self.contacts[n]<.15
            spring=self.states.get(n+'_spring_joint',0.);spring_v=self.vel.get(n+'_spring_joint',0.)
            normal=max(0.,self.k*spring+self.c*spring_v+r['unsprung_mass_per_module_kg']*9.81) if contact else 0.
            angle=self.states.get(n+'_steer_joint',0.);speed=self.vel.get(n+'_steer_joint',0.)
            error=clamp(self.steer[i],-math.pi/2,math.pi/2)-angle
            scrub=2/3*surface['mu_static_x']*normal*st['effective_patch_radius_m']
            available=min(st['output_torque_limit_Nm'],st['motor_holding_torque_Nm']*st['gear_ratio']*st['gear_efficiency']/(1+abs(speed)/st['pullout_speed_rad_s']))
            self.steer_integral[i]=clamp(self.steer_integral[i]+.4*error*dt,-available*.8,available*.8) if ready and not power['needs_home'] else 0.
            demand=st['kp_Nm_rad']*error+self.steer_integral[i]-st['kd_Nms_rad']*speed
            if abs(speed)>.8 and demand*speed>0:demand=0.
            # Native ground contact already provides scrub; do not add it twice.
            torque=clamp(demand,-available,available) if ready and not power['needs_home'] and not power['brownout'] else 0.
            torques.append(torque)
            x=(1 if i<2 else -1)*(.27+4*self.status.get('stage_offset',0.));y=(1 if i%2==0 else -1)*.27
            localx,localy=vx-wz*y,vy+wz*x
            longitudinal=math.cos(angle)*localx+math.sin(angle)*localy
            lateral=-math.sin(angle)*localx+math.cos(angle)*localy
            patch,pressure,loaded_radius=contact_patch(normal,r['wheel_radius_m'],r['wheel_width_m'],self.p['suspension']['tyre_stiffness_N_m'])
            pred=traction_prediction(normal,omega[i],longitudinal,lateral,r['wheel_radius_m'],surface,self.env['wetness'])
            soft=soft_ground_prediction(pressure,r['wheel_width_m'],surface)
            step_loss=abs(demand)>available*st['safety_factor'] and abs(error)>.02
            wheels[n]={'surface':material,'contact_material_candidates':materials,'material_source':'native_collision_name' if materials else 'nominal_sector_fallback','normal_load_estimate_N':normal,'contact':contact,'suspension_m':spring,'bottom_out':spring>=self.p['suspension']['upper_m']-.001,'wheel_lift':not contact,'scrub_prediction_Nm':scrub,'steering_demand_Nm':demand,'steering_command_Nm':torque,'step_loss_risk':step_loss,'steering_error_rad':error,'patch_length_m':patch,'pressure_Pa':pressure,'loaded_radius_prediction_m':loaded_radius,**pred,**soft}
            name=n+'_wheel_joint';position=self.states.get(name,0.);counts_per_turn=self.p['sensors']['encoder_counts_per_wheel_turn']
            ideal_count=round(position*counts_per_turn/(2*math.pi));previous=self.encoder_previous.get(name,ideal_count);delta=ideal_count-previous;self.encoder_previous[name]=ideal_count
            limit=max(1,int(self.p['sensors']['encoder_max_edges_s']*dt));observed=int(clamp(delta,-limit,limit));self.missed+=abs(delta-observed)
            self.counts[name]=self.counts.get(name,ideal_count-delta)+observed
            quantized=self.counts[name]*2*math.pi/counts_per_turn;measured_speed=observed*2*math.pi/(counts_per_turn*dt)
            resolution=self.p['sensors']['magnetic_encoder_resolution_rad']
            measured_angle=round((angle+self.p['sensors']['magnetic_eccentricity_rad']*math.sin(2*angle)+self.p['sensors']['magnetic_temp_bias_rad_K']*(self.env['ambient_C']-22))/resolution)*resolution
            sensor.name.extend([name,n+'_steer_joint']);sensor.position.extend([quantized,measured_angle]);sensor.velocity.extend([measured_speed,speed])
            co,si=math.cos(measured_angle),math.sin(measured_angle)
            matrix.append([co,si,-co*y+si*x]);observations.append(measured_speed*r['wheel_radius_m'])
        # Minimum norm swerve odometry; lateral velocity is unobservable with parallel wheels.
        a=np.asarray(matrix);v_est=np.linalg.lstsq(a,np.asarray(observations),rcond=None)[0]
        yaw=self.enc_pose[2];self.enc_pose[0]+=dt*(math.cos(yaw)*v_est[0]-math.sin(yaw)*v_est[1]);self.enc_pose[1]+=dt*(math.sin(yaw)*v_est[0]+math.cos(yaw)*v_est[1]);self.enc_pose[2]+=dt*v_est[2]
        od=Odometry();od.header.stamp=sensor.header.stamp;od.header.frame_id='world';od.child_frame_id='encoder_base';od.pose.pose.position.x,od.pose.pose.position.y=map(float,self.enc_pose[:2]);od.pose.pose.orientation.z=math.sin(self.enc_pose[2]/2);od.pose.pose.orientation.w=math.cos(self.enc_pose[2]/2)
        od.twist.twist.linear.x,od.twist.twist.linear.y,od.twist.twist.angular.z=map(float,v_est)
        self.encoder_odom.publish(od);self.sensor_joints.publish(sensor)
        if self.actuate:
            for transition in (self.target_transition,self.motion_transition):
                if ready and transition is not None and 'first_effort_publication_wall_ns' not in transition:
                    transition['first_effort_publication_wall_ns']=time.monotonic_ns();transition['first_effort_publication_sim_ns']=self.get_clock().now().nanoseconds
            self.drive_pub.publish(Float64MultiArray(data=power['torques_Nm']))
        if now-self.logtime<.1:return
        self.logtime=now
        self.ambient_pub.publish(Float64(data=float(self.env['ambient_C'])))
        drift=math.hypot(self.enc_pose[0]-self.odom.pose.pose.position.x,self.enc_pose[1]-self.odom.pose.pose.position.y) if self.odom else None
        record={'timing':{'last_motion_transition':self.motion_transition,'last_target_transition':self.target_transition,'target_receipt_wall_s':self.target_wall,'target_receipt_sim_s':self.target_time,'effort_publication_wall_ns':time.monotonic_ns(),'joint_sample_sim_s':self.state_time,'joint_receipt_wall_s':self.state_wall},'surface_scope':'Nominal sector label; per-wheel materials are resolved from native contact collision names','sim_time_s':now,'position_m':([self.odom.pose.pose.position.x,self.odom.pose.pose.position.y,self.odom.pose.pose.position.z] if self.odom else None),'manager_status':self.status,'actuation_mode':('electrical_drive_effort_position_steering' if self.actuate else 'native_controllers_with_model_predictions'),'parameter_status':'A','force_scope':'Fx/Fy, scrub and soft-ground are predictions; Gazebo native contact applies forces','zone':self.zone,'surface':self.surface,'scenario':self.scenario,'environment':self.env,'power':power,'suspension_thermal':self.suspension_telemetry,'wheels':wheels,'roll_rad':roll,'pitch_rad':pitch,'side_tip_margin_rad':math.atan(r['track_m']/(2*r['com_height_m']))-abs(roll),'encoder_drift_m':drift,'encoder_matrix_rank':int(np.linalg.matrix_rank(a)),'encoder_missed_counts':self.missed,'imu_injection_rms':self.imu_residual,'feedback_ready':ready}
        line=json.dumps(record,allow_nan=False);self.log.write(line+'\n');self.telemetry.publish(String(data=line))


def main():
    lifecycle.initialize();node=InteractionRuntime()
    try:lifecycle.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node.actuate and rclpy.ok():
            node.drive_pub.publish(Float64MultiArray(data=[0.]*4));node.steer_pub.publish(Float64MultiArray(data=[0.]*4))
        node.log.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
