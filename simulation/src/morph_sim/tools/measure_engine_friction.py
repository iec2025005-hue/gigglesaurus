#!/usr/bin/env python3
"""Measure effective Coulomb sliding friction on isolated 45-degree fixtures.

This is an engine coefficient test, not a tyre/soil or hardware calibration.
Fixtures are analytical metrology blocks, not presentation environment assets.
"""
from pathlib import Path
import json,math,os,signal,subprocess,sys,time,uuid
import xml.etree.ElementTree as E
import numpy as np
P=Path(__file__).resolve().parents[1];sys.path.insert(0,str(P))
from morph_sim.parameters import resolve_profile


def sub(parent,tag,text=None,**attrs):
    item=E.SubElement(parent,tag,attrs)
    if text is not None:item.text=str(text)
    return item


def main():
    out=Path(sys.argv[1] if len(sys.argv)>1 else 'validation/friction');out.mkdir(parents=True,exist_ok=True)
    params,_=resolve_profile(P);root=E.Element('sdf',version='1.8');world=sub(root,'world',name='friction_fixture')
    sub(world,'gravity','0 0 -9.81');physics=sub(world,'physics',name='physics',type='ignored');sub(physics,'max_step_size',.002);sub(physics,'real_time_factor',1);sub(sub(physics,'dart'),'collision_detector','bullet')
    sub(world,'plugin',filename='gz-sim-physics-system',name='gz::sim::systems::Physics')
    protocol=sys.argv[2] if len(sys.argv)>2 else 'incline'
    if protocol not in ('incline','pull'):raise ValueError('Protocol must be incline or pull')
    angle=math.pi/4 if protocol=='incline' else 0.;cases=[]
    wrench=sub(world,'plugin',filename='gz-sim-apply-link-wrench-system',name='gz::sim::systems::ApplyLinkWrench') if protocol=='pull' else None
    for axis in ('x','y'):
        for i,(surface,profile) in enumerate(params['surfaces'].items()):
            name=f'{axis}_{surface}';mu=profile['mu_kinetic_x'];mu2=profile['mu_kinetic_y'];z=0 if axis=='x' else 100
            x,y=(0,i*2) if axis=='x' else (i*2,0)
            roll,pitch=(0,angle) if axis=='x' else (-angle,0)
            normal=(math.sin(angle),0,math.cos(angle)) if axis=='x' else (0,math.sin(angle),math.cos(angle))
            for static in (True,False):
                model=sub(world,'model',name=name+('_ground' if static else '_block'));sub(model,'static',str(static).lower())
                pose=[x,y,z,roll,pitch,0] if static else [x+normal[0]*.016,y+normal[1]*.016,z+normal[2]*.016,roll,pitch,0]
                sub(model,'pose',' '.join(map(str,pose)));link=sub(model,'link',name='body')
                if not static:
                    inertial=sub(link,'inertial');sub(inertial,'mass',1);inertia=sub(inertial,'inertia')
                    for key,value in [('ixx',(.4**2+.02**2)/12),('iyy',(.4**2+.02**2)/12),('izz',(.4**2+.4**2)/12)]:sub(inertia,key,value)
                collision=sub(link,'collision',name='contact');sub(sub(sub(collision,'geometry'),'box'),'size','50 1 .01' if static and axis=='x' else '1 50 .01' if static else '.4 .4 .02')
                ode=sub(sub(sub(collision,'surface'),'friction'),'ode');sub(ode,'mu',mu if static else 1.2);sub(ode,'mu2',mu2 if static else 1.2);sub(ode,'fdir1','1 0 0')
                if not static:
                    plugin=sub(model,'plugin',filename='gz-sim-odometry-publisher-system',name='gz::sim::systems::OdometryPublisher');sub(plugin,'dimensions',3);sub(plugin,'odom_publish_frequency',100);sub(plugin,'odom_topic','/friction/'+name)
            if wrench is not None:
                persistent=sub(wrench,'persistent');sub(persistent,'entity_name',name+'_block');sub(persistent,'entity_type','model')
                applied=1.2*(mu if axis=='x' else mu2)*9.81
                sub(persistent,'force',f'{applied} 0 0' if axis=='x' else f'0 {applied} 0')
            cases.append((name,axis,mu if axis=='x' else mu2))
    file=out/'fixture.sdf';E.indent(root);E.ElementTree(root).write(file,encoding='utf-8',xml_declaration=True)
    env=dict(os.environ,GZ_PARTITION='morph_friction_'+uuid.uuid4().hex,GZ_IP='127.0.0.1')
    serverlog=(out/'server.log').open('w');server=subprocess.Popen(['gz','sim','-s',str(file.resolve())],env=env,stdout=serverlog,stderr=subprocess.STDOUT,start_new_session=True)
    subscriptions=[];results=[]
    try:
        time.sleep(2)
        for name,axis,expected in cases:
            handle=(out/(name+'.jsonl')).open('w')
            process=subprocess.Popen(['gz','topic','-e','--json-output','-t','/friction/'+name,'-d','9'],env=env,stdout=handle,stderr=subprocess.DEVNULL)
            subscriptions.append((process,handle,name,axis,expected))
        time.sleep(2)
        start=subprocess.run(['gz','service','-s','/world/friction_fixture/control','--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','5000','--req','pause: false'],env=env,capture_output=True,text=True)
        if start.returncode or 'true' not in start.stdout:raise RuntimeError('Fixture unpause failed: '+start.stdout+start.stderr)
        for process,handle,*_ in subscriptions:process.wait(timeout=15);handle.close()
        for _,_,name,axis,expected in subscriptions:
            raw=(out/(name+'.jsonl')).read_text();decoder=json.JSONDecoder();messages=[]
            while raw.strip():
                raw=raw.lstrip();message,end=decoder.raw_decode(raw);messages.append(message);raw=raw[end:]
            samples=[]
            for message in messages:
                stamp=message.get('header',{}).get('stamp',{});t=float(stamp.get('sec',0))+float(stamp.get('nsec',0))*1e-9
                position=message.get('pose',{}).get('position',{})
                if .5<=t<=2.5:samples.append((t,float(position.get(axis,0))*math.cos(angle)-float(position.get('z',0))*math.sin(angle)))
            if len(samples)<30:
                results.append(dict(case=name,status='insufficient_samples',samples=len(samples)));continue
            t,s=np.asarray(samples).T;fit=np.polyfit(t,s,2);accel=2*fit[0];effective=(9.81*math.sin(angle)-accel)/(9.81*math.cos(angle)) if protocol=='incline' else (1.2*expected*9.81-accel)/9.81;rmse=float(np.sqrt(np.mean((s-np.polyval(fit,t))**2)))
            results.append(dict(case=name,expected_mu=expected,effective_mu=float(effective),error=float(effective-expected),acceleration_m_s2=float(accel),fit_rmse_m=rmse,samples=len(samples),passed=bool(abs(effective-expected)<.05 and rmse<.01)))
        report=dict(scope='Gazebo DART/Bullet analytical sliding-block fixture; not tyre or hardware friction validation',timestep_s=.002,protocol=protocol,slope_deg=math.degrees(angle),pull_force_multiple_of_nominal_friction=1.2 if protocol=='pull' else None,fit_interval_sim_s=[.5,2.5],tolerance_mu=.05,results=results,passed=all(r.get('passed',False) for r in results))
        (out/'summary.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
        return 0 if report['passed'] else 1
    finally:
        for process,handle,*_ in subscriptions:
            if process.poll() is None:process.terminate();process.wait(timeout=5)
            handle.close()
        if server.poll() is None:
            os.killpg(server.pid,signal.SIGINT)
            try:server.wait(timeout=10)
            except subprocess.TimeoutExpired:os.killpg(server.pid,signal.SIGKILL);server.wait()
        serverlog.close()


if __name__=='__main__':raise SystemExit(main())
