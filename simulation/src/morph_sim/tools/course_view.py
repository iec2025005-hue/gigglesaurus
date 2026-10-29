#!/usr/bin/env python3
"""Move the Gazebo camera to an overview or a named course entrance."""
import json,math,subprocess,sys
from pathlib import Path
import os,hashlib
os.environ['GZ_PARTITION']=os.environ.get('MORPH_GZ_PARTITION','morph_'+hashlib.sha256(str(Path.cwd()).encode()).hexdigest()[:12])
sectors=json.loads((Path(__file__).resolve().parents[1]/'config/sectors.json').read_text())
name=sys.argv[1] if len(sys.argv)>1 else 'overview'
if name=='overview':x,y,z=0,-25,48;yaw=math.pi/2;pitch=1.10
else:
 target=sectors['default_spawn'] if name in ('robot','start') else next((s['spawn'] for s in sectors['sectors'] if s['id']==name),None)
 if target is None:raise SystemExit('Choose overview, start, grass, sand, mud, ramps, debris, rocks, snow, mixed, village, pipeline, warehouse or rubble')
 x,y,z=target[0]-2,target[1]-3,2.1;yaw=math.atan2(3,2);pitch=math.atan2(1.8,math.sqrt(13))
qx=-math.sin(pitch/2)*math.sin(yaw/2);qy=math.sin(pitch/2)*math.cos(yaw/2);qz=math.cos(pitch/2)*math.sin(yaw/2);qw=math.cos(pitch/2)*math.cos(yaw/2)
request='pose: {position: {x: %s y: %s z: %s} orientation: {x: %s y: %s z: %s w: %s}} duration: 0.7' % (x,y,z,qx,qy,qz,qw)
r=subprocess.run(['gz','service','-s','/gui/move_to/pose','--reqtype','gz.msgs.GUICamera','--reptype','gz.msgs.Boolean','--timeout','3000','--req',request],capture_output=True,text=True)
if 'true' not in r.stdout:raise SystemExit('No running Gazebo GUI in this GZ_PARTITION: '+r.stdout+r.stderr)
print('Camera: '+name)
