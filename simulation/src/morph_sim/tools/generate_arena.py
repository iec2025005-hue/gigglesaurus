from pathlib import Path
import numpy as np
from PIL import Image
import xml.etree.ElementTree as E
P=Path(__file__).resolve().parents[1];T=P/'models/materials/textures';T.mkdir(parents=True,exist_ok=True);H=P/'models/heightmaps';H.mkdir(exist_ok=True)
rng=np.random.default_rng(42);N=1024;yy,xx=np.mgrid[0:N,0:N];noise=rng.normal(0,1,(N,N))
colors={'asphalt':(43,51,61),'rock':(119,125,137),'sand':(210,168,94),'grass':(65,113,61),'cement':(156,168,178),'metal':(42,58,73),'cyan':(7,177,198),'rubber':(25,31,37)}
for name,color in colors.items():
 detail=noise*7
 if name=='sand':detail+=6*np.sin(xx/13+5*np.sin(yy/170))
 if name=='grass':detail+=10*np.sin(xx*1.8)*np.sin(yy/3)
 if name=='metal':detail+=2*np.sin(xx/2)
 arr=np.clip(np.asarray(color)[None,None,:]+detail[:,:,None],0,255).astype('uint8');Image.fromarray(arr).save(T/(name+'_albedo.png'))
 height=detail/80;dy,dx=np.gradient(height);normal=np.dstack((-dx,-dy,np.ones_like(dx)));normal/=np.linalg.norm(normal,axis=2)[:,:,None];Image.fromarray(((normal*.5+.5)*255).astype('uint8')).save(T/(name+'_normal.png'))
 Image.fromarray(np.full((N,N),int((.35 if name in ['metal','cyan'] else .90)*255),dtype='uint8')).save(T/(name+'_roughness.png'))
n=257;y,x=np.mgrid[-1:1:complex(n),-1:1:complex(n)];edge=np.clip((1-np.maximum(abs(x),abs(y)))/.2,0,1)
fields={
 'rock':(.45+.23*np.sin(14*x+3*np.sin(5*y))*np.cos(12*y)+.12*np.sin(30*x+21*y))*edge,
 'sand':(.35+.25*np.sin(5*x+1.4*np.sin(3*y)))*edge,
 'grass':(.25+.60*np.exp(-4*(x*x+y*y))+.10*np.sin(6*x)*np.cos(5*y))*edge,
}
for name,z in fields.items():Image.fromarray(np.clip(z*255,0,255).astype('uint8')).save(H/(name+'.png'))
def el(p,n,t=None,**attrs):
 a=E.SubElement(p,n,attrs)
 if t is not None:a.text=str(t)
 return a
sdf=E.Element('sdf',version='1.9');w=el(sdf,'world',name='morph_arena');el(w,'gravity','0 0 -9.81');phys=el(w,'physics',name='stable_1khz',type='ignored');el(phys,'max_step_size',.001);el(phys,'real_time_factor',1.)
for file,name in [('physics','Physics'),('user-commands','UserCommands'),('scene-broadcaster','SceneBroadcaster'),('contact','Contact'),('imu','Imu')]:el(w,'plugin',filename='gz-sim-'+file+'-system',name='gz::sim::systems::'+name)
sens=el(w,'plugin',filename='gz-sim-sensors-system',name='gz::sim::systems::Sensors');el(sens,'render_engine','ogre2')
scene=el(w,'scene');el(scene,'ambient','.36 .40 .47 1');el(scene,'background','.50 .64 .79 1');el(scene,'shadows','true');el(scene,'grid','false')
sun=el(w,'light',name='competition_sun',type='directional');el(sun,'pose','0 0 20 0 0 0');el(sun,'diffuse','1 .94 .83 1');el(sun,'specular','.7 .7 .7 1');el(sun,'direction','-.4 .3 -.85');el(sun,'cast_shadows','true')
att=el(sun,'attenuation');el(att,'range',1000);el(att,'constant',1);el(att,'linear',0);el(att,'quadratic',0)
def mat(v,name):
 m=el(v,'material');el(m,'ambient',' '.join(str(q/255) for q in colors[name])+' 1');el(m,'diffuse','1 1 1 1');p=el(el(m,'pbr'),'metal');el(p,'albedo_map','model://morph_sim/models/materials/textures/'+name+'_albedo.png');el(p,'normal_map','model://morph_sim/models/materials/textures/'+name+'_normal.png');el(p,'roughness_map','model://morph_sim/models/materials/textures/'+name+'_roughness.png');el(p,'roughness',.85);el(p,'metalness',.0)
def surface(c,mu,kp=1e7,kd=100):
 s=el(c,'surface');o=el(el(s,'friction'),'ode');el(o,'mu',mu);el(o,'mu2',mu);ct=el(el(s,'contact'),'ode');el(ct,'kp',kp);el(ct,'kd',kd);el(ct,'max_vel',.1);el(ct,'min_depth',.001)
def box(name,pose,size,material,mu=1.,color=None,collision=True):
 m=el(w,'model',name=name);el(m,'static','true');el(m,'pose',' '.join(map(str,pose)));l=el(m,'link',name='link')
 if collision:
  c=el(l,'collision',name='collision');el(el(el(c,'geometry'),'box'),'size',' '.join(map(str,size)));surface(c,mu)
 v=el(l,'visual',name='visual');el(el(el(v,'geometry'),'box'),'size',' '.join(map(str,size)))
 if color:
  mm=el(v,'material');el(mm,'ambient',color);el(mm,'diffuse',color)
 else:mat(v,material)
 return m
box('arena_floor',[0,0,-.06,0,0,0],[40,28,.12],'cement',.9)
for name,pose,size in [('road_south',[0,-9,.015,0,0,0],[30,3,.03]),('road_north',[0,9,.015,0,0,0],[30,3,.03]),('road_east',[14,0,.015,0,0,0],[3,18,.03]),('road_west',[-14,0,.015,0,0,0],[3,18,.03])]:box(name,pose,size,'asphalt',1.15)
for side in [-1,1]:
 for x0 in range(-13,14,2):box('lane_'+str(side)+'_'+str(x0),[x0,side*9,.032,0,0,0],[.9,.06,.003],'cement',color='.95 .87 .2 1',collision=False)
 for y0 in range(-7,8,2):box('lane_v_'+str(side)+'_'+str(y0),[side*14,y0,.032,0,0,0],[.06,.9,.003],'cement',color='.95 .87 .2 1',collision=False)
for name,cx,h,mu in [('rock',-8,.32,.85),('sand',0,.28,.35),('grass',8,.40,.65)]:
 m=el(w,'model',name=name+'_terrain');el(m,'static','true');el(m,'pose',f'{cx} 0 .005 0 0 0');l=el(m,'link',name='link')
 for kind in ['collision','visual']:
  child=el(l,kind,name=kind);hm=el(el(child,'geometry'),'heightmap');el(hm,'uri','model://morph_sim/models/heightmaps/'+name+'.png');el(hm,'size',f'6 6 {h}');el(hm,'pos','0 0 0');el(hm,'sampling',2)
  if kind=='visual':
   texture=el(hm,'texture');el(texture,'size',1);el(texture,'diffuse','model://morph_sim/models/materials/textures/'+name+'_albedo.png');el(texture,'normal','model://morph_sim/models/materials/textures/'+name+'_normal.png')
  else:surface(child,mu,50000 if name=='sand' else 1e7,500 if name=='sand' else 100)
# Steps start at 15 mm, graduated for the 80 mm demonstration wheels.
for i in range(5):
 h=.015*(i+1);box('cement_step_'+str(i),[7+i*.35,5,h/2,0,0,0],[.35,2,h],'cement',.95)
# Low ramp approximately 8 degrees: profile remains challenging for small wheels.
box('hill_ramp',[-1,5,.14,0,-.14,0],[2,2,.08],'grass',.75)
box('hill_platform',[.98,5,.24,0,0,0],[1,2,.12],'grass',.75)
for i in range(8):
 a=i*.7;box('rock_obstacle_'+str(i),[-10+(i%4)*.55,-5+(i//4)*.7,.025+.005*(i%3),0,.08,a],[.17,.20,.06],'rock',.85)
for x0 in [-18,18]:box('boundary_x_'+str(x0),[x0,0,.15,0,0,0],[.15,25,.3],'cyan',.8)
for y0 in [-12,12]:box('boundary_y_'+str(y0),[0,y0,.15,0,0,0],[36,.15,.3],'cyan',.8)
# Explicit GUI view without downloading Fuel assets.
gui=el(w,'gui',fullscreen='false');p=el(gui,'plugin',filename='MinimalScene',name='3D View');el(p,'engine','ogre2');el(p,'scene','scene');el(p,'ambient_light','.4 .4 .4');el(p,'background_color','.5 .65 .8');el(p,'camera_pose','20 -25 20 0 .52 2.22')
el(gui,'plugin',filename='WorldControl',name='World control');el(gui,'plugin',filename='WorldStats',name='World statistics')
E.indent(sdf);E.ElementTree(sdf).write(P/'worlds/competition.sdf',encoding='utf-8',xml_declaration=True)
print('World, 257x257 terrain maps and 1024px PBR textures generated')
