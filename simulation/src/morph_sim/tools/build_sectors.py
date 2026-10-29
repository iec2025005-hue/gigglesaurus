#!/usr/bin/env python3
"""Compose imported OSRF assets and adapted Leo Rover DEM into connected sectors.

No primitive terrain: every driven surface is a textured triangle mesh with the
same visual/collision URI. DEM elevations are normalized for the small-wheel
course; this is an engineered derivative, not the original Marsyard geography.
"""
from pathlib import Path
import copy,json,math,shutil,sys
import xml.etree.ElementTree as E
import numpy as np
from PIL import Image,ImageDraw,ImageFont

P=Path(__file__).resolve().parents[1]
OUT=P/'models/morph_course';MESH=OUT/'meshes';TEX=OUT/'textures'
MESH.mkdir(parents=True,exist_ok=True);TEX.mkdir(exist_ok=True)
H=np.asarray(Image.open(P/'models/marsyard2021_terrain/dem/marsyard_terrain_hm.tif'),dtype=float)
LANES=[('smooth','SMOOTH ROAD','road',.001,.9),('grass','GREEN GRASS','grass',.008,.65),('sand','DRY SAND','sand',.018,.5),('mud','MUD / WET','mud',.012,.3),('ramps','RAMP LAB','concrete',.001,.85),('debris','DEBRIS','gravel',.014,.65),('rocks','ROCK GARDEN','gravel',.025,.7),('snow','SNOW SURFACE','snow',.008,.25),('mixed','MIXED TERRAIN','grass',.018,.55)]
root=E.parse(P/'worlds/adaptive_arena.sdf');world=root.find('world')
world.find('physics/dart/collision_detector').text='bullet'
for model in world.findall('model'):world.remove(model)
world.find('gui/plugin/camera_pose').text='1.0 -4.4 .85 0 .28 2.1910'
subgrid=world.find('gui/plugin/grid')
if subgrid is None:subgrid=E.SubElement(world.find('gui/plugin'),'grid')
subgrid.text='false'
scenegrid=world.find('scene/grid')
if scenegrid is None:scenegrid=E.SubElement(world.find('scene'),'grid')
scenegrid.text='false'
world.find('scene/ambient').text='.30 .34 .39 1';world.find('scene/background').text='.58 .72 .86 1'
world.find("light[@name='sun']/direction").text='-.35 .4 -1'
world.find("light[@name='sun']/diffuse").text='1.0 .94 .84 1'
course=E.SubElement(world,'model',name='MORPH_MULTITERRAIN_COURSE');E.SubElement(course,'static').text='true'
link=E.SubElement(course,'link',name='survey_terrain')
manifest={'world':'leo_marsyard','display_name':'MORPH FIELD LAB — CONNECTED SECTORS','default_spawn':[0.,-3.,.38],'sectors':[],'source_dem':'Leo Rover ERC2021 Marsyard','terrain_derivation':'Bilinear resampling, elevation normalization and edge taper for the original 40 mm wheel baseline; current robot uses 75 mm radius','snow_scope':'Rigid textured surface with assumed low Coulomb friction; no deformable snow physics','route':[[-20,-3],[20,-3],[20,15],[-20,15],[-20,-3]],'native_physics':'DART/Bullet collision; native mu/mu2, no deformable soil'}

def sub(parent,tag,text=None,**attrs):
 n=E.SubElement(parent,tag,attrs)
 if text is not None:n.text=str(text)
 return n

def surface_sample(x,y):
 # Existing terrain source: deterministic central patch, no invented terrain noise.
 u=np.clip((x+32)/64*(H.shape[1]-2),0,H.shape[1]-2)
 v=np.clip((20-y)/44*(H.shape[0]-2),0,H.shape[0]-2)
 i=u.astype(int);j=v.astype(int);a=u-i;b=v-j
 return (1-a)*(1-b)*H[j,i]+a*(1-b)*H[j,i+1]+(1-a)*b*H[j+1,i]+a*b*H[j+1,i+1]

def obj_grid(name,x0,x1,y0,y1,amp,texture,mu,step=.18):
 nx=max(2,int(round((x1-x0)/step))+1);ny=max(2,int(round((y1-y0)/step))+1)
 xs=np.linspace(x0,x1,nx);ys=np.linspace(y0,y1,ny);x,y=np.meshgrid(xs,ys)
 h=surface_sample(x,y);design=np.stack([x.ravel(),y.ravel(),np.ones(x.size)],axis=1)
 coeff=np.linalg.lstsq(design,h.ravel(),rcond=None)[0];h=h-(design@coeff).reshape(h.shape)
 h=h/max(float(np.max(np.abs(h))),.001)*amp
 taper=np.clip(np.minimum.reduce([x-x0,x1-x,y-y0,y1-y])/.5,0,1)
 h*=taper # all sector seams are exactly z=0, no hidden overlap.
 gy,gx=np.gradient(h,ys[1]-ys[0],xs[1]-xs[0]);norm=np.stack([-gx,-gy,np.ones_like(h)],axis=-1);norm/=np.linalg.norm(norm,axis=-1)[...,None]
 path=MESH/(name+'.obj')
 with path.open('w') as f:
  f.write(f'mtllib {texture}.mtl\no {name}\n')
  for xx,yy,zz in zip(x.ravel(),y.ravel(),h.ravel()):f.write(f'v {xx:.6f} {yy:.6f} {zz:.6f}\n')
  for xx,yy in zip(x.ravel(),y.ravel()):f.write(f'vt {xx/2:.6f} {yy/2:.6f}\n')
  for normal in norm.reshape(-1,3):f.write('vn %.6f %.6f %.6f\n'%tuple(normal))
  f.write('usemtl Surface\ns 1\n')
  for j in range(ny-1):
   for i in range(nx-1):
    a=j*nx+i+1
    for tri in [(a,a+1,a+nx),(a+1,a+nx+1,a+nx)]:f.write('f '+' '.join(f'{v}/{v}/{v}' for v in tri)+'\n')
 (MESH/(texture+'.mtl')).write_text(f'newmtl Surface\nKa 1 1 1\nKd 1 1 1\nKs 0 0 0\nmap_Kd ../textures/{texture}_albedo.jpg\n')
 uri=f'model://morph_course/meshes/{name}.obj'
 for kind in ('visual','collision'):
  e=sub(link,kind,name=name+'_'+kind);sub(sub(sub(e,'geometry'),'mesh'),'uri',uri)
  if kind=='visual':
   mat=sub(e,'material');sub(mat,'ambient','1 1 1 1');sub(mat,'diffuse','1 1 1 1');metal=sub(sub(mat,'pbr'),'metal')
   for tag,suffix in [('albedo_map','albedo'),('normal_map','normal'),('roughness_map','roughness')]:
    if (TEX/f'{texture}_{suffix}.jpg').exists():sub(metal,tag,f'model://morph_course/textures/{texture}_{suffix}.jpg')
   sub(metal,'metalness',0);sub(metal,'roughness',.35 if texture=='mud' else .9)
  else:
   ode=sub(sub(sub(e,'surface'),'friction'),'ode');sub(ode,'mu',mu);sub(ode,'mu2',mu*.95)
 return path

# Nine separate columns; clear crossways in front and behind all lanes.
for i,(name,label,texture,amp,mu) in enumerate(LANES):
 x0=-18+4*i;x1=x0+4
 if name=='mixed':
  for j,t in enumerate(['grass','sand','mud','gravel']):obj_grid(name+str(j),x0,x1,j*3,(j+1)*3,amp,t,mu)
 else:obj_grid(name,x0,x1,0,12,amp,texture,mu)
 manifest['sectors'].append({'id':name,'label':label,'bounds':[x0,x1,0,12],'surface':{'grass':'short_grass','sand':'dry_sand','rocks':'wet_rock','smooth':'tarmac','ramps':'concrete','debris':'loose_gravel','mixed':'packed_dirt','snow':'snow'}.get(name,name),'spawn':[x0+2,1.,.38],'mu':mu,'mu2':mu*.95})
# Nonoverlapping surroundings and access roads. All share boundary elevation 0.
for name,bounds,tex in [('front_crossway',[-32,32,-6,0],'road'),('back_crossway',[-32,32,12,17],'road'),('west_service',[-32,-18,0,12],'concrete'),('east_village',[18,32,0,12],'grass'),('depot_forecourt',[-32,32,-24,-6],'concrete'),('north_landscape',[-32,32,17,20],'grass')]:obj_grid(name,*bounds,.002,tex,.8,step=.35)

# Source models: keep their meshes and textures, transform whole assets consistently.
def asset(source,name,x,y,z=0,scale=(1,1,1),yaw=0):
 t=E.parse(P/'models'/source/'model.sdf');m=copy.deepcopy(t.find('model'));m.set('name',name)
 static=m.find('static')
 if static is None:static=sub(m,'static')
 static.text='true'
 # Root pose is source-relative; scale its position, then wrap in a placement model.
 for pose in m.findall('.//pose'):
  vals=list(map(float,pose.text.split()))
  if len(vals)==6:pose.text=' '.join(map(str,[vals[i]*scale[i] for i in range(3)]+vals[3:]))
  pose.attrib.pop('frame',None)
 for l in m.findall('.//link'):
  inertial=l.find('inertial')
  if inertial is not None:l.remove(inertial)
 for mesh in m.findall('.//geometry/mesh'):
  s=mesh.find('scale');v=[1,1,1] if s is None else list(map(float,s.text.split()))
  if s is None:s=sub(mesh,'scale')
  s.text=' '.join(str(a*b) for a,b in zip(v,scale))
 for geometry in m.findall('.//geometry'):
  for tag in ['box/size','plane/size']:
   e=geometry.find(tag)
   if e is not None:e.text=' '.join(str(a*b) for a,b in zip(map(float,e.text.split()),scale))
  for tag,sc in [('cylinder/radius',scale[0]),('sphere/radius',scale[0]),('cylinder/length',scale[2])]:
   e=geometry.find(tag)
   if e is not None:e.text=str(float(e.text)*sc)
 for collision in m.findall('.//collision'):
  surf=collision.find('surface')
  if surf is None:surf=sub(collision,'surface')
  fr=surf.find('friction')
  if fr is None:fr=sub(surf,'friction')
  ode=fr.find('ode')
  if ode is None:ode=sub(fr,'ode')
  for key in ['mu','mu2']:
   e=ode.find(key)
   if e is None:e=sub(ode,key)
   e.text='.8'
 # SDF1 classic material scripts are not supported by Ogre2. Bind real texture maps.
 for visual in m.findall('.//visual'):
  mat=visual.find('material')
  if mat is not None:
   for tag in ('ambient','diffuse'):
    color=mat.find(tag)
    if color is None:color=sub(mat,tag)
    color.text='1 1 1 1'
   script=mat.find('script');shader=mat.find('shader')
   if script is not None:
    mat.remove(script)
    textures=list((P/'models'/source/'materials/textures').glob('*'))
    token='branch' if 'branch' in visual.get('name','').lower() else 'bark' if 'bark' in visual.get('name','').lower() else 'diffuse'
    albedo=next((f for f in textures if token in f.name.lower()),None)
    if albedo:
     for old in mat.findall('pbr'):mat.remove(old)
     metal=sub(sub(mat,'pbr'),'metal');sub(metal,'albedo_map',f'model://{source}/materials/textures/{albedo.name}');sub(metal,'roughness',.8);sub(metal,'metalness',0)
   if shader is not None:mat.remove(shader)
 wrapper=sub(world,'model',name=name+'_placement');sub(wrapper,'static','true');sub(wrapper,'pose',f'{x} {y} {z} 0 0 {yaw}');wrapper.append(m)
 return wrapper

# Actual imported NIST ramp meshes; 5 to25 degrees, rise <=0.19m, 0.75m long.
for j,deg in enumerate([5,10,15,20,25]):
 sy=.75/1.18;sz=(.75*math.tan(math.radians(deg)))/1.2
 asset('nist_simple_ramp_120',f'nist_ramp_{deg}',0,2+j*1.9,.002,scale=(1.,sy,sz),yaw=0)
# Imported NIST staircase as an optional dedicated fixture beside the ramps.
asset('nist_stairs_120','nist_scaled_steps',1.25,8,.002,scale=(.3,.3,.04))
# Separators: source barriers placed along lane boundaries, openings both ends.
for i in range(10):
 x=-18+4*i
 for y in [3.,6.,9.]:asset('drc_practice_orange_jersey_barrier',f'divider_{i}_{int(y)}',x,y,0,scale=(.35,1.0,.3))
# Imported village, trees and inspection infrastructure.
asset('house_1','village_home_1',25,4,0,scale=(.7,.7,.7))
asset('house_2','village_home_2',26,11,0,scale=(.55,.55,.55),yaw=math.pi)
asset('powerplant','pipeline_station',-26,7,0,scale=(.14,.14,.14))
for j in range(5):asset('standpipe',f'inspection_valve_{j}',-21.5,1+j*2.1,0,scale=(1.8,1.8,1.8))
asset('collapsed_house','rubble_house',-25,-15,0,scale=(.7,.7,.7))
asset('collapsed_industrial','rubble_industrial',24,-15,0,scale=(.2,.2,.2))
# Reduced source rubble fragments create actual mesh obstacles inside the debris lane.
for j,(x,y,sc) in enumerate([(2.8,3,.06),(4.8,5,.08),(3.4,8,.05),(5,10,.07)]):asset('collapsed_house',f'debris_mesh_{j}',x,y,-.006,scale=(sc,sc,sc),yaw=j*.8)
# Extract a connected scanned boulder from the existing source rock field.
lines=(P/'models/marsyard2021_terrain/meshes/rocks_big.obj').read_text().splitlines()
verts=[];faces=[]
for line in lines:
 if line.startswith('v '):verts.append(list(map(float,line.split()[1:4])))
 elif line.startswith('f '):faces.append([int(t.split('/')[0])-1 for t in line.split()[1:]])
parent=list(range(len(verts)))
def find(a):
 while parent[a]!=a:parent[a]=parent[parent[a]];a=parent[a]
 return a
for face in faces:
 for v in face[1:]:parent[find(v)]=find(face[0])
groups={}
for face in faces:groups.setdefault(find(face[0]),[]).append(face)
selected=sorted(groups.values(),key=len,reverse=True)[min(2,len(groups)-1)]
used=sorted({v for f in selected for v in f});mapping={v:i+1 for i,v in enumerate(used)}
v=np.asarray(verts)[used];v-=np.array([(v[:,0].min()+v[:,0].max())/2,(v[:,1].min()+v[:,1].max())/2,v[:,2].min()]);v/=max(v[:,2].max(),.01)
rock=MESH/'scanned_rock.obj'
with rock.open('w') as out:
 out.write('o Leo_ERC2021_scanned_boulder\n')
 for row in v:out.write('v %.7f %.7f %.7f\n'%tuple(row))
 for row in v:out.write('vt %.7f %.7f\n'%(row[0],row[1]))
 for face in selected:
  a,b,c=[v[mapping[i]-1] for i in face[:3]];normal=np.cross(b-a,c-a);normal/=max(np.linalg.norm(normal),1e-12);out.write('vn %.7f %.7f %.7f\n'%tuple(normal))
 for face_index,face in enumerate(selected):out.write('f '+' '.join(f'{mapping[i]}/{mapping[i]}/{face_index+1}' for i in face)+'\n')
rock_heights=[max(float(l.split()[3]) for l in (P/'models/ph_rock_moss_set_01'/f'rock_{i}.obj').read_text().splitlines() if l.startswith('v ')) for i in range(6)]
for j in range(34):
 x=6.4+(j%5)*.64;y=2+(j//5)*1.35
 height=.02+.007*(j%4) if j%5 in (2,3) else .08+.025*(j%3)
 rock_scale=height/rock_heights[j%6]
 m=sub(world,'model',name=f'scanned_rock_{j}');sub(m,'static','true');sub(m,'pose',f'{x} {y} -.004 0 0 {j*.8}');l=sub(m,'link',name='rock')
 for kind in ['visual','collision']:
  e=sub(l,kind,name=kind);mesh=sub(sub(e,'geometry'),'mesh');sub(mesh,'uri',f'model://ph_rock_moss_set_01/rock_{j%6}.obj');sub(mesh,'scale',f'{rock_scale} {rock_scale} {rock_scale}')
  if kind=='visual':
   mat=sub(e,'material');sub(mat,'ambient','1 1 1 1');sub(mat,'diffuse','1 1 1 1');metal=sub(sub(mat,'pbr'),'metal');sub(metal,'albedo_map','model://ph_rock_moss_set_01/textures/rock_moss_set_01_diff_1k.jpg');sub(metal,'roughness',.9);sub(metal,'metalness',0)
# Trees provide scale and visual separation, all from the OSRF library.
for j,(x,y) in enumerate([(-16,18),(-10,18),(-4,18),(4,18),(10,18),(16,18),(-30,0),(-30,11),(30,0),(30,14),(-15,-8),(15,-8)]):
 asset('ph_tree_small_02',f'tree_{j}',x,y,0,scale=(.65,.65,.65))
# Open depot section from existing AWS package; no enclosing wall prevents access.
for j,x in enumerate([-14,-10,-6]):asset('aws_robomaker_warehouse_ShelfD_01',f'depot_rack_{j}',x,-16,0)
asset('aws_robomaker_warehouse_PalletJackB_01','depot_palletjack',-10,-11,0)
for j,(x,y) in enumerate([(-2,-5),(2,-5),(-19,-1),(19,-1),(-19,13),(19,13)]):asset('construction_cone',f'cone_{j}',x,y,0,scale=(.5,.5,.5))

# Text signs are presentation annotations, not stand-in terrain geometry.
def sign(name,title,subtitle,x,y,width=3.7,z=.015):
 im=Image.new('RGB',(1024,256),'#101e2c');d=ImageDraw.Draw(im)
 font='/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
 d.rectangle((8,8,1015,247),outline='#33d6bf',width=7)
 for text,yy,size,color in [(title,40,60,'white'),(subtitle,140,31,'#9fc6d4')]:
  f=ImageFont.truetype(font,size);box=d.textbbox((0,0),text,font=f);d.text(((1024-(box[2]-box[0]))/2,yy),text,font=f,fill=color)
 im.save(TEX/(name+'.png'))
 path=MESH/(name+'.obj');height=width/4
 path.write_text(f'mtllib {name}.mtl\no {name}\nv {-width/2} {-height/2} 0\nv {width/2} {-height/2} 0\nv {width/2} {height/2} 0\nv {-width/2} {height/2} 0\nvt 0 0\nvt 1 0\nvt 1 1\nvt 0 1\nvn 0 0 1\nusemtl Sign\nf 1/1/1 2/2/1 3/3/1\nf 1/1/1 3/3/1 4/4/1\n')
 (MESH/(name+'.mtl')).write_text(f'newmtl Sign\nKa 1 1 1\nKd 1 1 1\nmap_Kd ../textures/{name}.png\n')
 v=sub(link,'visual',name=name);sub(v,'pose',f'{x} {y} {z} 0 0 0');sub(sub(sub(v,'geometry'),'mesh'),'uri',f'model://morph_course/meshes/{name}.obj')
 mat=sub(v,'material');sub(mat,'ambient','1 1 1 1');sub(mat,'diffuse','1 1 1 1');sub(mat,'emissive','.12 .12 .12 1');metal=sub(sub(mat,'pbr'),'metal');sub(metal,'albedo_map',f'model://morph_course/textures/{name}.png');sub(metal,'roughness',1);sub(metal,'metalness',0)
for i,(name,title,tex,amp,mu) in enumerate(LANES):sign('label_'+name,f'{i+1:02d}  {title}','ENTER  /  DRIVE NORTH',-16+i*4,-.8)
sign('start','MORPH  /  FIELD LAB','SWERVE • TRANSFORM • EXPLORE',0,-4.7,width=8)
sign('pipeline_label','PIPELINE','INSPECTION SECTOR',-25,-1.5,width=5)
sign('village_label','VILLAGE','ROAD / ACCESS TRIAL',25,-1.5,width=5)
sign('depot_label','WAREHOUSE','OPEN LOGISTICS DEPOT',-10,-8,width=6)
sign('rubble_label','RUBBLE','DISASTER RESPONSE',24,-8,width=6)
for item in [('pipeline',[-32,-18,0,12],'concrete',[-20,1,.38]),('village',[18,32,0,12],'short_grass',[19,0.5,.38]),('warehouse',[-18,-2,-22,-8],'concrete',[-10,-10,.38]),('rubble',[18,32,-22,-8],'loose_gravel',[20,-10,.38])]:
 name,bounds,surface,spawn=item;manifest['sectors'].append({'id':name,'label':name.upper(),'bounds':bounds,'surface':surface,'spawn':spawn})
# Continuous survey-derived landscape surrounding the test facility. The inner
# boundary meets the course at z=0; both rendering and contact use this mesh.
xs=np.linspace(-80,80,161);ys=np.linspace(-72,68,141);xx,yy=np.meshgrid(xs,ys)
raw=surface_sample(xx*.35+8,yy*.35-5)
distance=np.maximum.reduce([np.abs(xx)-32,yy-20,-24-yy,np.zeros_like(xx)])
zz=np.maximum(raw+.8,0)*np.clip(distance/12,0,1)*1.2
verts=np.stack([xx,yy,zz],axis=-1).reshape(-1,3);faces=[]
for j in range(len(ys)-1):
 for i in range(len(xs)-1):
  cx=(xs[i]+xs[i+1])/2;cy=(ys[j]+ys[j+1])/2
  if -32<cx<32 and -24<cy<20:continue
  a=j*len(xs)+i;faces.extend([(a,a+1,a+len(xs)),(a+1,a+len(xs)+1,a+len(xs))])
gy,gx=np.gradient(zz,1.,1.);normal=np.stack([-gx,-gy,np.ones_like(zz)],axis=-1);normal/=np.linalg.norm(normal,axis=-1)[...,None]
with (MESH/'surrounding_landscape.obj').open('w') as out:
 out.write('mtllib grass.mtl\no SurveyLandscape\n')
 for v in verts:out.write('v %.6f %.6f %.6f\n'%tuple(v))
 for v in verts:out.write('vt %.6f %.6f\n'%(v[0]/2,v[1]/2))
 for v in normal.reshape(-1,3):out.write('vn %.6f %.6f %.6f\n'%tuple(v))
 out.write('usemtl Surface\ns 1\n')
 for face in faces:out.write('f '+' '.join(f'{v+1}/{v+1}/{v+1}' for v in face)+'\n')
for kind in ('visual','collision'):
 element=sub(link,kind,name='surrounding_landscape_'+kind);sub(sub(sub(element,'geometry'),'mesh'),'uri','model://morph_course/meshes/surrounding_landscape.obj')
 if kind=='visual':
  mat=sub(element,'material');sub(mat,'ambient','1 1 1 1');sub(mat,'diffuse','1 1 1 1');metal=sub(sub(mat,'pbr'),'metal');sub(metal,'albedo_map','model://morph_course/textures/grass_albedo.jpg');sub(metal,'normal_map','model://morph_course/textures/grass_normal.jpg');sub(metal,'roughness',.95);sub(metal,'metalness',0)
 else:
  ode=sub(sub(sub(element,'surface'),'friction'),'ode');sub(ode,'mu',.65);sub(ode,'mu2',.6)
# Source trees make the boundaries readable at robot-eye height.
for j,(x,y) in enumerate([(-35,-15),(-35,-2),(-35,16),(-29,23),(-16,24),(-3,24),(10,23),(23,24),(35,16),(35,3),(35,-10),(27,-27),(10,-27),(-8,-27),(-26,-27)]):
 h=max(float(surface_sample(np.array(x*.35+8),np.array(y*.35-5)))+.8,0)*min(max(abs(x)-32,y-20,-24-y,0)/12,1)*1.2
 sc=.36+.07*(j%3);asset('oak_tree',f'landscape_oak_{j}',x,y,h,scale=(sc,sc,sc),yaw=j*.71)

# Consolidate static scenery into one rigid body. This prevents unnecessary
# scenery-to-scenery contact work while preserving every triangle and placement.
def pose_matrix(text):
 vals=list(map(float,(text or '0 0 0 0 0 0').split()));x,y,z,r,p,yaw=vals
 cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(yaw),math.sin(yaw)
 matrix=np.eye(4);matrix[:3,:3]=[[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]];matrix[:3,3]=[x,y,z];return matrix
def pose_text(matrix):
 r=math.atan2(matrix[2,1],matrix[2,2]);p=math.asin(max(-1,min(1,-matrix[2,0])));yaw=math.atan2(matrix[1,0],matrix[0,0])
 return ' '.join(map(str,[*matrix[:3,3],r,p,yaw]))
scenery=sub(world,'model',name='imported_static_scenery');sub(scenery,'static','true');scenery_link=sub(scenery,'link',name='scenery');sub(scenery_link,'self_collide','false')
count=[0]
def flatten(model,parent_matrix):
 matrix=parent_matrix@pose_matrix(model.findtext('pose'))
 for l in model.findall('link'):
  lm=matrix@pose_matrix(l.findtext('pose'))
  for item in list(l):
   if item.tag not in ('collision','visual','light'):continue
   element=copy.deepcopy(item);count[0]+=1;element.set('name','scenery_'+str(count[0]))
   old=element.find('pose');em=lm@pose_matrix(element.findtext('pose'))
   if old is None:old=sub(element,'pose')
   old.text=pose_text(em);old.attrib.clear();scenery_link.append(element)
 for nested in model.findall('model'):flatten(nested,matrix)
for model in list(world.findall('model')):
 if model is course or model is scenery:continue
 flatten(model,np.eye(4));world.remove(model)
(OUT/'model.config').write_text('<model><name>MORPH course assets</name><version>1</version><sdf version="1.8">model.sdf</sdf><description>Adapted source DEM and CC0 surface textures.</description></model>')
# Native materials and diagnostics resolve from the same authoritative table.
sys.path.insert(0,str(P))
from morph_sim.parameters import resolve_profile
from morph_sim.surface_parameters import synchronize_world
parameters,_=resolve_profile(P)
synchronize_world(root,P,parameters)
for sector in manifest['sectors']:
 surface=parameters['surfaces'][sector['surface']]
 sector['mu']=surface['mu_kinetic_x'];sector['mu2']=surface['mu_kinetic_y']
# Standalone model holds the terrain; world embeds the same tree so scenery remains visible.
modelroot=E.Element('sdf',version='1.8');modelroot.append(copy.deepcopy(course));E.indent(modelroot);E.ElementTree(modelroot).write(OUT/'model.sdf',encoding='utf-8',xml_declaration=True)
E.indent(root);root.write(P/'worlds/multisector.sdf',encoding='utf-8',xml_declaration=True)
(P/'config/sectors.json').write_text(json.dumps(manifest,indent=2))
print(f'Wrote multisector.sdf: {len(scenery_link.findall("visual"))} scenery visuals in one static body, {len(manifest["sectors"])} sectors')

if __name__=='__main__':pass
