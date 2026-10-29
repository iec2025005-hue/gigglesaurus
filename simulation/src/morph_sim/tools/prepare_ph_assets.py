#!/usr/bin/env python3
"""Convert downloaded CC0 Poly Haven glTF assets into local Gazebo mesh LODs.
Uses deterministic geometric decimation and nearest-source-vertex UV transfer.
Collision: tree trunk only; rock collision matches its visual triangle mesh.
"""
from pathlib import Path
import numpy as np,trimesh as tm
from scipy.spatial import cKDTree
import xml.etree.ElementTree as E
P=Path(__file__).resolve().parents[1]/'models'
def sub(p,t,text=None,**a):
 e=E.SubElement(p,t,a)
 if text is not None:e.text=str(text)
 return e
def export(m,path):
 with path.open('w') as f:
  f.write('o ImportedSource\n')
  for v in m.vertices:f.write('v %.7f %.7f %.7f\n'%tuple(v))
  for uv in m.visual.uv:f.write('vt %.7f %.7f\n'%tuple(uv))
  for v in m.vertex_normals:f.write('vn %.7f %.7f %.7f\n'%tuple(v))
  for face in m.faces:f.write('f '+' '.join(f'{i+1}/{i+1}/{i+1}' for i in face)+'\n')
def model(directory,name,parts):
 r=E.Element('sdf',version='1.8');m=sub(r,'model',name=name);sub(m,'static','true');l=sub(m,'link',name='source')
 for part,collision,albedo,normal in parts:
  for kind in ('visual','collision') if collision else ('visual',):
   v=sub(l,kind,name=part+'_'+kind);sub(sub(sub(v,'geometry'),'mesh'),'uri',f'model://{directory.name}/{part}.obj')
   if kind=='visual':
    mat=sub(v,'material');sub(mat,'ambient','1 1 1 1');sub(mat,'diffuse','1 1 1 1');metal=sub(sub(mat,'pbr'),'metal');sub(metal,'albedo_map',f'model://{directory.name}/textures/{albedo}');sub(metal,'normal_map',f'model://{directory.name}/textures/{normal}');sub(metal,'roughness',.86);sub(metal,'metalness',0)
   else:
    ode=sub(sub(sub(v,'surface'),'friction'),'ode');sub(ode,'mu',.7);sub(ode,'mu2',.65)
 E.indent(r);E.ElementTree(r).write(directory/(name+'.sdf'),encoding='utf-8',xml_declaration=True)
 return r
rot=tm.transformations.rotation_matrix(np.pi/2,[1,0,0])
for name in ['tree_small_02','rock_moss_set_01']:
 d=P/('ph_'+name);scene=tm.load(d/(name+'_1k.gltf'));parts=[]
 for index,node in enumerate(scene.graph.nodes_geometry):
  transform,key=scene.graph[node];m=scene.geometry[key].copy();m.apply_transform(rot@transform)
  material=m.visual.material.name
  if name.startswith('tree'):
   part='leaves' if 'leaves' in material else 'branches' if 'branches' in material else 'trunk'
   target={'leaves':90000,'branches':18000,'trunk':1000}[part]
   if len(m.faces)>target:
    old=m;merged=m.copy();merged.merge_vertices(merge_tex=True,merge_norm=True)
    if part=='leaves':
     components=tm.graph.connected_components(merged.face_adjacency,nodes=np.arange(len(merged.faces)),engine='scipy')
     keep=np.concatenate(components[::4]);merged=merged.submesh([keep],append=True);target=50000
    simple=merged.simplify_quadric_decimation(face_count=target);nearest=cKDTree(old.vertices).query(simple.vertices)[1];simple.visual=tm.visual.TextureVisuals(uv=old.visual.uv[nearest]);m=simple
   stem='tree_small_02_leaves' if part=='leaves' else 'tree_small_02_branch' if part=='branches' else 'tree_small_02'
   export(m,d/(part+'.obj'));parts.append((part,part=='trunk',stem+'_diff_1k.jpg',stem+'_nor_gl_1k.jpg'))
   print(part,len(m.faces),flush=True)
  else:
   if len(m.faces)>1500:
    old=m;m=old.simplify_quadric_decimation(face_count=1500);nearest=cKDTree(old.vertices).query(m.vertices)[1];m.visual=tm.visual.TextureVisuals(uv=old.visual.uv[nearest])
   part='rock_'+str(index);m.apply_translation([-m.bounds[:,0].mean(),-m.bounds[:,1].mean(),-m.bounds[0,2]]);export(m,d/(part+'.obj'));model(d,part,[(part,True,name+'_diff_1k.jpg',name+'_nor_gl_1k.jpg')]);print(part,len(m.faces),m.bounds.tolist(),flush=True)
 if parts:
  r=model(d,'model',parts);(d/'model.config').write_text('<model><name>Poly Haven Tree Small 02</name><version>1</version><sdf version="1.8">model.sdf</sdf></model>')
