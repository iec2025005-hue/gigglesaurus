#!/usr/bin/env python3
"""Parametric assumed 150 mm swerve wheel module; dimensions in metres.
Run with trimesh + numpy. Visual meshes only; collision is a smooth tyre envelope.
"""
from pathlib import Path
import numpy as np
import trimesh as tm
P=Path(__file__).resolve().parents[1]/'meshes'
def save(name,parts):
 m=tm.util.concatenate(parts);m.export(P/(name+'.stl'));print(name,len(m.faces))
def cyl(r,h,xyz=(0,0,0),axis='z',sections=48):
 m=tm.creation.cylinder(r,h,sections=sections)
 if axis=='y':m.apply_transform(tm.transformations.rotation_matrix(np.pi/2,[1,0,0]))
 m.apply_translation(xyz);return m
def box(size,xyz):
 m=tm.creation.box(size);m.apply_translation(xyz);return m
# Closed rounded tyre profile revolved around the axle, then z axle -> y axle.
profile=np.array([[.034,-.020],[.058,-.0225],[.070,-.019],[.072,-.012],[.072,.012],[.070,.019],[.058,.0225],[.034,.020],[.034,-.020]])
tire=tm.creation.revolve(profile,sections=96);tire.apply_transform(tm.transformations.rotation_matrix(np.pi/2,[1,0,0]));parts=[tire]
for i in range(40):
 a=2*np.pi*i/40
 for sy in [-1,1]:
  m=box([.009,.021,.005],[0,sy*.011,.0725]);m.apply_transform(tm.transformations.rotation_matrix(sy*.35,[0,0,1],point=[0,sy*.011,.0725]));m.apply_transform(tm.transformations.rotation_matrix(a,[0,1,0]));parts.append(m)
save('research_tire',parts)
hubs=[];bolts=[]
for sy in [-1,1]:
 hubs+=[cyl(.033,.006,(0,sy*.023,0),'y'),cyl(.015,.010,(0,sy*.028,0),'y')]
 for j in range(6):
  a=j*np.pi/3;bolts.append(cyl(.0026,.003,(.024*np.cos(a),sy*.027,.024*np.sin(a)),'y',6))
save('research_hub',hubs);save('research_bolts',bolts)
# Complete axle fork, motor and encoder stay with the steering frame.
save('research_fork',[box([.038,.082,.012],[0,0,-.162]),box([.012,.010,.098],[0,-.036,-.211]),box([.012,.010,.098],[0,.036,-.211]),cyl(.011,.086,(0,0,-.26),'y')])
save('research_motor',[cyl(.020,.043,(0,-.063,-.26),'y'),cyl(.023,.012,(0,-.038,-.26),'y'),cyl(.022,.008,(0,0,-.14)),cyl(.018,.04,(0,0,-.118))])
save('research_encoder',[cyl(.015,.006,(0,-.0875,-.26),'y'),box([.012,.009,.009],[0,-.09,-.242])])
# Fixed damper barrel on height link, moving chrome shaft on suspension link.
save('research_damper',[cyl(.012,.075,(0,0,-.055)),cyl(.019,.008,(0,0,-.015)),cyl(.016,.008,(0,0,-.095))])
save('research_piston',[cyl(.005,.07,(0,0,-.105)),cyl(.019,.009,(0,0,-.14)),cyl(.012,.015,(0,0,-.153))])
# Helical coil is visual; the piston motion and spring force use the actual joint.
points=400;ring=8;v=[];f=[]
for i in range(points):
 a=i/(points-1)*2*np.pi*9;center=np.array([.016*np.cos(a),.016*np.sin(a),-.02-.11*i/(points-1)])
 for j in range(ring):
  b=j*2*np.pi/ring;v.append(center+.0018*np.array([np.cos(a)*np.cos(b),np.sin(a)*np.cos(b),np.sin(b)]))
for i in range(points-1):
 for j in range(ring):
  a=i*ring+j;b=i*ring+(j+1)%ring;c=a+ring;d=b+ring;f.extend([[a,b,c],[b,d,c]])
save('research_coil',[tm.Trimesh(v,f)])
