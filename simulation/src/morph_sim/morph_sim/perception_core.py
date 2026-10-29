"""Geometry-only local terrain assessment. No arena IDs or ground-truth poses."""
import math
import numpy as np

def assess(points, sensor_height=.43, roll=0., pitch=0.):
 p=np.asarray(points,dtype=float)
 if p.ndim!=2 or p.shape[1]!=3:return {'valid':False,'reason':'invalid point cloud'}
 p=p[np.isfinite(p).all(axis=1)]
 if not len(p):return {'valid':False,'reason':'no range returns'}
 # Level sensor axes using the measured IMU, retaining measured range geometry.
 cr,sr,cp,sp=math.cos(roll),math.sin(roll),math.cos(pitch),math.sin(pitch)
 level=np.array([[cp,sp*sr,sp*cr],[0,cr,-sr],[-sp,cp*sr,cp*cr]])
 p=p@level.T
 # Sensor is 0.22m ahead of the body origin. Ignore own-body returns.
 x=p[:,0]+.22;y=p[:,1];z=p[:,2];ground=-sensor_height
 roi=(x>.55)&(x<2.)&(np.abs(y)<.50)
 candidate=roi&(z>ground-.20)&(z<ground+.50)
 q=np.column_stack([x[candidate],y[candidate],z[candidate]])
 if len(q)<18:return {'valid':False,'reason':'insufficient ground observations','point_count':int(len(q))}
 a=np.column_stack([q[:,0],q[:,1],np.ones(len(q))]);mask=np.ones(len(q),dtype=bool)
 for _ in range(4):
  if mask.sum()<12:break
  fit=np.linalg.lstsq(a[mask],q[mask,2],rcond=None)[0]
  residual=q[:,2]-a@fit
  median=np.median(residual[mask]);mad=np.median(np.abs(residual[mask]-median))
  mask=np.abs(residual-median)<max(.025,2.8*mad)
 slope=math.degrees(math.atan(fit[0]));cross=math.degrees(math.atan(fit[1]))
 residual=q[:,2]-a@fit
 rough=float(np.percentile(residual,90)-np.percentile(residual,10))
 plane=fit[0]*x+fit[1]*y+fit[2]
 # Objects standing >0.18m above locally fitted ground block the chassis path.
 obstacles=(x>.55)&(x<2.)&(np.abs(y)<.46)&(z-plane>.18)&(z-plane<1.)
 nearest=float(np.min(x[obstacles])) if obstacles.sum()>=3 else None
 tilt=max(abs(math.degrees(roll)),abs(math.degrees(pitch)))
 if abs(slope)>8 or abs(cross)>8 or tilt>12:mode='climb';shape='expand';reason='observed slope / body tilt'
 elif rough>.035:mode='rocky';shape='compact';reason='observed uneven ground'
 else:mode='track';shape='normal';reason='observed smooth ground'
 speed=.3 if mode=='track' else .10
 if nearest is not None:speed=min(speed,max(0.,(nearest-.85)*.45))
 return {'valid':True,'reason':reason,'mode':mode,'shape':shape,'slope_deg':slope,'cross_slope_deg':cross,'roughness_m':rough,'nearest_obstacle_m':nearest,'speed_limit_m_s':speed,'point_count':int(len(q)),'ground_inliers':int(mask.sum()),'source':'range cloud + IMU; no sector map'}

class DecisionFilter:
 def __init__(self):self.candidate=None;self.count=0;self.mode='track'
 def update(self,observation):
  if not observation.get('valid'):return observation
  requested=observation['mode']
  if requested==self.candidate:self.count+=1
  else:self.candidate=requested;self.count=1
  required=10 if requested=='track' else 3
  if self.count>=required:self.mode=requested
  out=dict(observation);out['raw_mode']=requested;out['mode']=self.mode;out['shape']={'track':'normal','climb':'expand','rocky':'compact'}[self.mode];out['speed_limit_m_s']=min(out['speed_limit_m_s'],.3 if self.mode=='track' else .1);out['stable_scans']=self.count
  return out
