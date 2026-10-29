"""ROS-independent control logic. SI units; no hardware-performance claims."""
from dataclasses import dataclass
import math
LEGS = ('fl', 'fr', 'rl', 'rr')
STAGES = tuple(f'tel{s}{i}_joint' for s in ('R', 'L') for i in range(1, 7))
ARMS = ('hexarmFR_joint', 'hexarmBR_joint', 'hexarmFL_joint', 'hexarmBL_joint')
D_MIN, D_MAX = -0.0379, 0.016511111111
def _profiles():
 from pathlib import Path
 from .parameters import resolve_profile
 share=Path(__file__).resolve().parents[1]
 if not (share/'config/parameter_schema.yaml').exists():
  from ament_index_python.packages import get_package_share_directory
  share=Path(get_package_share_directory('morph_sim'))
 return resolve_profile(share)[0]['control_modes']
PROFILES = _profiles()

def clamp(x, low, high): return max(low, min(high, x))
def approach(x, target, step): return x + clamp(target-x, -step, step)
def optimized_wheel(vx, vy, radius=.04):
 if math.hypot(vx, vy)<1e-8: return 0., 0.
 theta=math.atan2(vy,vx); speed=math.hypot(vx,vy)/radius
 if theta>math.pi/2: theta-=math.pi; speed=-speed
 if theta<-math.pi/2: theta+=math.pi; speed=-speed
 return theta,speed

def spring_force(q, v, k, c, limit=150.):
 return clamp(-k*q-c*v,-limit,limit)

@dataclass
class Output:
 wheels:list
 steering:list
 heights:list
 shape:list
 status:dict

class AdaptationCore:
 def __init__(self, radius=.075):
  if not .02<=radius<=.15: raise ValueError('wheel radius outside supported demonstration range')
  self.radius=radius; self.mode='track'; self.auto=False; self.estop=False
  self.d=0.; self.d_cmd=0.; self.target_d=0.; self.height_trim=0.; self.steering=[0.]*4; self.wheels=[0.]*4
  self.v=[0.,0.,0.]; self.roll=0.; self.pitch=0.; self.flip_phase=None;self.stop_hold=None
 def event(self, event):
  if event in PROFILES:
   self.mode=event; self.target_d=PROFILES[event]['extension']
  elif event=='compact': self.target_d=D_MIN
  elif event=='normal': self.target_d=0.
  elif event=='expand': self.target_d=D_MAX
  elif event=='raise': self.height_trim=clamp(self.height_trim+.003,-.01,.01)
  elif event=='lower': self.height_trim=clamp(self.height_trim-.003,-.01,.01)
  elif event=='auto': self.auto=not self.auto
  elif event=='flip' and max(abs(x) for x in self.wheels)<.5: self.flip_phase=-1
  elif event=='estop': self.estop=True
  elif event=='reset' and max(abs(x) for x in self.wheels)<.1: self.estop=False
 def step(self, dt, command, measured=None, underbody_hit=False, imu_fresh=False):
  dt=clamp(dt,0.,.05); measured=measured or {}
  if self.auto and imu_fresh and max(abs(self.roll),abs(self.pitch))>.25:
   self.mode='climb'; self.target_d=D_MAX
  p=PROFILES[self.mode]
  states_ready=all(n in measured and math.isfinite(measured[n]) for n in STAGES+ARMS+tuple(leg+suffix for leg in LEGS for suffix in ('_steer_joint','_height_joint','_wheel_joint')))
  if states_ready:self.d=sum(measured[n] for n in STAGES)/12
  # Coordinate the moving module attachment with chassis velocity.
  # Transformation is allowed at low speed, with steering alignment gating only the stages.
  transition_pending=states_ready and abs(self.target_d-self.d)>.0005
  if underbody_hit:self.estop=True
  if self.estop and self.stop_hold is None:
   self.stop_hold=([clamp(measured.get(n+'_height_joint',0.),-.015,.015) for n in LEGS],
    [clamp(measured.get(n,0.),D_MIN,D_MAX) for n in STAGES]+[clamp(measured.get(n,0.),-.25,.25) for n in ARMS])
  if not self.estop:self.stop_hold=None
  request=[clamp(command[0],-p['speed'],p['speed']),clamp(command[1],-p['speed'],p['speed']),clamp(command[2],-p['yaw'],p['yaw'])]
  if math.hypot(*request[:2])>p['speed']:
   f=p['speed']/math.hypot(*request[:2]); request[0]*=f; request[1]*=f
  if self.estop or not measured or self.flip_phase is not None : request=[0.,0.,0.]
  if transition_pending:
   request[0]=clamp(request[0],-.15,.15);request[1]=clamp(request[1],-.15,.15);request[2]=clamp(request[2],-.25,.25)
  for i in range(3):self.v[i]=approach(self.v[i],request[i],(1.2 if i<2 else 1.2)*dt)
  # Any movement sequence requires joint-state feedback. Stop if the caller marks it stale.
  target=clamp(self.target_d,D_MIN,D_MAX)
  morph_requested=states_ready and abs(target-self.d)>.0001 and math.hypot(*self.v[:2])<.22 and not self.estop and self.flip_phase is None
  dd=math.copysign(.0045,target-self.d) if morph_requested else 0.
  targets=[];speeds=[]
  for i,name in enumerate(LEGS):
   sx=1 if i<2 else -1; sy=1 if i%2==0 else -1
   x=sx*(.27+4*self.d);y=sy*.27
   theta,omega=optimized_wheel(self.v[0]-self.v[2]*y+sx*4*dd,self.v[1]+self.v[2]*x,self.radius)
   if abs(omega)<1e-8:theta=self.steering[i]
   if self.flip_phase is not None:theta=self.flip_phase*math.pi/2;omega=0.
   targets.append(theta);speeds.append(omega)
  aligned=all(abs(targets[i]-measured.get(LEGS[i]+'_steer_joint',self.steering[i]))<.10 for i in range(4))
  if morph_requested and not aligned:
   dd=0.
   # Remove stage feedforward while waiting, retain ordinary driving.
   speeds=[(math.cos(targets[i])*(self.v[0]-self.v[2]*(1 if i%2==0 else -1)*.27)+math.sin(targets[i])*(self.v[1]+self.v[2]*(1 if i<2 else -1)*(.27+4*self.d)))/self.radius for i in range(4)]
  if dd:
   # Bounded trajectory lead accommodates the position controller's lag.
   self.d_cmd=approach(self.d_cmd,target,abs(dd)*dt)
   self.d_cmd=clamp(self.d_cmd,self.d-.001,self.d+.001)
  else:self.d_cmd=self.d
  self.d_cmd=clamp(self.d_cmd,D_MIN,D_MAX)
  for i,n in enumerate(LEGS):
   self.steering[i]=approach(self.steering[i],targets[i],1.0*dt)
   error=abs(targets[i]-measured.get(n+'_steer_joint',self.steering[i]))
   desired=speeds[i]*max(0.,math.cos(error)) if not self.estop and states_ready else 0.
   self.wheels[i]=approach(self.wheels[i],desired,20.*dt)
  if self.flip_phase is not None and aligned:
   if self.flip_phase==-1:self.flip_phase=1
   else:self.flip_phase=None
  heights=[]
  for i in range(4):
   sx=1 if i<2 else -1;sy=1 if i%2==0 else -1
   correction=clamp(-self.roll*sy*.27+self.pitch*sx*(.27+4*self.d),-.006,.006) if self.auto and imu_fresh else 0.
   heights.append(clamp(p['height']+self.height_trim+correction,-.015,.015))
  if self.estop:heights=self.stop_hold[0].copy()
  arm=[]
  for sx,sy in [(1,1),(1,-1),(-1,1),(-1,-1)]:arm.append(math.atan2(sy*.379,-sx*2*self.d_cmd)-sy*math.pi/2)
  shape=[self.d_cmd]*12+arm
  if self.estop:shape=self.stop_hold[1].copy()
  return Output(self.wheels.copy(),self.steering.copy(),heights,shape,dict(mode=self.mode,auto=self.auto,estop=self.estop,transition_pending=transition_pending,morph_rate_m_s=dd,steering_alignment_rad=max(abs(targets[i]-measured.get(LEGS[i]+'_steer_joint',self.steering[i])) for i in range(4)),command_speed=math.hypot(*self.v[:2]),stage_offset=self.d,target_stage_offset=target,steering_degrees=[math.degrees(a) for a in self.steering],height=heights,stiffness=p['stiffness'],damping=p['damping'],feedback_ready=states_ready,flip_phase=self.flip_phase))
