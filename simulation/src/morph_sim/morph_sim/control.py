"""ROS-independent control logic. SI units; no hardware-performance claims."""
from dataclasses import dataclass
import math
LEGS = ('fl', 'fr', 'rl', 'rr')
STAGES = tuple(f'tel{s}{i}_joint' for s in ('R', 'L') for i in range(1, 7))
ARMS = ('hexarmFR_joint', 'hexarmBR_joint', 'hexarmFL_joint', 'hexarmBL_joint')
D_MIN, D_MAX = -0.0379, 0.016511111111
MORPH_SPEED_GATE = .22       # m/s: stages may move only below this body speed (commanded AND measured)
MORPH_STAGE_RATE = .0045     # m/s per stage: commanded stage trajectory rate
TRANSITION_SPEED = .15       # m/s: body speed request while a shape change is pending
STAGE_RATE_FILTER_S = .05    # s: low-pass on the measured stage rate used for wheel feedforward
WHEEL_LOCK_GAIN = 4.         # 1/s: wheel-angle tracking gain while the stages move
WHEEL_LOCK_LIMIT = .6        # rad/s: bound on the wheel-angle tracking correction
MORPH_DECAY_S = .15          # s: feedforward envelope decay after the stage command ends
FLIP_MARGIN = .12            # rad: direction overshoot past +-90 deg tolerated before a 180 deg re-steer
ALIGNMENT_FLOOR = .05        # fraction of the request kept while the wheels re-steer, so the travel direction stays defined
TUCK_BAND = .35              # rad around +-90 deg in which the crab tuck picks the steering sign
SETTLE_IN, SETTLE_OUT = .0005, .0012   # m: shape-change completion band with hysteresis
STALL_RATE, STALL_TIME = 2e-4, .8      # m/s, s: stage at rest this long inside SETTLE_OUT counts as complete
FEEDFORWARD_FLOOR = .0012    # m/s per stage: commanded-direction intent while the stages are not yet moving
JAM_TIME = 3.                # s: commanded stages at rest this long outside SETTLE_OUT raise morph_stalled
CONTINUOUS_SPEED_GATE = .65  # m/s: 'continuous' morph policy lets the stages move at any reachable driving speed
HEIGHT_LIMIT = .015          # m: corner height actuator travel (+ raises the chassis at that corner)
SUSPENSION_PRESETS = {       # corner height targets [fl, fr, rl, rr] in m
 'pitch_nose_up':(HEIGHT_LIMIT,HEIGHT_LIMIT,-HEIGHT_LIMIT,-HEIGHT_LIMIT),     # front up, rear down: CoM projection moves rearward
 'pitch_nose_down':(-HEIGHT_LIMIT,-HEIGHT_LIMIT,HEIGHT_LIMIT,HEIGHT_LIMIT),   # front down, rear up: CoM projection moves forward
 # Single-wheel lift for stepping over a gap: that corner retracts, its two neighbours extend, and the opposite corner
 # stays at mid travel so the body leans towards it (about 4 deg: CoM about 20 mm inside the three-wheel support triangle).
 'lift_fl':(-HEIGHT_LIMIT,HEIGHT_LIMIT,HEIGHT_LIMIT,0.),'lift_fr':(HEIGHT_LIMIT,-HEIGHT_LIMIT,0.,HEIGHT_LIMIT),
 'lift_rl':(HEIGHT_LIMIT,0.,-HEIGHT_LIMIT,HEIGHT_LIMIT),'lift_rr':(0.,HEIGHT_LIMIT,HEIGHT_LIMIT,-HEIGHT_LIMIT)}

def wheel_position(i, d):
 """Wheel i contact point r_i(L) in the body frame; wheelbase L = 0.54 + 8 d, track fixed at 0.54 m."""
 return (1 if i<2 else -1)*(.27+4*d),(1 if i%2==0 else -1)*.27

def wheel_velocity(i, v, d, d_dot):
 """Coupled 4WIS + variable-wheelbase inverse kinematics for wheel i (fl, fr, rl, rr):

     v_wheel,i = v_body + omega x r_i(L) + (dr_i/dL) L_dot,   dr_i/dd = (+-4, 0),   L_dot = 8 d_dot

 v = (vx, vy, wz) in the body frame; d is the stage offset and d_dot its rate. The last term is what keeps the
 contact points on the ground stationary relative to the terrain while the rails move."""
 x,y=wheel_position(i,d)
 return v[0]-v[2]*y+(1 if i<2 else -1)*4*d_dot,v[1]+v[2]*x
def _profiles():
 from pathlib import Path
 from .parameters import resolve_profile
 share=Path(__file__).resolve().parents[1]
 if not (share/'config/parameter_schema.yaml').exists():
  from ament_index_python.packages import get_package_share_directory
  share=Path(get_package_share_directory('morph_sim'))
 return resolve_profile(share)[0]
_PARAMETERS = _profiles()
PROFILES = _PARAMETERS['control_modes']
MOTION = _PARAMETERS['motion']                                       # jerk and acceleration limits (V2 brief)
STEER_RATE = _PARAMETERS['steering']['output_speed_limit_rad_s']     # rad/s steering setpoint slew
GRAVITY = 9.81

def friction_acceleration_limit(mu):
 """Acceleration limit for a known surface friction: fraction * mu * g, bounded; default when mu is unknown."""
 if mu is None or not mu>0:return MOTION['acceleration_limit_m_s2']
 return clamp(MOTION['friction_acceleration_fraction']*mu*GRAVITY,MOTION['acceleration_floor_m_s2'],MOTION['acceleration_ceiling_m_s2'])

def jerk_limited_step(value, rate, target, dt, rate_limit, jerk_limit):
 """One step of an S-curve (jerk-limited) approach of `value` to `target`; value and rate are equal-length lists.

 The acceleration demand follows the discrete profile that lets it ramp down at the jerk limit and reach zero
 exactly at the target:  a = sqrt((J dt / 2)^2 + 2 J e) - J dt / 2,  capped at the acceleration limit."""
 error=[t-v for t,v in zip(target,value)];e=math.sqrt(sum(x*x for x in error))
 if dt<=0:return list(value),list(rate)
 # Discretisation margin: the final step of the sampled profile can exceed the design jerk by up to 12 %,
 # so the profile is designed at 0.88 J and the jerk of the OUTPUT stays within J.
 jerk_limit*=.88
 if e<1e-9:
  n=math.sqrt(sum(a*a for a in rate));keep=max(0.,n-jerk_limit*dt)/n if n>0 else 0.
  return list(target),[a*keep for a in rate]
 demand=min(rate_limit,math.sqrt((jerk_limit*dt/2)**2+2*jerk_limit*e)-jerk_limit*dt/2,e/dt)
 change=[x/e*demand-a for x,a in zip(error,rate)];n=math.sqrt(sum(c*c for c in change))
 if n>jerk_limit*dt:change=[c*jerk_limit*dt/n for c in change]
 rate=[a+c for a,c in zip(rate,change)];new=[v+a*dt for v,a in zip(value,rate)]
 if sum((t-v)*x for t,v,x in zip(target,new,error))<=0:return list(target),rate    # would pass the target this step
 return new,rate

def clamp(x, low, high): return max(low, min(high, x))
def approach(x, target, step): return x + clamp(target-x, -step, step)
def optimized_wheel(vx, vy, radius=.075, previous=None):
 """Steering angle in [-pi/2, pi/2] and signed wheel speed for a contact velocity.

 previous: current steering setpoint. A direction just past the +-90 deg joint limit would
 otherwise demand a 180 deg re-steer (e.g. crab travel with a small yaw or stage-motion
 component); within FLIP_MARGIN the wheel stays at the limit instead."""
 if math.hypot(vx, vy)<1e-8: return 0., 0.
 theta=math.atan2(vy,vx); speed=math.hypot(vx,vy)/radius
 if theta>math.pi/2: theta-=math.pi; speed=-speed
 if theta<-math.pi/2: theta+=math.pi; speed=-speed
 if previous is not None and abs(theta-previous)>math.pi/2:
  alternative=theta-math.copysign(math.pi,theta)
  if abs(alternative)<=math.pi/2+FLIP_MARGIN:return clamp(alternative,-math.pi/2,math.pi/2),-speed
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
  self.morph_policy='gated'; self.corner_target=None; self.torque_boost=False; self.command_synced=False
  self.a=[0.,0.,0.]; self.accel_limit=MOTION['acceleration_limit_m_s2']; self.auto_level=False; self.alignment=1.; self.crab_tuck=False
  self.stage_rate=0.; self.wheel_reference=[None]*4; self.morph_envelope=0.; self.settled_target=None; self.stall_time=0.; self.stalled_target=None; self.jam_time=0.; self.morph_fault=False
 def event(self, event):
  if event in PROFILES:
   self.mode=event; self.target_d=PROFILES[event]['extension']
  elif event=='compact': self.target_d=D_MIN
  elif event=='normal': self.target_d=0.
  elif event=='expand': self.target_d=D_MAX
  elif event in ('morph_gated','morph_continuous'): self.morph_policy=event[6:]
  elif event in SUSPENSION_PRESETS: self.corner_target=list(SUSPENSION_PRESETS[event])
  elif event=='suspension_level': self.corner_target=None; self.auto_level=False
  elif event=='suspension_auto_level': self.corner_target=None; self.auto_level=True
  elif event in ('boost_on','boost_off'): self.torque_boost=event=='boost_on'
  elif event in ('crab_tuck_on','crab_tuck_off'): self.crab_tuck=event=='crab_tuck_on'
  elif event=='raise': self.height_trim=clamp(self.height_trim+.003,-.01,.01)
  elif event=='lower': self.height_trim=clamp(self.height_trim-.003,-.01,.01)
  elif event=='auto': self.auto=not self.auto
  elif event=='flip' and max(abs(x) for x in self.wheels)<.5: self.flip_phase=-1
  elif event=='estop': self.estop=True
  elif event=='reset' and max(abs(x) for x in self.wheels)<.1: self.estop=False
 def step(self, dt, command, measured=None, underbody_hit=False, imu_fresh=False, velocities=None):
  """velocities: optional measured joint velocities. When supplied, the wheel feedforward for
  the moving module attachment uses the MEASURED stage rate (the position servo does not
  track the commanded rate), the morph speed gate also checks the encoder body-speed estimate,
  and wheel angles are tracked while the stages move so the body is not dragged."""
  dt=clamp(dt,0.,.05); measured=measured or {}; velocities=velocities or {}
  if self.auto and imu_fresh and max(abs(self.roll),abs(self.pitch))>.25:
   self.mode='climb'; self.target_d=D_MAX
  p=PROFILES[self.mode]
  states_ready=all(n in measured and math.isfinite(measured[n]) for n in STAGES+ARMS+tuple(leg+suffix for leg in LEGS for suffix in ('_steer_joint','_height_joint','_wheel_joint')))
  if states_ready:self.d=sum(measured[n] for n in STAGES)/12
  # The held length starts from the measurement (first feedback, after feedback loss, after an e-stop).
  if not states_ready or self.estop:self.command_synced=False
  elif not self.command_synced:self.d_cmd=self.d;self.command_synced=True
  # Coordinate the moving module attachment with chassis velocity.
  # Transformation is allowed at low speed, with steering alignment gating only the stages.
  # Completion latch with hysteresis: the position servo can rest a few tenths of a millimetre
  # short of the target, which previously toggled the pending flag (and the speed clamp).
  shape_error=abs(self.target_d-self.d)
  if self.settled_target!=self.target_d:self.settled_target=None
  # The stall latch stops the stage command near the target; it must release if the rails are pushed away again.
  if self.stalled_target!=self.target_d or shape_error>SETTLE_OUT:self.stalled_target=None
  if states_ready and shape_error<=SETTLE_IN:self.settled_target=self.target_d
  transition_pending=states_ready and shape_error>(SETTLE_OUT if self.settled_target is not None else SETTLE_IN)
  if underbody_hit:self.estop=True
  if self.estop and self.stop_hold is None:
   self.stop_hold=([clamp(measured.get(n+'_height_joint',0.),-.015,.015) for n in LEGS],
    [clamp(measured.get(n,0.),D_MIN,D_MAX) for n in STAGES]+[clamp(measured.get(n,0.),-.25,.25) for n in ARMS])
  if not self.estop:self.stop_hold=None
  request=[clamp(command[0],-p['speed'],p['speed']),clamp(command[1],-p['speed'],p['speed']),clamp(command[2],-p['yaw'],p['yaw'])]
  if math.hypot(*request[:2])>p['speed']:
   f=p['speed']/math.hypot(*request[:2]); request[0]*=f; request[1]*=f
  if self.estop or not measured or self.flip_phase is not None : request=[0.,0.,0.]
  continuous=self.morph_policy=='continuous';speed_gate=CONTINUOUS_SPEED_GATE if continuous else MORPH_SPEED_GATE
  if transition_pending and not continuous:
   # Bound the planar speed vector (a per-axis clamp let diagonal requests reach 0.21 m/s).
   planar=math.hypot(*request[:2])
   if planar>TRANSITION_SPEED:request[0]*=TRANSITION_SPEED/planar;request[1]*=TRANSITION_SPEED/planar
   request[2]=clamp(request[2],-.25,.25)
  applied_scale=1.
  if self.estop or not measured:
   # Emergency or no feedback: plain rate limit, not jerk limited (reported as jerk_limited False).
   for i in range(3):self.v[i]=approach(self.v[i],request[i],MOTION['acceleration_limit_m_s2']*dt)
   self.a=[0.,0.,0.]
  else:
   # Steering alignment acts on the REQUEST, ahead of the S-curve filter, so wheels that are still turning hold the
   # body back smoothly instead of snapping the drive in afterwards (which broke the jerk limit in crab and spin starts).
   applied_scale=max(ALIGNMENT_FLOOR,self.alignment);request=[r*applied_scale for r in request]
   # S-curve command filter: planar velocity as one vector (direction of travel is preserved), yaw separately.
   planar,rate=jerk_limited_step(self.v[:2],self.a[:2],request[:2],dt,self.accel_limit,MOTION['jerk_limit_m_s3'])
   yaw,yaw_rate=jerk_limited_step(self.v[2:],self.a[2:],request[2:],dt,MOTION['yaw_acceleration_limit_rad_s2'],MOTION['yaw_jerk_limit_rad_s3'])
   self.v=planar+yaw;self.a=rate+yaw_rate
  # Any movement sequence requires joint-state feedback. Stop if the caller marks it stale.
  target=clamp(self.target_d,D_MIN,D_MAX)
  # Encoder body-speed estimate: with wheels placed symmetrically about the body origin the
  # yaw and stage-motion terms cancel in the mean of the wheel contact velocities.
  wheel_rates=[velocities.get(n+'_wheel_joint') for n in LEGS]
  body_speed=None
  if states_ready and all(w is not None and math.isfinite(w) for w in wheel_rates):
   body_speed=math.hypot(sum(w*self.radius*math.cos(measured[n+'_steer_joint']) for w,n in zip(wheel_rates,LEGS))/4,sum(w*self.radius*math.sin(measured[n+'_steer_joint']) for w,n in zip(wheel_rates,LEGS))/4)
  morph_requested=states_ready and abs(target-self.d)>.0001 and self.stalled_target is None and math.hypot(*self.v[:2])<speed_gate and (body_speed is None or body_speed<speed_gate) and not self.estop and self.flip_phase is None
  dd=math.copysign(MORPH_STAGE_RATE,target-self.d) if morph_requested else 0.
  stage_rates=[velocities.get(n) for n in STAGES]
  measured_rate=states_ready and all(v is not None and math.isfinite(v) for v in stage_rates)
  if measured_rate:
   self.stage_rate+=(sum(stage_rates)/12-self.stage_rate)*(dt/(STAGE_RATE_FILTER_S+dt) if dt>0 else 0.)
   # Envelope: full while the stages are commanded, then a MORPH_DECAY_S ramp to zero, so stage
   # velocity noise cannot steer the wheels when no shape change is in progress.
   self.morph_envelope=1. if dd else max(0.,self.morph_envelope-dt/MORPH_DECAY_S)
   following=self.stage_rate*self.morph_envelope if abs(self.stage_rate)>1e-4 else 0.
   # Intent: while a shape change is commanded the wheels are steered and led in the commanded
   # direction even before the stages move (wheels pointing sideways would otherwise block the
   # stages for ever); the magnitude follows the measured rate once that exceeds the floor.
   feedforward=math.copysign(max(abs(following),FEEDFORWARD_FLOOR),dd) if dd and not self.morph_fault else following
   stalled=bool(dd) and abs(self.stage_rate)<STALL_RATE and shape_error<SETTLE_OUT
   self.stall_time=self.stall_time+dt if stalled else 0.
   if self.stall_time>STALL_TIME:self.settled_target=self.stalled_target=self.target_d
  else:
   self.stage_rate=0.;following=0.;feedforward=dd   # legacy path: no velocity feedback, assume the commanded rate
  targets=[];speeds=[]
  for i,name in enumerate(LEGS):
   sx=1 if i<2 else -1; sy=1 if i%2==0 else -1
   x=sx*(.27+4*self.d);y=sy*.27
   theta,omega=optimized_wheel(*wheel_velocity(i,self.v,self.d,feedforward),self.radius,None if feedforward else self.steering[i])
   if abs(omega)<1e-8:theta=self.steering[i]
   elif self.crab_tuck and abs(abs(theta)-math.pi/2)<TUCK_BAND and theta*sx>0:
    # Crab tuck: near +-90 deg both signs give the same wheel axis; take the one that points the drive unit inward
    # (front -90, rear +90), which is the only crab pose of the Compact chassis that is 330 mm long.
    theta=clamp(theta-math.copysign(math.pi,theta),-math.pi/2,math.pi/2);omega=-omega
   if self.flip_phase is not None:theta=self.flip_phase*math.pi/2;omega=0.
   targets.append(theta);speeds.append(omega)
  aligned=all(abs(targets[i]-measured.get(LEGS[i]+'_steer_joint',self.steering[i]))<.10 for i in range(4))
  self.alignment=min(max(0.,math.cos(targets[i]-measured.get(LEGS[i]+'_steer_joint',self.steering[i])))**2 for i in range(4)) if states_ready else 1.
  if morph_requested and not aligned:
   dd=0.
   # Remove the commanded stage feedforward while waiting, retain ordinary driving and whatever
   # motion the stages still really have (zero without velocity feedback).
   speeds=[(math.cos(targets[i])*(self.v[0]-self.v[2]*(1 if i%2==0 else -1)*.27+(1 if i<2 else -1)*4*following)+math.sin(targets[i])*(self.v[1]+self.v[2]*(1 if i<2 else -1)*(.27+4*self.d)))/self.radius for i in range(4)]
   feedforward=following
  # Jam detection: stages commanded but at rest well short of the target.
  jammed=measured_rate and bool(dd) and abs(self.stage_rate)<STALL_RATE and shape_error>=SETTLE_OUT
  self.jam_time=self.jam_time+dt if jammed else 0.
  if self.jam_time>JAM_TIME:self.morph_fault=True
  elif not dd or abs(self.stage_rate)>=STALL_RATE:self.morph_fault=False
  if dd:
   # Bounded trajectory lead accommodates the position controller's lag.
   self.d_cmd=approach(self.d_cmd,target,abs(dd)*dt)
  # Otherwise hold the last commanded length. Two earlier behaviours let the rails collapse under load: following
  # the measurement when idle (no holding force at all), and bounding the command to the MEAN stage position +-1 mm
  # (a stage that had not moved was then commanded inward after its neighbours slipped). The per-joint lead bound
  # in PositionTrajectoryLimiter already keeps each servo error small.
  self.d_cmd=clamp(self.d_cmd,D_MIN,D_MAX)
  for i,n in enumerate(LEGS):
   self.steering[i]=approach(self.steering[i],targets[i],STEER_RATE*dt)
   error=abs(targets[i]-measured.get(n+'_steer_joint',self.steering[i]))
   # Drive scaled by steering alignment: v_exec = v_cmd * cos^2(theta_err), zero beyond 90 degrees. The body request
   # has already been scaled by the worst wheel's factor (applied_scale), so each wheel gets the remaining ratio.
   desired=speeds[i]*min(1.,max(0.,math.cos(error))**2/applied_scale) if not self.estop and states_ready else 0.
   # While the stages move, track the wheel ANGLE implied by the commanded wheel speed. The
   # drive speed loop alone lags at these low speeds, and the stage actuators would then drag
   # the body instead of the wheels rolling under their modules.
   if measured_rate and feedforward and not self.estop:
    angle=measured[n+'_wheel_joint']
    if self.wheel_reference[i] is None:self.wheel_reference[i]=angle
    # The measurement belongs to the start of this period: compare before advancing.
    correction=clamp(WHEEL_LOCK_GAIN*(self.wheel_reference[i]-angle),-WHEEL_LOCK_LIMIT,WHEEL_LOCK_LIMIT)
    self.wheel_reference[i]+=desired*dt
    desired+=correction
   else:self.wheel_reference[i]=None
   self.wheels[i]=approach(self.wheels[i],desired,20.*dt)
  if self.flip_phase is not None and aligned:
   if self.flip_phase==-1:self.flip_phase=1
   else:self.flip_phase=None
  heights=[]
  for i in range(4):
   sx=1 if i<2 else -1;sy=1 if i%2==0 else -1
   level=-self.roll*sy*.27+self.pitch*sx*(.27+4*self.d)
   correction=clamp(level,-HEIGHT_LIMIT,HEIGHT_LIMIT) if self.auto_level and imu_fresh else clamp(level,-.006,.006) if self.auto and imu_fresh else 0.
   heights.append(clamp(self.corner_target[i] if self.corner_target is not None else p['height']+self.height_trim+correction,-HEIGHT_LIMIT,HEIGHT_LIMIT))
  if self.estop:heights=self.stop_hold[0].copy()
  arm=[]
  for sx,sy in [(1,1),(1,-1),(-1,1),(-1,-1)]:arm.append(math.atan2(sy*.379,-sx*2*self.d_cmd)-sy*math.pi/2)
  shape=[self.d_cmd]*12+arm
  if self.estop:shape=self.stop_hold[1].copy()
  return Output(self.wheels.copy(),self.steering.copy(),heights,shape,dict(mode=self.mode,auto=self.auto,estop=self.estop,transition_pending=transition_pending,morph_rate_m_s=dd,steering_alignment_rad=max(abs(targets[i]-measured.get(LEGS[i]+'_steer_joint',self.steering[i])) for i in range(4)),command_speed=math.hypot(*self.v[:2]),body_speed_estimate=body_speed,stage_rate_measured_m_s=(self.stage_rate if measured_rate else None),wheel_angle_lock=any(r is not None for r in self.wheel_reference),transition_settled=self.settled_target is not None,morph_stalled=self.morph_fault,jerk_limited=not (self.estop or not measured),command_accel_m_s2=math.hypot(*self.a[:2]),accel_limit_m_s2=self.accel_limit,auto_level=self.auto_level,alignment_scale=self.alignment,crab_tuck=self.crab_tuck,morph_policy=self.morph_policy,suspension_override=self.corner_target is not None,torque_boost=self.torque_boost,wheelbase_m=.54+8*self.d,stage_offset=self.d,target_stage_offset=target,steering_degrees=[math.degrees(a) for a in self.steering],height=heights,stiffness=p['stiffness'],damping=p['damping'],feedback_ready=states_ready,flip_phase=self.flip_phase))
