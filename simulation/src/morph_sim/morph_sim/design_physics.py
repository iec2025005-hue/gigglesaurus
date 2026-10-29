"""Deterministic SI design models. Assumed parameters are not calibration data.

Pacejka / Bekker functions are reference predictions, not Gazebo contact forces.
Native contact remains the single source of tangential ground force.
"""
import math
from pathlib import Path
import yaml


def clamp(x, low, high):
    return max(low, min(high, x))


def load_parameters(path):
    p = yaml.safe_load(Path(path).read_text())
    for section in ('robot', 'drive', 'steering', 'power', 'sensors', 'environment', 'surfaces'):
        if section not in p:
            raise ValueError('Missing parameter section: ' + section)
    for key in ('mass_kg', 'wheel_radius_m', 'wheel_width_m', 'wheelbase_m', 'track_m', 'com_height_m'):
        if not math.isfinite(p['robot'][key]) or p['robot'][key] <= 0:
            raise ValueError('Invalid robot parameter: ' + key)
    if p['provenance']['default_status'] != 'A':
        raise ValueError('This uncalibrated profile must retain assumed provenance')
    return p


def contact_patch(normal, radius, width, stiffness):
    delta = clamp(normal / stiffness, 0., radius * .5)
    length = 2 * math.sqrt(max(0., 2 * radius * delta - delta * delta))
    return length, normal / max(width * length, 1e-9), radius - delta


def traction_prediction(normal, omega, vx, vy, radius, surface, wetness=0.):
    """Bounded ellipse; no singularities at rest or when airborne."""
    if normal <= 0:
        return dict(slip_ratio=0., slip_angle=0., Fx_N=0., Fy_N=0.)
    sx = (omega * radius - vx) / max(abs(omega * radius), abs(vx), .02)
    alpha = math.atan2(vy, max(abs(vx), .02))
    b, c, e = (surface['pacejka_' + k] for k in ('B', 'C', 'E'))
    def force(slip, mu):
        bs = b * slip
        return mu * normal * math.sin(c * math.atan(bs - e * (bs - math.atan(bs))))
    mux = (1-wetness)*surface['mu_kinetic_x'] + wetness*surface['wet_mu_x']
    muy = (1-wetness)*surface['mu_kinetic_y'] + wetness*surface['wet_mu_y']
    fx, fy = force(sx, mux), -force(alpha, muy)
    ellipse = math.hypot(fx / max(mux*normal, 1e-9), fy / max(muy*normal, 1e-9))
    scale = max(1., ellipse)
    return dict(slip_ratio=sx, slip_angle=alpha, Fx_N=fx/scale, Fy_N=fy/scale)


def soft_ground_prediction(pressure, width, surface):
    if 'bekker' not in surface:
        return dict(sinkage_m=0., compaction_drag_N=0.)
    b = surface['bekker']; modulus = b['kc']/width + b['kphi']
    z = clamp((max(0., pressure)/modulus)**(1/b['n']), 0., b['max_sinkage_m'])
    return dict(sinkage_m=z, compaction_drag_N=width*modulus*z**(b['n']+1)/(b['n']+1))


def design_report(p):
    r, s, d, st = (p[k] for k in ('robot', 'suspension', 'drive', 'steering'))
    load = r['mass_kg'] * 9.81 / 4
    sprung = r['mass_kg']/4-r['unsprung_mass_per_module_kg']
    sag = max(0., (sprung*9.81-s['preload_N']) / s['stiffness_N_m'])
    fn = math.sqrt(s['stiffness_N_m']/sprung)/(2*math.pi)
    torque = min(d['output_torque_limit_Nm'], d['kt_Nm_A']*d['current_limit_A']*d['gear_ratio']*d['gear_efficiency'])
    return {'status':'A', 'mass_kg':r['mass_kg'], 'level_load_per_wheel_N':load,
            'spring_static_sag_m':sag, 'remaining_compression_m':s['upper_m']-sag,
            'sprung_frequency_Hz':fn, 'washboard_resonance_speed_m_s_at_0_3m':fn*.3,
            'output_torque_limit_Nm':torque, 'ideal_total_drive_force_N':4*torque/r['wheel_radius_m'],
            'side_tip_angle_deg':math.degrees(math.atan(r['track_m']/(2*r['com_height_m']))),
            'scrub_Nm_at_mu_0_8':2/3*.8*load*st['effective_patch_radius_m'],
            'limitations':'Static estimates; actual CoM changes with suspension and morphology; no hardware measurements.'}


class PowerPlant:
    """Current/voltage-limited DC motors, shared battery, lumped thermal feedback.

    Torque is returned in output_shaft frame. No regenerative charge is claimed.
    The caller must use an effort controller, never a parallel ideal velocity servo.
    """
    def __init__(self, p, ambient=22.):
        self.p = p; self.ambient = ambient
        self.temp = [ambient]*4; self.driver_temp = [ambient]*4
        self.current = [0.]*4; self.integral = [0.]*4; self.previous_targets = [0.]*4
        self.soc = p['power']['soc_initial']; self.battery_temp = ambient
        self.voltage = p['power']['ocv_full_V']; self.cutoff = [False]*4
        self.brownout = False; self.needs_home = False; self.resets = 0

    def home(self, stopped):
        if stopped and not self.brownout:
            self.needs_home = False
            return True
        return False

    def step(self, dt, targets, velocities, enabled=True):
        if not 0 < dt <= .05 or len(targets)!=4 or len(velocities)!=4:
            raise ValueError('Plant expects four wheels and dt in (0,.05]')
        if not all(math.isfinite(v) for v in targets+velocities):
            raise ValueError('Non-finite motor state')
        d, p = self.p['drive'], self.p['power']
        cold = 1 + clamp((22-self.ambient)/32, 0, 1)*(p['cold_resistance_multiplier_at_minus10C']-1)
        resistance = p['internal_resistance_ohm']*cold + p['wire_resistance_ohm']
        ocv = p['ocv_empty_V']+(p['ocv_full_V']-p['ocv_empty_V'])*self.soc
        duties=[]; currents=[]; torques=[]
        for i,(target,omega) in enumerate(zip(targets,velocities)):
            if self.temp[i]>=d['cutoff_C'] or self.driver_temp[i]>=d['cutoff_C']: self.cutoff[i]=True
            if max(self.temp[i],self.driver_temp[i])<d['recover_C']: self.cutoff[i]=False
            active=enabled and not self.cutoff[i] and not self.brownout and not self.needs_home and self.soc>0
            r=d['resistance_ohm']*(1+d['copper_alpha_per_K']*(self.temp[i]-20))
            kt=d['kt_Nm_A']*max(.5,1+d['magnet_alpha_per_K']*(self.temp[i]-20))
            error=target-omega
            available=max(0.,self.voltage-d['bridge_drop_V'])
            feedforward=d['ke_Vs_rad']*target*d['gear_ratio']
            # Acceleration feedforward supplies the force needed to accelerate the
            # assumed vehicle mass; torque/current/voltage limits remain active.
            alpha=clamp((target-self.previous_targets[i])/dt,-20.,20.)
            wheel_radius=self.p['robot']['wheel_radius_m']
            inertia=(self.p['robot']['mass_kg']/4+.5*self.p['robot']['wheel_mass_kg'])*wheel_radius**2
            acceleration_current=inertia*alpha/max(kt*d['gear_ratio']*d['gear_efficiency'],1e-9)
            if d.get('acceleration_feedforward',True):feedforward+=r*acceleration_current
            self.previous_targets[i]=target if active else omega
            raw=feedforward+d['speed_kp_Vs_rad']*error+self.integral[i]
            voltage=clamp(raw,-available,available) if active else 0.
            if active and (abs(raw)<available or raw*error<0):
                self.integral[i]=clamp(self.integral[i]+d['speed_ki_V_rad']*error*dt,-available,available)
            if not active:self.integral[i]=0.
            steady=(voltage-d['ke_Vs_rad']*omega*d['gear_ratio'])/r if active else 0.
            current=steady+(self.current[i]-steady)*math.exp(-r*dt/d['inductance_H'])
            derate=clamp((d['cutoff_C']-max(self.temp[i],self.driver_temp[i]))/(d['cutoff_C']-d['derate_C']),0.,1.)
            current=clamp(current,-d['current_limit_A']*derate,d['current_limit_A']*derate) if active else 0.
            duties.append(voltage/max(available,1e-9)); currents.append(current)
            torques.append(clamp(kt*current*d['gear_ratio']*d['gear_efficiency'],-d['output_torque_limit_Nm'],d['output_torque_limit_Nm']))
        # Conservative battery budget includes PWM duty, and electronics.
        electronics=p['electronics_W']/max(self.voltage*p['buck_efficiency'],1.)
        requested=sum(abs(i*u) for i,u in zip(currents,duties))
        budget=max(0.,p['bms_limit_A']-electronics)
        scale=min(1.,budget/max(requested,1e-9))
        currents=[i*scale for i in currents];torques=[t*scale for t in torques]
        total=electronics+requested*scale
        self.voltage=max(0.,ocv-total*resistance)
        rail5=min(5.,max(0.,self.voltage-p['buck_dropout_V']))
        if rail5<p['brownout_5V']:
            if not self.brownout:self.resets+=1
            self.brownout=True;self.needs_home=True
        elif rail5>p['recover_5V']:self.brownout=False
        if self.brownout or self.needs_home:torques=[0.]*4
        for i,current in enumerate(currents):
            r=d['resistance_ohm']*(1+d['copper_alpha_per_K']*(self.temp[i]-20))
            self.temp[i]+=dt*(current*current*r-(self.temp[i]-self.ambient)/d['thermal_resistance_K_W'])/d['thermal_capacity_J_K']
            self.driver_temp[i]+=dt*(abs(current)*d['bridge_drop_V']-(self.driver_temp[i]-self.ambient)/d['driver_thermal_resistance_K_W'])/d['driver_capacity_J_K']
        self.battery_temp+=dt*(total*total*resistance-(self.battery_temp-self.ambient)/p['battery_thermal_resistance_K_W'])/p['battery_capacity_J_K']
        self.soc=max(0.,self.soc-total*dt/(3600*p['capacity_Ah']))
        self.current=currents
        return {'torques_Nm':torques,'motor_current_A':currents,'duty':duties,'winding_C':self.temp.copy(),'driver_C':self.driver_temp.copy(),'battery_C':self.battery_temp,'battery_V':self.voltage,'rail_5V':rail5,'rail_3V3':min(3.3,max(0.,rail5-.3)),'battery_current_A':total,'soc':self.soc,'brownout':self.brownout,'needs_home':self.needs_home,'reset_count':self.resets}
