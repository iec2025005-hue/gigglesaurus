"""Autonomous arena tour as a pure state machine (ROS-independent; nodes/autonomous_tour_node drives it).

One call per control tick: `step(state)` takes the measured state and returns the commands. The plan is
built from config/tetra_arena.json. Four policies share the same route and the same low-level driving;
they differ only in what the tour is allowed to do, so the ablation compares like with like:

    fixed_compact     stance locked at 237 mm; no suspension commands, no torque boost
    fixed_extended    stance locked at 672 mm; no suspension commands, no torque boost
    reactive          stops at each zone boundary, changes stance at rest (gated policy), then continues;
                      same suspension presets and torque boost as the proposed policy
    proposed          changes stance while driving, triggered t_morph + 1 s ahead; suspension presets,
                      torque boost on the ramp and trenches

The reactive and proposed policies have identical capabilities; they differ only in WHEN the stance changes
(at rest at the boundary, or on the move ahead of it). The fixed policies model a rover without the rails,
the corner actuators or the boost.

Pose comes from ground-truth odometry in simulation and the fixture positions from the arena file; the
LiDAR/camera pipeline reports distances alongside for comparison but does not steer the tour.
"""
import math
from . import decision_core as dc
from .trench_crossing import best_yaw, plan as step_plan

POLICIES = ('fixed_compact', 'fixed_extended', 'reactive', 'proposed')
ADAPTIVE = ('reactive', 'proposed')          # policies with rails, corner actuators and torque boost
CRUISE = {'flat': .5, 'trench': .3, 'soil': .18, 'ramp': .12, 'passage': .15, 'rough': .15}
STALL_SPEED, STALL_TIME = .01, 12.          # a leg fails after this long without progress
LEG_TIMEOUT = 150.
YAW_TOLERANCE = math.radians(2.)


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def build_legs(arena):
    """Route as a list of legs along +x. Each leg: zone id/kind, x range, and what the zone asks for."""
    legs = []
    for zone in arena['zones']:
        x0, x1 = zone['bounds'][0], zone['bounds'][1]; kind = zone['kind']; leg = {'id': zone['id'], 'kind': kind, 'x0': x0, 'x1': x1, 'zone': dict(zone), 'yaw': 0., 'speed': CRUISE[kind]}
        if kind == 'ramp':
            tour = next(r for r in zone['ramps'] if r['down_ramp']); leg.update(x0=tour['x_foot'] - .8, x1=tour['x_end'] + .6, slope_deg=tour['angle_deg'], x_top=tour['x_top'], x_down=tour['x_top'] + tour['platform_m'])
            leg['zone'].update(slope_deg=tour['angle_deg'], mu=.7225, label=f"ramp {tour['angle_deg']:.0f} deg")
        elif kind == 'trench':
            leg.update(x0=x0, x1=x1 - .2, trenches=[(t['x'], t['width_m']) for t in zone['trenches']]); leg['zone'].update(mu=.765, label='trenches ' + '/'.join(f"{t['width_m'] * 1e3:.0f}" for t in zone['trenches']) + ' mm')
        elif kind == 'passage':
            crab = zone.get('wall_height_m', 1.) > .041 and zone['clear_width_m'] < dc.BODY_WIDTH + 2 * dc.CLEARANCE_MARGIN
            leg.update(x0=x0 - .9, x1=x1 + .9, crab=crab); leg['zone'].update(mu=.765, label=f"{zone['id']} {zone['clear_width_m'] * 1e3:.0f} mm")
        elif kind == 'soil':
            leg['zone'].update(mu={'dry_sand': .35, 'mud': .25}.get(zone['surface'], .5), label=zone['id'])
        else:
            leg['zone'].update(mu=.765, label=zone['id'])
        legs.append(leg)
    legs[0]['x0'] = arena['spawn'][0]; legs[-1]['x1'] = arena['x_end'] - 1.
    return legs


class Tour:
    def __init__(self, arena, params, policy='proposed'):
        if policy not in POLICIES: raise ValueError('policy must be one of ' + ', '.join(POLICIES))
        self.policy = policy; self.legs = build_legs(arena); self.params = params; self.index = 0; self.phase = 'drive'
        self.events = []; self.log = []; self.results = []; self.finished = False
        self.leg_start = None; self.progress_x = None; self.progress_time = None; self.hold_until = 0.
        self.shape = {'fixed_compact': 'COMPACT', 'fixed_extended': 'EXTENDED'}.get(policy, 'NORMAL'); self.sent_shape = None
        self.suspension = 'suspension_level'; self.boost = False; self.step_state = {'event': 'suspension_level', 'current': None, 'done': set()}
        self.friction = None; self.explanation = ''; self.cost = dc.cost_vector(None); self.last_decision_leg = None
        self.shape_for = 0                      # index of the leg the commanded stance was chosen for

    # --- helpers -------------------------------------------------------------------------------
    def emit(self, name):
        self.events.append(name)

    def say(self, now, text):
        if text != self.explanation: self.explanation = text; self.log.append({'t': now, 'text': text})

    def context(self, state):
        return dict(mass_kg=self.params['robot']['mass_kg'], drive=self.params['drive'], current_d=state['d'], winding_C=state.get('winding_C', 22.),
                    torque_boost=self.policy in ADAPTIVE, soil_resistance_N=state.get('soil_resistance_N', 0.))

    def wanted_shape(self, leg, state):
        """Stance the policy wants for a leg, with the evaluation behind it."""
        best, results = dc.choose(leg['zone'], self.context(state))
        if self.policy in ('fixed_compact', 'fixed_extended'):
            fixed = next(r for r in results if r['shape'] == self.shape); return self.shape, fixed, results
        return (best['shape'] if best else self.shape), best, results

    @staticmethod
    def at_shape(state, shape):
        """The rails have actually reached this stance (the settled flag alone lags a new request by one tick)."""
        return state['settled'] and abs(dc.SHAPES[shape] - state['d']) < .0015

    def set_shape(self, shape):
        if shape != self.sent_shape:
            self.emit({'COMPACT': 'compact', 'NORMAL': 'normal', 'EXTENDED': 'expand'}[shape]); self.sent_shape = shape; self.shape = shape

    def set_suspension(self, name):
        if self.policy in ADAPTIVE and name != self.suspension: self.emit(name); self.suspension = name

    def set_boost(self, on):
        if self.policy in ADAPTIVE and on != self.boost: self.emit('boost_on' if on else 'boost_off'); self.boost = on

    def close_leg(self, now, state, success, reason):
        leg = self.legs[self.index]
        self.results.append({'zone': leg['id'], 'kind': leg['kind'], 'success': success, 'reason': reason, 'time_s': now - self.leg_start, 't_start': self.leg_start, 't_end': now, 'x_end': state['x']})
        self.index += 1; self.phase = 'drive'; self.leg_start = None; self.progress_x = None
        self.set_suspension('suspension_level'); self.set_boost(False); self.step_state = {'event': 'suspension_level', 'current': None, 'done': set()}
        if self.index >= len(self.legs): self.finished = True

    # --- one control tick ----------------------------------------------------------------------
    def step(self, now, state):
        """state: x, y, yaw, speed, d (stage offset), pitch_deg, settled (shape change complete), winding_C.
        Returns dict(command=(vx, vy, wz) body frame, events=[...], explanation, cost, leg, phase, friction, skip_to)."""
        self.events = []
        if self.sent_shape is None:
            self.emit('morph_continuous' if self.policy == 'proposed' else 'morph_gated'); self.set_shape(self.shape)
        if self.finished: return self.output((0., 0., 0.), None)
        leg = self.legs[self.index]
        if self.leg_start is None: self.leg_start = now; self.progress_x = state['x']; self.progress_time = now
        if state['x'] > self.progress_x + .02: self.progress_x = state['x']; self.progress_time = now
        command, skip = self.drive(now, leg, state)
        waiting = self.phase in ('morph_wait', 'turn', 'lift') or now < self.hold_until
        if waiting: self.progress_time = max(self.progress_time, now - STALL_TIME / 2)       # deliberate stops are not stalls
        if state['x'] >= leg['x1'] and self.phase in ('drive', 'exit'):
            self.close_leg(now, state, True, 'completed')
        elif now - self.progress_time > STALL_TIME or now - self.leg_start > LEG_TIMEOUT:
            reason = 'stalled' if now - self.progress_time > STALL_TIME else 'timeout'; skip = self.legs[self.index]['x1'] + .3
            self.say(now, f"[{leg['zone'].get('label', leg['id']).upper()}] FAILED ({reason}) at x = {state['x']:.2f} m in {self.shape}; repositioning past the zone")
            self.close_leg(now, state, False, reason); command = (0., 0., 0.)
        return self.output(command, skip)

    def output(self, command, skip):
        leg = self.legs[self.index] if not self.finished else None
        return {'command': command, 'events': list(self.events), 'explanation': self.explanation, 'cost': list(self.cost), 'leg': leg['id'] if leg else 'done',
                'phase': self.phase, 'friction': self.friction, 'skip_to': skip, 'finished': self.finished, 'shape': self.shape}

    def track(self, state, speed, heading, lane=0.):
        """World +x at `speed` with lateral and heading hold, expressed in the body frame."""
        yaw = state['yaw']; lateral = max(-.08, min(.08, -1.2 * (state['y'] - lane)))
        return (speed * math.cos(yaw) + lateral * math.sin(yaw), -speed * math.sin(yaw) + lateral * math.cos(yaw), max(-.5, min(.5, -1.8 * wrap(yaw - heading))))

    def drive(self, now, leg, state):
        kind = leg['kind']; speed = leg['speed']; heading = 0.; self.friction = leg['zone'].get('mu')
        shape, best, results = self.wanted_shape(leg, state)
        if self.last_decision_leg != leg['id']:
            self.last_decision_leg = leg['id']; self.cost = dc.cost_vector(best if best and best.get('feasible') else None)
            self.say(now, dc.explain(leg['zone'], best if best and best.get('feasible') else None, results if self.policy in ADAPTIVE else [best], speed=max(state['speed'], .05)))
        # Stance for THIS zone: the reactive policy stops and waits; the proposed one should already have it.
        # A stance already requested ahead for a later zone is left alone (it was checked to be passable here).
        if self.policy in ADAPTIVE and shape != self.sent_shape and self.phase == 'drive' and self.shape_for <= self.index:
            self.set_shape(shape); self.shape_for = self.index
            if self.policy == 'reactive' or (kind == 'passage' and leg.get('crab')): self.phase = 'morph_wait'
        if self.phase == 'morph_wait':
            if self.at_shape(state, self.sent_shape): self.phase = 'drive'
            else: return (0., 0., 0.), None
        # Predictive trigger for the NEXT zone (proposed policy only).
        if self.policy == 'proposed' and self.index + 1 < len(self.legs) and self.phase == 'drive':
            nxt = self.legs[self.index + 1]; next_shape, next_best, next_results = self.wanted_shape(nxt, state)
            usable_here = dc.evaluate(next_shape, leg['zone'], self.context(state))['feasible']       # never pre-morph into a stance that cannot finish this zone
            if next_shape != self.sent_shape and usable_here and not (kind in ('trench', 'ramp') or (kind == 'passage' and leg.get('crab'))):
                t_morph = dc.morph_time(state['d'], next_shape); distance = nxt['x0'] - state['x']
                if dc.lookahead_trigger(distance, state['speed'], t_morph):
                    self.set_shape(next_shape); self.shape_for = self.index + 1; self.cost = dc.cost_vector(next_best)
                    self.say(now, dc.explain(nxt['zone'], next_best, next_results, distance=distance, speed=max(state['speed'], .05)) + f" | time to zone {distance / max(state['speed'], .05):.1f} s <= t_morph {t_morph:.1f} s + 1.0 s: starting now")
        if kind == 'trench': return self.trench(now, leg, state)
        if kind == 'ramp': return self.ramp(now, leg, state)
        if kind == 'passage' and leg.get('crab'): return self.pinch(now, leg, state)
        if kind == 'rough': self.set_suspension('suspension_auto_level')
        return self.track(state, speed, heading), None

    def trench(self, now, leg, state):
        """Yawed crossing so the wheels meet each gap one at a time; the proposed policy also lifts each wheel."""
        yaw_target, stagger = best_yaw(dc.SHAPES[self.shape]); inside = leg['trenches'][0][0] - 1.0 <= state['x'] <= leg['trenches'][-1][0] + .9
        heading = yaw_target if inside else 0.
        if abs(wrap(state['yaw'] - heading)) > YAW_TOLERANCE * 2 and (state['x'] < leg['trenches'][0][0] - .75 or state['x'] > leg['trenches'][-1][0] + .6):
            self.phase = 'turn'; return (0., 0., max(-.5, min(.5, -1.8 * wrap(state['yaw'] - heading)))), None
        self.phase = 'drive'; self.set_boost(True); speed = leg['speed']
        if self.policy in ADAPTIVE:
            p = step_plan(state['x'], state['d'], state['yaw'], leg['trenches'], done=self.step_state['done']); wanted = 'lift_' + p['lift'] if p['lift'] else 'suspension_level'
            key = (p['lift'], p['trench']) if p['lift'] else None
            if key != self.step_state['current']:
                if self.step_state['current'] is not None: self.step_state['done'].add(self.step_state['current'])
                self.step_state['current'] = key
            if wanted != self.suspension:
                self.set_suspension(wanted)
                if p['lift']:
                    self.hold_until = now + 3.0
                    self.say(now, f"[TRENCH {leg['trenches'][p['trench']][1] * 1e3:.0f} mm] wheel {p['lift'].upper()} over the gap next: lifting it and leaning on the other three (stagger {stagger * 1e3:.0f} mm at yaw {math.degrees(yaw_target):.0f} deg)")
            speed = .1
            if now < self.hold_until: self.phase = 'lift'; return (0., 0., 0.), None
        return self.track(state, speed, heading), None

    def ramp(self, now, leg, state):
        climbing = leg['x0'] + .5 <= state['x'] <= leg['x_top'] + .2; descending = state['x'] >= leg['x_down'] - .1
        self.set_boost(climbing)
        if climbing and abs(state['pitch_deg']) > 8.:
            if self.suspension != 'pitch_nose_down':
                self.say(now, f"[RAMP {leg['slope_deg']:.0f} deg] IMU pitch {abs(state['pitch_deg']):.1f} deg > 8 deg -> front -15 mm / rear +15 mm, torque boost on; {self.shape} keeps {dc.axle_loads(dc.wheelbase(self.shape), leg['slope_deg'], self.params['robot']['mass_kg'])[0]:.0f} N on the front axle")
            self.set_suspension('pitch_nose_down')
        elif not climbing: self.set_suspension('suspension_level')
        return self.track(state, .10 if descending else leg['speed'], 0.), None

    def pinch(self, now, leg, state):
        """Compact first, then yaw 90 deg at rest, then crab sideways with the drive units tucked inward."""
        zone = leg['zone']; entry = zone['bounds'][0] - .55; exit_x = zone['bounds'][1] + .55
        fits = dc.clearance_cost(self.shape, zone)[0] < math.inf
        if state['x'] < entry - .02 and self.phase in ('drive',): return self.track(state, .25, 0.), None
        if not fits:                                              # a locked stance that cannot fit: drive up and fail honestly
            return self.track(state, leg['speed'], 0.), None
        if self.phase == 'drive' and state['x'] < exit_x:
            if not self.at_shape(state, self.shape): self.phase = 'morph_wait'; return (0., 0., 0.), None    # the shape change must finish before the crab starts
            self.phase = 'turn'; self.emit('crab_tuck_on')
            self.say(now, f"[PINCH ZONE {zone['clear_width_m'] * 1e3:.0f}mm] body {dc.BODY_WIDTH * 1e3:.0f} mm > gap {zone['clear_width_m'] * 1e3:.0f} mm -> {self.shape} is {dc.CRAB_LENGTH[self.shape] * 1e3:.0f} mm long: yawing 90 deg for a sideways crab-walk with {(zone['clear_width_m'] - dc.CRAB_LENGTH[self.shape]) / 2 * 1e3:.0f} mm per side")
        if self.phase == 'turn':
            error = wrap(state['yaw'] - math.pi / 2)
            if abs(error) > YAW_TOLERANCE or abs(state.get('yaw_rate', 0.)) > .05: return (0., 0., max(-.5, min(.5, -1.8 * error))), None
            self.phase = 'crab'
        if self.phase == 'crab':
            if state['x'] < exit_x: return self.track(state, leg['speed'], math.pi / 2), None
            self.phase = 'exit_turn'; self.emit('crab_tuck_off')
        if self.phase == 'exit_turn':
            error = wrap(state['yaw'])
            if abs(error) > YAW_TOLERANCE: return (0., 0., max(-.5, min(.5, -1.8 * error))), None
            self.phase = 'exit'
        return self.track(state, .25, 0.), None
