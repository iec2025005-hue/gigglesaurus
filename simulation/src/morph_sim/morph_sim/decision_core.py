"""Decision core for the V2 rover: cost terms, shape choice, predictive trigger and explanations.

ROS-independent. Every quantity is either geometry from the URDF audit, a parameter from
config/params.yaml, or an input measured at run time. Nothing here is tuned to a result.

Shape choice minimises, over the three stances S in {COMPACT, NORMAL, EXTENDED},

    J(S) = w_stab J_stab + w_therm J_therm + w_slip J_slip + w_clear J_clear + w_time J_time

    J_stab  = MARGIN_REF / margin(S)        margin = smallest static tip-over angle [deg] on the slope ahead
    J_therm = dT_ss(S) / headroom           steady winding temperature rise for the force the terrain needs
    J_slip  = max axle traction utilisation required traction / (mu * axle normal load)
    J_clear = CLEARANCE_REF / spare margin when the stance fits the aperture / trench, +inf if it cannot pass
    J_time  = t_morph(current -> S) / T_FULL

A stance with any infinite term is infeasible. The predictive trigger starts a shape change when the
time to the next zone, d / v, has shrunk to t_morph + margin.
"""
import math

G = 9.81
COM_HEIGHT = .291                 # m, from URDF inertials (Phase 0 audit)
TRACK = .54                       # m, fixed
WHEEL_RADIUS = .075
SHAPES = {'COMPACT': -.0379, 'NORMAL': 0., 'EXTENDED': .016511111111}
STAGE_RATE = .0032                # m/s per stage, MEASURED rail rate (commanded 0.0045)
T_FULL = (SHAPES['EXTENDED'] - SHAPES['COMPACT']) / STAGE_RATE
MARGIN_REF = 20.                  # deg: a margin of 20 deg costs 1
# Envelope of the trimmed chassis [m] (tests/test_v2_model_and_arena.py checks these against the URDF)
TYRE_WIDTH = .586                 # over both tyres; the only part below 42 mm
BODY_WIDTH = .701                 # drive units and frame, from 42 mm above ground
CRAB_LENGTH = {'COMPACT': .330, 'NORMAL': .690, 'EXTENDED': .822}       # crab pose, drive units inward
STEP_STAGGER = {'COMPACT': .178, 'NORMAL': .241, 'EXTENDED': .284}      # best one-wheel-at-a-time spacing
STEP_WINDOW_MARGIN = .060         # lead + trail around a trench for the lifted wheel
CLEARANCE_MARGIN = .020           # m per side counted as "fits"
CLEARANCE_REF = .020              # m: a spare margin of 20 mm costs 1 (capped at 5)


def _margin_cost(spare):
    return min(5., CLEARANCE_REF / spare) if spare > 0 else math.inf


def wheelbase(shape_or_d):
    d = SHAPES[shape_or_d] if isinstance(shape_or_d, str) else shape_or_d
    return .54 + 8 * d


def gravity_in_body(roll, pitch):
    """Unit gravity vector in the body frame; ROS convention, positive pitch is nose down."""
    return math.sin(pitch), -math.sin(roll) * math.cos(pitch), -math.cos(roll) * math.cos(pitch)


def stability_margins(length, roll=0., pitch=0., height=COM_HEIGHT, track=TRACK, com_x=0., com_y=0.):
    """Static tip-over angle margins [deg] about the four support edges (gravity only).

    For each edge the margin is the angle the gravity vector can still rotate towards that edge before it
    passes through it: atan(distance from CoM projection to the edge / CoM height) minus the current lean.
    """
    gx, gy, gz = gravity_in_body(roll, pitch)
    lean_x = math.atan2(gx, -gz); lean_y = math.atan2(gy, -gz)
    return {'front': math.degrees(math.atan((length / 2 - com_x) / height) - lean_x), 'rear': math.degrees(math.atan((length / 2 + com_x) / height) + lean_x),
            'left': math.degrees(math.atan((track / 2 - com_y) / height) - lean_y), 'right': math.degrees(math.atan((track / 2 + com_y) / height) + lean_y)}


def stability_margin(length, roll=0., pitch=0., **kwargs):
    return min(stability_margins(length, roll, pitch, **kwargs).values())


def axle_loads(length, slope_deg, mass, height=COM_HEIGHT, drive_force=0.):
    """Front and rear axle normal loads [N] climbing a slope (front uphill). Resistance at ground level adds no couple;
    the grade force acts at CoM height."""
    theta = math.radians(slope_deg); weight = mass * G
    shift = (weight * math.sin(theta) * height) / length          # load moved from the front to the rear axle
    half = weight * math.cos(theta) / 2
    return max(0., half - shift), half + shift


def required_force(slope_deg, mass, resistance=0.):
    return mass * G * math.sin(math.radians(slope_deg)) + resistance


def traction_utilisation(length, slope_deg, mass, mu, torque_limit, resistance=0., radius=WHEEL_RADIUS):
    """How hard the worst axle works: required traction / available. Above 1 the stance cannot hold the grade.

    Each axle can deliver min(friction, torque) = min(mu N_axle, 2 tau / r). The requirement is shared in proportion
    to what each axle can give; utilisation is requirement over the total available."""
    front, rear = axle_loads(length, slope_deg, mass)
    available = sum(min(mu * n, 2 * torque_limit / radius) for n in (front, rear))
    need = required_force(slope_deg, mass, resistance)
    return need / available if available > 0 else math.inf


def winding_rise(force, drive, radius=WHEEL_RADIUS):
    """Steady winding temperature rise [K] when the four wheels share `force` equally."""
    torque = abs(force) * radius / 4
    current = torque / (drive['kt_Nm_A'] * drive['gear_ratio'] * drive['gear_efficiency'])
    return current * current * drive['resistance_ohm'] * drive['thermal_resistance_K_W']


def morph_time(from_d, to_shape, rate=STAGE_RATE):
    return abs(SHAPES[to_shape] - from_d) / rate


def lookahead_trigger(distance, speed, t_morph, margin=1., minimum_speed=.02):
    """True when the shape change must start now to finish `margin` seconds before the zone is reached."""
    if t_morph <= 0: return False
    return distance / max(abs(speed), minimum_speed) <= t_morph + margin


def trigger_distance(speed, t_morph, margin=1.):
    """Distance before the zone at which the trigger fires, d = v (t_morph + margin)."""
    return abs(speed) * (t_morph + margin)


def clearance_cost(shape, zone):
    """0 when the stance can pass the zone's fixture, inf when it cannot. Returns (cost, reason)."""
    kind = zone.get('kind')
    if kind == 'passage':
        width = zone['clear_width_m']
        if zone.get('wall_height_m', 1.) <= .041:                 # kerbs: only the tyres pass between them
            margin = (width - TYRE_WIDTH) / 2
            return (_margin_cost(margin), f'tyres {TYRE_WIDTH * 1e3:.0f} mm fit {width * 1e3:.0f} mm kerbs with {margin * 1e3:.0f} mm per side') if margin > 0 else (math.inf, 'tyres wider than the kerb gap')
        if width - BODY_WIDTH >= 2 * CLEARANCE_MARGIN: return _margin_cost((width - BODY_WIDTH) / 2), f'body {BODY_WIDTH * 1e3:.0f} mm fits head-on'
        margin = (width - CRAB_LENGTH[shape]) / 2
        if margin >= CLEARANCE_MARGIN: return _margin_cost(margin), f'{shape} is {CRAB_LENGTH[shape] * 1e3:.0f} mm long: fits sideways with {margin * 1e3:.0f} mm per side'
        return math.inf, f'{shape} is {CRAB_LENGTH[shape] * 1e3:.0f} mm long: does not fit {width * 1e3:.0f} mm even sideways'
    if kind == 'trench':
        widest = max(t['width_m'] for t in zone['trenches']); spare = STEP_STAGGER[shape] - (widest + STEP_WINDOW_MARGIN)
        if spare > 0: return _margin_cost(spare), f'{shape} wheel stagger {STEP_STAGGER[shape] * 1e3:.0f} mm exceeds the {widest * 1e3:.0f} mm trench window by {spare * 1e3:.0f} mm'
        return math.inf, f'{shape} wheel stagger {STEP_STAGGER[shape] * 1e3:.0f} mm is too short for a {widest * 1e3:.0f} mm trench'
    return 0., 'no geometric constraint'


def evaluate(shape, zone, context, weights=None):
    """Cost terms of one stance for the zone ahead.

    zone: dict from config/tetra_arena.json (kind, slope_deg, clear_width_m, trenches, mu ...).
    context: mass_kg, drive (params['drive']), current_d, winding_C, torque_boost (bool), soil_resistance_N.
    """
    w = dict(stability=1., thermal=1., slip=1., clearance=1., time=.2); w.update(weights or {})
    length = wheelbase(shape); slope = zone.get('slope_deg', 0.); mass = context['mass_kg']; drive = context['drive']
    mu = zone.get('mu', .7); resistance = context.get('soil_resistance_N', 0.) + context.get('rolling_resistance_N', 2.2)
    torque = drive['output_torque_limit_Nm'] * (1 + drive.get('peak_boost_fraction', 0.) if context.get('torque_boost') else 1.)
    margin = stability_margin(length, 0., -math.radians(slope))    # climbing: nose up
    force = required_force(slope, mass, resistance)
    headroom = max(1., drive['derate_C'] - context.get('winding_C', 22.))
    clear, why = clearance_cost(shape, zone)
    utilisation = traction_utilisation(length, slope, mass, mu, torque, resistance)
    terms = {'stability': MARGIN_REF / margin if margin > 0 else math.inf, 'thermal': winding_rise(force, drive) / headroom,
             'slip': utilisation if utilisation <= 1. else math.inf, 'clearance': clear, 'time': morph_time(context.get('current_d', 0.), shape) / T_FULL}
    total = sum(w[k] * v for k, v in terms.items())
    return {'shape': shape, 'total': total, 'terms': terms, 'feasible': math.isfinite(total), 'stability_margin_deg': margin,
            'traction_utilisation': utilisation, 'required_force_N': force, 'clearance_reason': why, 'morph_time_s': morph_time(context.get('current_d', 0.), shape)}


def choose(zone, context, weights=None, preferred=('NORMAL', 'EXTENDED', 'COMPACT')):
    """Lowest-cost feasible stance; ties go to `preferred` order. Returns (best or None, all evaluations)."""
    results = [evaluate(shape, zone, context, weights) for shape in preferred]
    feasible = [r for r in results if r['feasible']]
    return (min(feasible, key=lambda r: r['total']) if feasible else None), results


def explain(zone, best, results, distance=None, speed=None):
    """One human-readable line with the numbers that decided it."""
    label = zone.get('label') or zone.get('id', 'zone'); head = f"[{label.upper()}" + (f" {distance:.1f} m AHEAD]" if distance is not None else ']')
    if best is None:
        def why(r): return ', '.join(k for k, v in r['terms'].items() if not math.isfinite(v)) + (f" ({r['clearance_reason']})" if not math.isfinite(r['terms']['clearance']) else '')
        if len(results) == 1: return head + f" fixed stance {results[0]['shape']} cannot pass: " + why(results[0])
        return head + ' no stance can pass: ' + '; '.join(f"{r['shape']}: " + why(r) for r in results)
    rejected = [r for r in results if not r['feasible']]
    parts = [f"{best['shape']} ({wheelbase(best['shape']) * 1e3:.0f} mm)", f"tip margin {best['stability_margin_deg']:.0f} deg", f"traction use {best['traction_utilisation'] * 100:.0f} %"]
    if zone.get('kind') in ('passage', 'trench'): parts.append(best['clearance_reason'])
    if rejected: parts.append('rejected ' + ', '.join(f"{r['shape']} ({next(k for k, v in r['terms'].items() if not math.isfinite(v))})" for r in rejected))
    if best['morph_time_s'] > .5:
        parts.append(f"shape change {best['morph_time_s']:.1f} s" + (f", start within {trigger_distance(speed, best['morph_time_s']):.1f} m at {speed:.2f} m/s" if speed else ''))
    return head + ' -> ' + '; '.join(parts)


def cost_vector(best):
    """Flat list for /morph/cost_breakdown: [total, stability, thermal, slip, clearance, time, margin_deg, traction_use]."""
    if best is None: return [math.inf] * 6 + [0., math.inf]
    t = best['terms']
    return [best['total'], t['stability'], t['thermal'], t['slip'], t['clearance'], t['time'], best['stability_margin_deg'], best['traction_utilisation']]


COST_FIELDS = ['total', 'stability', 'thermal', 'slip', 'clearance', 'time', 'stability_margin_deg', 'traction_utilisation']
