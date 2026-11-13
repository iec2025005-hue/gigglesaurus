#!/usr/bin/env python3
"""V2 Phase-0 feasibility calculations: geometry, CoM and arena-spec checks.

    python3 src/morph_sim/tools/phase0_feasibility.py [OUT.json]

Everything is computed from urdf/morph.urdf.xacro, the meshes and config/params.yaml, so the
audit numbers are reproducible. Static rigid-body estimates of an ASSUMED-parameter model:
suspension compliance, tyre deformation and dynamics are not included. Measured values
(speed, morph time) are passed in from reports/baseline_data when available.
"""
import json, math, struct, sys
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import xacro
import yaml

PKG = Path(__file__).resolve().parents[1]
G = 9.81
SHAPES = {'COMPACT': -0.0379, 'NORMAL': 0.0, 'EXTENDED': 0.016511111111}


def transform(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    r, p, y = rpy
    rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    m = np.eye(4); m[:3, :3] = rz @ ry @ rx; m[:3, 3] = xyz
    return m


def origin(element):
    o = element.find('origin') if element is not None else None
    if o is None: return np.eye(4)
    return transform([float(v) for v in o.get('xyz', '0 0 0').split()], [float(v) for v in o.get('rpy', '0 0 0').split()])


def stl_vertices(path):
    with open(path, 'rb') as f:
        f.read(80); n = struct.unpack('<I', f.read(4))[0]
        a = np.frombuffer(f.read(n * 50), dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))
    return a['v'].reshape(-1, 3)


class Robot:
    def __init__(self, params):
        doc = xacro.process_file(str(PKG / 'urdf/morph.urdf.xacro'), mappings={
            'controllers_file': 'controllers.yaml', 'wheel_radius': str(params['robot']['wheel_radius_m']),
            'wheel_width': str(params['robot']['wheel_width_m'])})
        self.root = ET.fromstring(doc.toxml())
        self.links = {l.get('name'): l for l in self.root.findall('link')}
        self.children = {}
        for j in self.root.findall('joint'):
            self.children.setdefault(j.find('parent').get('link'), []).append(j)
        self.mesh_cache = {}

    def poses(self, d, steer):
        q = {f'tel{s}{i}_joint': d for s in 'RL' for i in range(1, 7)}
        q.update({n + '_steer_joint': steer for n in ('fl', 'fr', 'rl', 'rr')})
        pose = {'base_link': np.eye(4)}; stack = ['base_link']
        while stack:
            parent = stack.pop()
            for j in self.children.get(parent, []):
                m = pose[parent] @ origin(j); axis_el = j.find('axis')
                axis = np.array([float(v) for v in axis_el.get('xyz').split()]) if axis_el is not None else np.array([1., 0, 0])
                value = q.get(j.get('name'), 0.)
                if j.get('type') == 'prismatic': m = m @ transform(axis * value)
                elif j.get('type') in ('revolute', 'continuous'):
                    k = axis / np.linalg.norm(axis); kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
                    a = np.eye(4); a[:3, :3] = np.eye(3) + math.sin(value) * kx + (1 - math.cos(value)) * kx @ kx; m = m @ a
                child = j.find('child').get('link'); pose[child] = m; stack.append(child)
        return pose

    def analyse(self, d, steer=0.):
        pose = self.poses(d, steer); mass = 0.; com = np.zeros(3)
        for name, link in self.links.items():
            inertial = link.find('inertial'); m = float(inertial.find('mass').get('value'))
            mass += m; com += m * (pose[name] @ origin(inertial))[:3, 3]
        com /= mass
        collision = []; visual = []
        for name, link in self.links.items():
            for c in link.findall('collision'):
                m = pose[name] @ origin(c); g = c.find('geometry')
                if g.find('box') is not None:
                    s = [float(v) / 2 for v in g.find('box').get('size').split()]
                    local = [[a * s[0], b * s[1], e * s[2], 1] for a in (-1, 1) for b in (-1, 1) for e in (-1, 1)]
                else:
                    r = float(g.find('cylinder').get('radius')); h = float(g.find('cylinder').get('length')) / 2
                    local = [[r * math.cos(t), r * math.sin(t), z, 1] for t in np.linspace(0, 2 * math.pi, 145) for z in (-h, h)]
                collision += [(m @ np.array(p))[:3] for p in local]
            for v in link.findall('visual'):
                mesh = v.find('geometry').find('mesh')
                if mesh is None: continue
                file = mesh.get('filename').split('/')[-1]
                if file not in self.mesh_cache: self.mesh_cache[file] = stl_vertices(PKG / 'meshes' / file)
                scale = [float(x) for x in mesh.get('scale', '1 1 1').split()]
                m = pose[name] @ origin(v); pts = self.mesh_cache[file] * scale
                visual.append((m[:3, :3] @ pts.T).T + m[:3, 3])
        collision = np.array(collision); visual = np.vstack(visual)
        radius = float(self.links['fl_wheel'].find('collision/geometry/cylinder').get('radius'))
        ground = pose['fl_wheel'][2, 3] - radius
        wheelbase = pose['fl_wheel'][0, 3] - pose['rl_wheel'][0, 3]; track = pose['fl_wheel'][1, 3] - pose['fr_wheel'][1, 3]

        def band(points, lo, hi, axis):
            sel = points[(points[:, 2] - ground >= lo) & (points[:, 2] - ground < hi)]
            return float(np.ptp(sel[:, axis])) if len(sel) else 0.
        return {'mass_kg': mass, 'wheelbase_m': float(wheelbase), 'track_m': float(track), 'wheel_radius_m': radius,
                'com_base_m': com.tolist(), 'com_height_m': float(com[2] - ground), 'base_origin_height_m': float(-ground),
                'collision_length_x_m': float(np.ptp(collision[:, 0])), 'collision_width_y_m': float(np.ptp(collision[:, 1])),
                'visual_length_x_m': float(np.ptp(visual[:, 0])), 'visual_width_y_m': float(np.ptp(visual[:, 1])),
                'visual_lowest_non_leg_m': float(min(pts[:, 2].min() for pts in [visual[np.abs(visual[:, 1]) < .15]]) - ground),
                'collision_x_below_0p30_m': band(collision, 0, .30, 0), 'visual_x_below_0p15_m': band(visual, 0, .15, 0),
                'visual_x_0p15_to_0p30_m': band(visual, .15, .30, 0), 'visual_x_above_0p30_m': band(visual, .30, 9, 0),
                'visual_y_below_0p15_m': band(visual, 0, .15, 1), 'visual_y_0p15_to_0p30_m': band(visual, .15, .30, 1), 'visual_y_above_0p30_m': band(visual, .30, 9, 1),
                'underbody_collision_bottom_m': float(collision[np.abs(collision[:, 1]) > .30][:, 2].min() - ground)}


def main():
    params = yaml.safe_load((PKG / 'config/params.yaml').read_text()); robot = Robot(params)
    r = params['robot']; drive = params['drive']; power = params['power']
    radius = r['wheel_radius_m']; width = r['wheel_width_m']; mass = r['mass_kg']
    out = {'scope': 'Static rigid-body estimates of an assumed-parameter model; simulation measurements are labelled measured',
           'inputs': {'mass_kg': mass, 'wheel_radius_m': radius, 'wheel_width_m': width, 'assumed_com_height_m': r['com_height_m'],
                      'drive_output_torque_limit_Nm': drive['output_torque_limit_Nm']}}
    geometry = {}
    for name, d in SHAPES.items():
        geometry[name] = {'straight': robot.analyse(d, 0.), 'crab_wheels_90deg': robot.analyse(d, math.pi / 2)}
    out['geometry'] = geometry
    wheels = sorted(n for n in robot.links if n.endswith('_wheel'))
    out['wheel_count'] = {'links': wheels, 'count': len(wheels), 'telescope_stages_per_side': 6}
    com_h = geometry['NORMAL']['straight']['com_height_m']; com_x = geometry['NORMAL']['straight']['com_base_m'][0]
    track = geometry['NORMAL']['straight']['track_m']

    # --- Zone 1: trenches -------------------------------------------------------------
    def wheel_drop(w):   # rigid wheel centred over a gap of width w with square edges
        return radius - math.sqrt(radius ** 2 - (w / 2) ** 2) if w < 2 * radius else None
    zone1 = {'wheel_diameter_m': 2 * radius,
             'transverse_crossing': {
                 'principle': 'With two axles the support polygon collapses to the rear contact line as soon as the front axle is '
                              'unsupported; the CoM is ahead of that line for every wheelbase, so the front axle drops. A longer '
                              'wheelbase does not bridge a gap wider than the wheel can span.',
                 'max_gap_spanned_by_one_wheel_m': 2 * radius,
                 'wheel_drop_m': {f'{int(w * 100)}cm': wheel_drop(w) for w in (.05, .10, .14, .25, .35, .40, .45, .55)},
                 'pitch_acceleration_when_front_unsupported_rad_s2': {
                     name: mass * G * (g['straight']['wheelbase_m'] / 2 + com_x) /
                     (mass * ((g['straight']['wheelbase_m'] / 2) ** 2 + g['straight']['com_height_m'] ** 2))
                     for name, g in geometry.items()}},
             'straddle_inner_gap_m': {name: {'straight_entry_between_left_right_wheels': track - width,
                                             'crab_entry_between_front_rear_wheels': g['straight']['wheelbase_m'] - width}
                                      for name, g in geometry.items()}}
    table = {}
    for label, widths in (('ORIGINAL_25_35_45', (.25, .35, .45)), ('PROPOSED_25_40_55', (.25, .40, .55))):
        table[label] = {}
        for w in widths:
            table[label][f'{int(w * 100)}cm'] = {
                name: {'crab_straddle_margin_per_side_mm': round(1000 * (g['straight']['wheelbase_m'] - width - w) / 2, 1),
                       'straight_straddle_margin_per_side_mm': round(1000 * (track - width - w) / 2, 1)}
                for name, g in geometry.items()}
    zone1['longitudinal_trench_straddle'] = table
    out['zone1_trench'] = zone1

    # --- Zone 2: narrow passage -------------------------------------------------------
    zone2 = {}
    for name, g in geometry.items():
        s, c = g['straight'], g['crab_wheels_90deg']
        zone2[name] = {'straight_entry_collision_width_mm': round(1000 * s['collision_width_y_m']), 'straight_entry_visual_width_mm': round(1000 * s['visual_width_y_m']),
                       'straight_entry_over_wheels_mm': round(1000 * (track + width)),
                       'crab_entry_collision_width_mm': round(1000 * c['collision_length_x_m']), 'crab_entry_visual_width_mm': round(1000 * c['visual_length_x_m']),
                       'crab_entry_legs_only_below_0p30m_mm': round(1000 * c['collision_x_below_0p30_m']),
                       'crab_visual_width_by_height_mm': {'0-150': round(1000 * c['visual_x_below_0p15_m']), '150-300': round(1000 * c['visual_x_0p15_to_0p30_m']), '>300': round(1000 * c['visual_x_above_0p30_m'])}}
        for gap in (.32, .34):
            zone2[name][f'crab_margin_per_side_collision_{int(gap * 100)}cm_mm'] = round(1000 * (gap - c['collision_length_x_m']) / 2, 1)
            zone2[name][f'crab_margin_per_side_visual_{int(gap * 100)}cm_mm'] = round(1000 * (gap - c['visual_length_x_m']) / 2, 1)
    out['zone2_passage'] = zone2

    # --- Zone 3: ramp -----------------------------------------------------------------
    force = 4 * drive['output_torque_limit_Nm'] / radius
    zone3 = {'total_drive_force_limit_N': force, 'weight_N': mass * G,
             'torque_limited_grade_deg_no_resistance': math.degrees(math.asin(min(1., force / (mass * G)))),
             'force_needed_N': {f'{a}deg': mass * G * math.sin(math.radians(a)) for a in (10, 15, 20, 25)},
             'wheel_torque_needed_Nm_per_wheel': {f'{a}deg': mass * G * math.sin(math.radians(a)) * radius / 4 for a in (10, 15, 20, 25)},
             'friction_needed_mu': {f'{a}deg': math.tan(math.radians(a)) for a in (10, 15, 20, 25)},
             'motor_torque_available_by_current_limit_Nm': drive['kt_Nm_A'] * drive['current_limit_A'] * drive['gear_ratio'] * drive['gear_efficiency'],
             'static_pitch_tip_angle_deg': {}}
    for h_label, h in (('computed_from_urdf', com_h), ('300mm', .30), ('340mm_assumed', .34), ('400mm', .40)):
        zone3['static_pitch_tip_angle_deg'][h_label] = {'com_height_m': h, **{name: math.degrees(math.atan((g['straight']['wheelbase_m'] / 2 - abs(com_x)) / h)) for name, g in geometry.items()},
                                                        'roll_any_shape': math.degrees(math.atan((track / 2) / h))}
    out['zone3_ramp'] = zone3

    # --- Zone 4: slip -----------------------------------------------------------------
    per_wheel_force = drive['output_torque_limit_Nm'] / radius; normal = mass * G / 4
    control_accel = 1.2
    zone4 = {'drive_force_limit_per_wheel_N': per_wheel_force, 'normal_load_per_wheel_N': normal,
             'friction_limit_per_wheel_N': {f'mu_{mu}': mu * normal for mu in (.35, .30, .25, .22, .20, .15)},
             'mu_below_which_full_torque_slips_on_level_ground': per_wheel_force / normal,
             'max_acceleration_by_torque_m_s2': force / mass,
             'mu_below_which_the_current_1p2_m_s2_ramp_slips': control_accel / G,
             'torque_per_wheel_to_slip_at_mu_0p35_Nm': .35 * normal * radius,
             'note': 'Gazebo uses min(wheel mu 1.2, ground mu). Level ground, equal wheel loads, no rolling resistance.'}
    out['zone4_slip'] = zone4

    # --- Speed, acceleration, morph time ------------------------------------------------
    available_v = power['ocv_full_V'] - drive['bridge_drop_V']; no_load = available_v / (drive['ke_Vs_rad'] * drive['gear_ratio'])
    travel = SHAPES['EXTENDED'] - SHAPES['COMPACT']
    stage_limit = float(robot.root.find("joint[@name='telR1_joint']/limit").get('velocity'))
    wheel_limit = float(robot.root.find("joint[@name='fl_wheel_joint']/limit").get('velocity'))
    out['speed_and_acceleration'] = {
        'motor_no_load_wheel_speed_rad_s': no_load, 'motor_no_load_ground_speed_m_s': no_load * radius,
        'urdf_wheel_velocity_limit_ground_speed_m_s': wheel_limit * radius,
        'control_mode_speed_caps_m_s': {k: v['speed'] for k, v in params['control_modes'].items()},
        'spec_speeds_m_s': {'exhibition_burst': 2.0, 'example_speed_cap': 1.10, 'zone4_cap': 0.9},
        'max_acceleration_by_torque_m_s2': force / mass,
        'proposed_terrain_table': {name: {'A_max': a, 'J_max': j, 'J_over_A': j / a, 'J_if_2p5A': 2.5 * a, 'A_reachable_by_torque': a <= force / mass}
                                   for name, (a, j) in {'Road': (3.5, 8.0), 'Grass': (2.8, 6.0), 'Sand': (1.5, 4.0), 'Mud': (1.2, 3.0), 'Snow': (.8, 2.0)}.items()}}
    out['morph_time'] = {'stage_travel_compact_to_extended_m': travel, 'wheelbase_change_m': 8 * travel,
                         'commanded_stage_rate_m_s': .0045, 'urdf_stage_velocity_limit_m_s': stage_limit,
                         'full_morph_s_at_commanded_rate': travel / .0045, 'full_morph_s_at_urdf_limit': travel / stage_limit,
                         'stage_rate_needed_for_2s_m_s': travel / 2., 'ratio_to_urdf_limit': travel / 2. / stage_limit}

    def trigger(v, t_morph, a):   # [PROPOSED] corrected formula
        return v * t_morph + v * v / (2 * a)
    def gated(v, t_morph, a, gate=.22, hold=.15):   # what the 0.22 m/s morph gate implies
        return max(0., (v * v - hold * hold) / (2 * a)) + hold * t_morph
    out['predictive_morph_trigger_distance_m'] = {
        f'v{v}_T{t}': {'proposed_v_T_plus_v2_over_2a': trigger(v, t, 1.2), 'with_0p22_gate_decel_then_morph_at_0p15': gated(v, t, 1.2)}
        for v in (.3, .6) for t in (2., 5., 11.5, 16.7)}
    text = json.dumps(out, indent=2)
    if len(sys.argv) > 1: Path(sys.argv[1]).write_text(text)
    print(text)


if __name__ == '__main__':
    main()
