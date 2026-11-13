"""V2 commit 1: trimmed chassis variant, benchmark arena fixtures and terramechanics model."""
import json, math, shutil, subprocess, sys
from pathlib import Path
import xml.etree.ElementTree as ET
import pytest
import yaml

P = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(P)); sys.path.insert(0, str(P / 'tools'))
from morph_sim.terramechanics import wheel_soil_state, solve_slip_parameter, virgin_work, tandem_resistance
from morph_sim.parameters import resolve_profile
import phase0_feasibility as feasibility

PARAMS = resolve_profile(P)[0]
CONFIG = PARAMS['terramechanics']
RADIUS, WIDTH = PARAMS['robot']['wheel_radius_m'], PARAMS['robot']['wheel_width_m']
ARENA = json.loads((P / 'config/tetra_arena.json').read_text())
WORLD = ET.parse(P / 'worlds/tetra_arena.sdf').getroot().find('world')
HALF_PI = math.pi / 2


def boxes():
    out = {}
    for collision in WORLD.findall('./model/link/collision'):
        if collision.find('geometry/box') is None or collision.get('name').startswith(('rock_', 'rubble_')): continue
        pose = [float(v) for v in collision.findtext('pose').split()]; size = [float(v) for v in collision.findtext('geometry/box/size').split()]
        out[collision.get('name')] = (pose, size, float(collision.findtext('surface/friction/ode/mu')))
    return out


# --- chassis -----------------------------------------------------------------------------------
def envelope(chassis, d, steer):
    robot = feasibility.Robot(PARAMS) if chassis == 'trimmed' else None
    if robot is None:
        import xacro
        robot = feasibility.Robot.__new__(feasibility.Robot)
        doc = xacro.process_file(str(P / 'urdf/morph.urdf.xacro'), mappings={'controllers_file': 'controllers.yaml', 'wheel_radius': str(RADIUS), 'wheel_width': str(WIDTH), 'chassis': 'cad'})
        robot.root = ET.fromstring(doc.toxml()); robot.links = {l.get('name'): l for l in robot.root.findall('link')}; robot.children = {}; robot.mesh_cache = {}
        for j in robot.root.findall('joint'): robot.children.setdefault(j.find('parent').get('link'), []).append(j)
    original = robot.poses
    def poses(d_, _steer):
        import numpy as np
        q = {f'tel{s}{i}_joint': d_ for s in 'RL' for i in range(1, 7)}; q.update(dict(zip(('fl_steer_joint', 'fr_steer_joint', 'rl_steer_joint', 'rr_steer_joint'), steer)))
        pose = {'base_link': np.eye(4)}; stack = ['base_link']
        while stack:
            parent = stack.pop()
            for j in robot.children.get(parent, []):
                m = pose[parent] @ feasibility.origin(j); axis_el = j.find('axis')
                axis = np.array([float(v) for v in axis_el.get('xyz').split()]) if axis_el is not None else np.array([1., 0, 0]); value = q.get(j.get('name'), 0.)
                if j.get('type') == 'prismatic': m = m @ feasibility.transform(axis * value)
                elif j.get('type') in ('revolute', 'continuous'):
                    k = axis / np.linalg.norm(axis); kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
                    a = np.eye(4); a[:3, :3] = np.eye(3) + math.sin(value) * kx + (1 - math.cos(value)) * kx @ kx; m = m @ a
                child = j.find('child').get('link'); pose[child] = m; stack.append(child)
        return pose
    robot.poses = poses
    result = robot.analyse(d, 0.); robot.poses = original
    return result


def test_trimmed_chassis_is_330_mm_long_in_compact_crab_with_drive_units_inward():
    inward = envelope('trimmed', feasibility.SHAPES['COMPACT'], (-HALF_PI, -HALF_PI, HALF_PI, HALF_PI))
    assert inward['visual_length_x_m'] == pytest.approx(.330, abs=.001) and inward['collision_length_x_m'] <= .3305
    outward = envelope('trimmed', feasibility.SHAPES['COMPACT'], (HALF_PI, HALF_PI, -HALF_PI, -HALF_PI))
    assert outward['visual_length_x_m'] > PINCH_WIDTH               # the other crab solution does not fit the pinch


PINCH_WIDTH = next(z for z in ARENA['zones'] if z['id'] == 'pinch')['clear_width_m']


def test_cad_chassis_variant_is_unchanged_and_longer():
    cad = envelope('cad', feasibility.SHAPES['COMPACT'], (-HALF_PI, -HALF_PI, HALF_PI, HALF_PI))
    assert cad['visual_length_x_m'] == pytest.approx(.469, abs=.003) and cad['mass_kg'] == pytest.approx(10.)


def test_collision_envelope_covers_the_visible_frame_and_mass_is_unchanged():
    straight = envelope('trimmed', 0., (0., 0., 0., 0.))
    assert straight['mass_kg'] == pytest.approx(PARAMS['robot']['mass_kg'])
    assert straight['collision_width_y_m'] >= .77 and straight['visual_width_y_m'] <= .801
    assert straight['com_height_m'] == pytest.approx(.2911, abs=.001)   # collisions carry no mass


def test_only_normal_and_extended_fail_the_pinch():
    for name in ('NORMAL', 'EXTENDED'):
        assert envelope('trimmed', feasibility.SHAPES[name], (-HALF_PI, -HALF_PI, HALF_PI, HALF_PI))['collision_length_x_m'] > PINCH_WIDTH


# --- arena -------------------------------------------------------------------------------------
def test_world_identity_and_engine_settings_match_the_reference_arena():
    assert WORLD.get('name') == ARENA['world'] == 'morph_tetra'
    reference = ET.parse(P / 'worlds/multisector.sdf').getroot().find('world')
    assert WORLD.findtext('physics/dart/collision_detector') == reference.findtext('physics/dart/collision_detector')
    assert [p.get('filename') for p in WORLD.findall('plugin')] == [p.get('filename') for p in reference.findall('plugin')]


def test_trench_widths_are_100_130_155_mm():
    slabs = sorted((pose[0] - size[0] / 2, pose[0] + size[0] / 2) for name, (pose, size, _) in boxes().items() if name.startswith('tarmac_') and name != 'tarmac_main_collision')
    gaps = [round(b[0] - a[1], 4) for a, b in zip(slabs, slabs[1:])]
    assert gaps == [.100, .130, .155]
    assert [t['width_m'] for t in next(z for z in ARENA['zones'] if z['id'] == 'trenches')['trenches']] == gaps
    floor = boxes()['trench_floor_collision']; assert floor[0][2] + floor[1][2] / 2 == pytest.approx(-ARENA['trench_depth_m'])


def test_ramp_angles_and_top_edges_meet_their_platforms():
    b = boxes()
    for ramp in next(z for z in ARENA['zones'] if z['id'] == 'ramps')['ramps']:
        pose, size, _ = b[ramp['id'] + '_up_collision']; theta = -pose[4]
        assert math.degrees(theta) == pytest.approx(ramp['angle_deg'], abs=1e-3)      # SDF pose is written with 6 decimals
        top_x = pose[0] + math.cos(theta) * size[0] / 2 - math.sin(theta) * size[2] / 2      # upper end of the top surface
        top_z = pose[2] + math.sin(theta) * size[0] / 2 + math.cos(theta) * size[2] / 2
        assert top_x == pytest.approx(ramp['x_top'], abs=1e-4) and top_z == pytest.approx(ramp['rise_m'], abs=1e-4)
        platform = b[ramp['id'] + '_platform_collision']; assert platform[0][2] + platform[1][2] / 2 == pytest.approx(ramp['rise_m'])
    ramps = next(z for z in ARENA['zones'] if z['id'] == 'ramps')['ramps']
    assert sorted(r['angle_deg'] for r in ramps) == [15., 16., 18., 20., 22.] and [r['angle_deg'] for r in ramps if r['down_ramp']] == [16.]


def test_passage_clear_widths():
    b = boxes()
    for prefix, zone_id, width in (('kerb', 'corridor', .620), ('pinch', 'pinch', .380)):
        left, right = b[prefix + '_left_collision'], b[prefix + '_right_collision']
        clear = (left[0][1] - left[1][1] / 2) - (right[0][1] + right[1][1] / 2)
        assert clear == pytest.approx(width, abs=1e-4) == next(z for z in ARENA['zones'] if z['id'] == zone_id)['clear_width_m']
    # Kerbs stay below every drive-unit part (lowest is the hub at 42 mm) so only the tyres pass between them.
    assert b['kerb_left_collision'][1][2] <= .040 and b['pinch_left_collision'][1][2] >= .50


def test_friction_comes_from_the_parameter_table():
    b = boxes()
    assert b['tarmac_main_collision'][2] == pytest.approx(PARAMS['surfaces']['tarmac']['mu_kinetic_x'])
    assert b['sand_lane_collision'][2] == pytest.approx(.35) and b['clay_lane_collision'][2] == pytest.approx(.25)      # V2 brief values
    for name, surface in (('sand_lane_collision', 'dry_sand'), ('clay_lane_collision', 'mud'), ('tarmac_main_collision', 'tarmac')):
        assert ARENA['collision_surfaces'][name]['surface'] == surface
    for zone in ARENA['zones']:
        if zone.get('soil'): assert zone['soil'] in CONFIG['soils']


def test_soil_zones_reach_the_robot_model_as_a_terramechanics_plugin(tmp_path):
    if shutil.which('gz') is None: pytest.skip('gz executable not available')
    from morph_sim.assets import prepare_robot
    zones = [z for z in ARENA['zones'] if z.get('soil')]
    _, sdf = prepare_robot(P, tmp_path, physics_profile=True, position_backend='trajectory', soil_zones=zones)
    plugins = ET.parse(sdf).getroot().findall('./model/plugin')
    assert plugins[-1].get('name') == 'morph_sim::MorphTerramechanics'          # must run after the force-writing plugins
    assert [z.findtext('name') for z in plugins[-1].findall('zone')] == ['sand', 'mud']
    assert float(plugins[-1].find('zone').findtext('kphi')) == CONFIG['soils']['loose_sand']['kphi']
    assert len(plugins[-1].findall('wheel')) == 4
    _, plain = prepare_robot(P, tmp_path / 'plain', physics_profile=True, position_backend='trajectory')
    assert all('Terramechanics' not in p.get('name') for p in ET.parse(plain).getroot().findall('./model/plugin'))


# --- terramechanics model ----------------------------------------------------------------------
SAND = CONFIG['soils']['loose_sand']


def test_no_load_no_soil_reaction():
    for load in (0., -5., float('nan')):
        state = wheel_soil_state(load, 3., RADIUS, WIDTH, SAND, CONFIG)
        assert state['sinkage_m'] == 0 and state['resistance_N'] == 0


def test_sinkage_and_resistance_increase_with_load():
    states = [wheel_soil_state(w, 0., RADIUS, WIDTH, SAND, CONFIG) for w in (5., 15., 24.5, 40., 60.)]
    for a, b in zip(states, states[1:]):
        assert b['static_sinkage_m'] > a['static_sinkage_m'] and b['compaction_resistance_N'] > a['compaction_resistance_N']


def test_sinkage_exponent_matches_the_rigid_wheel_formula():
    soil = dict(SAND, n=1.0)                                   # z ~ W^(2/3) for n = 1
    a = wheel_soil_state(10., 0., RADIUS, WIDTH, soil, CONFIG)['static_sinkage_m']; b = wheel_soil_state(20., 0., RADIUS, WIDTH, soil, CONFIG)['static_sinkage_m']
    assert b / a == pytest.approx(2 ** (2 / 3), rel=1e-9)
    assert a == pytest.approx((3 * 10. / (2. * (soil['kc'] + WIDTH * soil['kphi']) * math.sqrt(2 * RADIUS))) ** (2 / 3), rel=1e-12)


def test_virgin_resistance_equals_bekker_compaction_resistance():
    state = wheel_soil_state(24.5, 0., RADIUS, WIDTH, SAND, CONFIG)
    z = state['sinkage_m']; expected = WIDTH * (SAND['kc'] / WIDTH + SAND['kphi']) * z ** (SAND['n'] + 1) / (SAND['n'] + 1)
    assert state['compaction_resistance_N'] == pytest.approx(expected, rel=1e-12) == pytest.approx(virgin_work(z, WIDTH, SAND))


def test_wheel_in_a_rut_only_pays_for_new_compaction():
    front, rear = tandem_resistance(24.5, 24.5, 0., RADIUS, WIDTH, SAND, CONFIG)
    assert rear['compaction_resistance_N'] == pytest.approx(CONFIG['reload_resistance_fraction'] * front['compaction_resistance_N'])
    front, heavy = tandem_resistance(15., 34., 0., RADIUS, WIDTH, SAND, CONFIG)
    virgin = wheel_soil_state(34., 0., RADIUS, WIDTH, SAND, CONFIG)['compaction_resistance_N']
    assert heavy['sinkage_m'] > front['sinkage_m'] and CONFIG['reload_resistance_fraction'] * front['compaction_resistance_N'] < heavy['compaction_resistance_N'] < virgin
    assert heavy['rut_depth_m'] == heavy['sinkage_m']


def test_soil_slip_grows_with_thrust_and_respects_the_limit():
    slips = [wheel_soil_state(24.5, h, RADIUS, WIDTH, SAND, CONFIG)['slip_ratio'] for h in (0., 2., 5., 10., 14., 500.)]
    assert slips[0] == 0 and all(b >= a for a, b in zip(slips, slips[1:])) and slips[-1] == pytest.approx(CONFIG['slip_ratio_limit'], abs=1e-6)
    x = solve_slip_parameter(.4); assert 1 - (1 - math.exp(-x)) / x == pytest.approx(.4, abs=1e-9)


def test_optional_couplings_are_off_by_default_and_act_when_enabled():
    assert CONFIG['slip_sinkage_gain'] == 0 and CONFIG['slip_drag_enabled'] == 0
    base = wheel_soil_state(24.5, 6., RADIUS, WIDTH, SAND, CONFIG)
    assert base['sinkage_m'] == base['static_sinkage_m'] and base['slip_resistance_N'] == 0 and base['slip_ratio'] > 0
    coupled = wheel_soil_state(24.5, 6., RADIUS, WIDTH, SAND, dict(CONFIG, slip_sinkage_gain=1., slip_drag_enabled=1.))
    assert coupled['sinkage_m'] > base['sinkage_m'] and coupled['slip_resistance_N'] > 0


def test_cpp_model_matches_the_python_reference():
    cli = next((p for p in [P.parents[1] / 'install/morph_sim/lib/morph_sim/terramechanics_cli', P.parents[1] / 'build/morph_sim/terramechanics_cli'] if p.exists()), None)
    if cli is None: pytest.skip('terramechanics_cli not built')
    for soil_name, soil in CONFIG['soils'].items():
        for normal, thrust, rut, gain, drag in [(24.5, 0., 0., 0., 0.), (40., 5., .004, 0., 0.), (12., 3., .02, 1., 1.), (60., 14., 0., 1., 0.)]:
            cfg = dict(CONFIG, slip_sinkage_gain=gain, slip_drag_enabled=drag)
            args = [normal, thrust, RADIUS, WIDTH, soil['kc'], soil['kphi'], soil['n'], soil['cohesion_Pa'], soil['phi_rad'], soil['shear_length_m'],
                    cfg['sinkage_limit_radius_fraction'], cfg['reload_resistance_fraction'], gain, drag, cfg['slip_ratio_limit'], rut]
            native = json.loads(subprocess.run([str(cli)] + [repr(float(a)) for a in args], capture_output=True, text=True, check=True).stdout)
            reference = wheel_soil_state(normal, thrust, RADIUS, WIDTH, soil, cfg, rut)
            for key, value in reference.items(): assert native[key] == pytest.approx(value, rel=1e-9, abs=1e-12), (soil_name, key)


def test_rough_zones_are_reproducible_and_low_enough_to_drive_over():
    for zone_id, limit in (('rocks', .020), ('rubble', .012)):
        zone = next(z for z in ARENA['zones'] if z['id'] == zone_id)
        assert len(zone['obstacles']) >= 25 and zone['obstacle_height_m'] == limit
    for collision in WORLD.findall('./model/link/collision'):
        pose = [float(v) for v in collision.findtext('pose').split()]
        if collision.get('name').startswith('rock_'): assert pose[2] + float(collision.findtext('geometry/sphere/radius')) == pytest.approx(.020, abs=1e-4)
        if collision.get('name').startswith('rubble_'): assert float(collision.findtext('geometry/box/size').split()[2]) == pytest.approx(.012)
    import build_tetra_arena
    before = (P / 'worlds/tetra_arena.sdf').read_text(), (P / 'config/tetra_arena.json').read_text()
    build_tetra_arena.main()                                        # same seed -> byte-identical files
    assert before == ((P / 'worlds/tetra_arena.sdf').read_text(), (P / 'config/tetra_arena.json').read_text())


def test_rail_holding_force_options_and_the_25_newton_default():
    import xacro
    from morph_sim.assets import STAGE_EFFORT_N
    assert STAGE_EFFORT_N == {'backdrivable': 8., 'reinforced': 25., 'self_locking': 60.}
    for hold, effort in STAGE_EFFORT_N.items():
        doc = xacro.process_file(str(P / 'urdf/morph.urdf.xacro'), mappings={'controllers_file': 'controllers.yaml', 'wheel_radius': str(RADIUS), 'wheel_width': str(WIDTH), 'stage_effort': str(effort)})
        limits = [j.find('limit') for j in ET.fromstring(doc.toxml()).findall('joint') if j.get('name').startswith('tel')]
        assert len(limits) == 12 and all(float(l.get('effort')) == effort and float(l.get('velocity')) == .006 for l in limits)
    default = xacro.process_file(str(P / 'urdf/morph.urdf.xacro'), mappings={'controllers_file': 'controllers.yaml', 'wheel_radius': str(RADIUS), 'wheel_width': str(WIDTH)})
    assert float(ET.fromstring(default.toxml()).find("joint[@name='telR1_joint']/limit").get('effort')) == 25.       # V2 brief default
