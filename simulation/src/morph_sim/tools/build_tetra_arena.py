#!/usr/bin/env python3
"""Generate the Tetra-Arena benchmark world: worlds/tetra_arena.sdf and config/tetra_arena.json.

    python3 src/morph_sim/tools/build_tetra_arena.py

One straight course along +x, 10 m wide, built from static boxes (deterministic, no randomness):

    start | stepped trenches 100/130/155 mm | loose sand | mud | ramps | corridor 620 mm | pinch 380 mm | rocks | rubble | finish

Friction comes from config/params.yaml (surfaces.<name>.mu_kinetic_*), except the sand and mud lanes, which use
the values fixed by the V2 brief (mu 0.35 and 0.25); soil constants for the
terramechanics zones come from terramechanics.soils. World settings (physics, systems, GUI, light)
are copied from worlds/multisector.sdf so the two arenas run under identical engine settings.

Fixture notes that follow from the Phase 0 geometry audit:
- The 620 mm lane has 40 mm kerbs. Only the tyres (586 mm over both) pass between them; the drive
  units and frame (up to 701 mm wide from 42 mm above ground) pass over. Full-height walls would
  need at least 760 mm.
- The 380 mm pinch has full-height walls. It admits the trimmed chassis only in Compact, yawed 90
  degrees, with the drive units turned inward (330 mm envelope, 25 mm clearance per side).
- Rocks are buried spheres standing 20 mm proud and rubble pieces are 12 mm boards: with 0.5 Nm per wheel the
  quasi-static single-wheel step limit is about 25 mm, so taller obstacles would be barriers, not rough ground.
"""
import json, math, random
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

PKG = Path(__file__).resolve().parents[1]
HALF_WIDTH = 5.0
SLAB = 0.25                       # ground slab thickness = trench depth [m]
X_START, X_END = -2.0, 46.0
ROCKS = (36.0, 39.5); RUBBLE = (40.5, 44.0)
SEED = 42                         # obstacle placement only; fixtures are deterministic
TRENCHES = [(4.5, 0.100), (6.0, 0.130), (7.5, 0.155)]          # (centre x, width) [m]
SAND = (9.5, 14.5); CLAY = (14.5, 17.5)
RAMP_X0 = 20.0; RAMP_RISE = 0.45; PLATFORM = 1.5
RAMPS = [  # (id, centre y, width, angle deg, has down ramp)
    ('ramp_tour_16', 0.0, 1.6, 16.0, True), ('ramp_lab_15', -2.0, 1.4, 15.0, False), ('ramp_lab_18', 2.0, 1.4, 18.0, False),
    ('ramp_lab_20', -3.7, 1.4, 20.0, False), ('ramp_lab_22', 3.7, 1.4, 22.0, False)]
LANE_FRICTION = {'sand_lane': 0.35, 'clay_lane': 0.25}        # V2 brief values for the soil lanes
KERB_LANE = (29.0, 31.0, 0.620, 0.040)                        # x0, x1, clear width, kerb height
PINCH = (32.5, 34.5, 0.380, 0.60)                             # x0, x1, clear width, wall height
COLOURS = {'tarmac': '0.22 0.22 0.24 1', 'dry_sand': '0.76 0.66 0.42 1', 'mud': '0.33 0.24 0.16 1', 'concrete': '0.55 0.55 0.53 1',
           'kerb': '0.85 0.55 0.10 1', 'wall': '0.70 0.25 0.20 1', 'floor': '0.10 0.10 0.11 1'}


def main():
    params = yaml.safe_load((PKG / 'config/params.yaml').read_text())
    source = ET.parse(PKG / 'worlds/multisector.sdf').getroot().find('world')
    sdf = ET.Element('sdf', version='1.8'); world = ET.SubElement(sdf, 'world', name='morph_tetra')
    for child in source:
        if child.tag not in ('model', 'include'): world.append(child)
    camera = world.find("./gui/plugin[@filename='MinimalScene']/camera_pose")
    if camera is not None: camera.text = '-6 -7 5 0 0.45 0.75'
    model = ET.SubElement(world, 'model', name='tetra_arena'); ET.SubElement(model, 'static').text = 'true'
    link = ET.SubElement(model, 'link', name='arena'); surfaces = {}

    def box(name, centre, size, surface, colour=None, pitch=0.):
        mu = [params['surfaces'][surface]['mu_kinetic_x'], params['surfaces'][surface]['mu_kinetic_y']]
        if name in LANE_FRICTION: mu = [LANE_FRICTION[name]] * 2
        pose = f'{centre[0]:.5f} {centre[1]:.5f} {centre[2]:.5f} 0 {pitch:.6f} 0'; dims = ' '.join(f'{v:.5f}' for v in size)
        collision = ET.SubElement(link, 'collision', name=name + '_collision'); ET.SubElement(collision, 'pose').text = pose
        ET.SubElement(ET.SubElement(ET.SubElement(collision, 'geometry'), 'box'), 'size').text = dims
        ode = ET.SubElement(ET.SubElement(ET.SubElement(collision, 'surface'), 'friction'), 'ode')
        ET.SubElement(ode, 'mu').text = str(mu[0]); ET.SubElement(ode, 'mu2').text = str(mu[1])
        visual = ET.SubElement(link, 'visual', name=name + '_visual'); ET.SubElement(visual, 'pose').text = pose
        ET.SubElement(ET.SubElement(ET.SubElement(visual, 'geometry'), 'box'), 'size').text = dims
        material = ET.SubElement(visual, 'material'); rgba = COLOURS[colour or surface]
        ET.SubElement(material, 'ambient').text = rgba; ET.SubElement(material, 'diffuse').text = rgba
        surfaces[name + '_collision'] = {'surface': surface, 'mu': mu[0], 'mu2': mu[1]}

    def slab(name, x0, x1, surface):
        box(name, ((x0 + x1) / 2, 0, -SLAB / 2), (x1 - x0, 2 * HALF_WIDTH, SLAB), surface)

    # Ground: slabs with gaps for the trenches, soil lanes, and a floor under everything.
    box('trench_floor', ((X_START + X_END) / 2, 0, -SLAB - .05), (X_END - X_START, 2 * HALF_WIDTH, .1), 'concrete', 'floor')
    edges = [X_START] + [v for x, w in TRENCHES for v in (x - w / 2, x + w / 2)] + [SAND[0]]
    for i in range(0, len(edges), 2): slab(f'tarmac_{i // 2}', edges[i], edges[i + 1], 'tarmac')
    slab('sand_lane', SAND[0], SAND[1], 'dry_sand'); slab('clay_lane', CLAY[0], CLAY[1], 'mud'); slab('tarmac_main', CLAY[1], X_END, 'tarmac')

    ramps = []
    for name, y, width, angle, down in RAMPS:
        theta = math.radians(angle); run = RAMP_RISE / math.tan(theta); length = RAMP_RISE / math.sin(theta); t = .05
        def plate(tag, x0, sign):   # sign +1 climbs towards +x, -1 descends towards +x
            centre = (x0 + run / 2 + sign * math.sin(theta) * t / 2, y, RAMP_RISE / 2 - math.cos(theta) * t / 2)
            box(f'{name}_{tag}', centre, (length, width, t), 'concrete', pitch=-sign * theta)
        plate('up', RAMP_X0, 1)
        x_top = RAMP_X0 + run
        box(f'{name}_platform', (x_top + PLATFORM / 2, y, RAMP_RISE / 2), (PLATFORM, width, RAMP_RISE), 'concrete')
        if down: plate('down', x_top + PLATFORM, -1)
        else: box(f'{name}_stop', (x_top + PLATFORM - .03, y, RAMP_RISE + .06), (.06, width, .12), 'concrete', 'kerb')
        ramps.append({'id': name, 'y': y, 'width': width, 'angle_deg': angle, 'x_foot': RAMP_X0, 'run_m': run, 'rise_m': RAMP_RISE,
                      'x_top': x_top, 'platform_m': PLATFORM, 'x_end': x_top + PLATFORM + (run if down else 0.), 'down_ramp': down})

    x0, x1, clear, height = KERB_LANE
    for side in (1, -1): box(f'kerb_{"left" if side > 0 else "right"}', ((x0 + x1) / 2, side * (clear / 2 + .05), height / 2), (x1 - x0, .10, height), 'concrete', 'kerb')
    x0, x1, clear, height = PINCH
    for side in (1, -1): box(f'pinch_{"left" if side > 0 else "right"}', ((x0 + x1) / 2, side * (clear / 2 + .075), height / 2), (x1 - x0, .15, height), 'concrete', 'wall')

    # Rough ground: fixed seed so the same file is regenerated every time.
    rng = random.Random(SEED); obstacles = {'rocks': [], 'rubble': []}
    def shape(name, geometry, pose, surface, colour):
        mu = [params['surfaces'][surface]['mu_kinetic_x'], params['surfaces'][surface]['mu_kinetic_y']]
        for tag in ('collision', 'visual'):
            element = ET.SubElement(link, tag, name=f'{name}_{tag}'); ET.SubElement(element, 'pose').text = pose
            g = ET.SubElement(element, 'geometry'); kind, dims = geometry
            if kind == 'sphere': ET.SubElement(ET.SubElement(g, 'sphere'), 'radius').text = f'{dims:.4f}'
            else: ET.SubElement(ET.SubElement(g, 'box'), 'size').text = ' '.join(f'{v:.4f}' for v in dims)
            if tag == 'collision':
                ode = ET.SubElement(ET.SubElement(ET.SubElement(element, 'surface'), 'friction'), 'ode'); ET.SubElement(ode, 'mu').text = str(mu[0]); ET.SubElement(ode, 'mu2').text = str(mu[1])
            else:
                material = ET.SubElement(element, 'material'); ET.SubElement(material, 'ambient').text = colour; ET.SubElement(material, 'diffuse').text = colour
        surfaces[name + '_collision'] = {'surface': surface, 'mu': mu[0], 'mu2': mu[1]}
    for i in range(36):       # spheres of 70-110 mm radius buried so that 20 mm stands proud
        radius = rng.uniform(.07, .11); x = rng.uniform(ROCKS[0] + .3, ROCKS[1] - .3); y = rng.uniform(-1.3, 1.3)
        shape(f'rock_{i:02d}', ('sphere', radius), f'{x:.4f} {y:.4f} {0.020 - radius:.4f} 0 0 0', 'wet_rock', '0.38 0.37 0.35 1')
        obstacles['rocks'].append({'x': round(x, 4), 'y': round(y, 4), 'radius_m': round(radius, 4), 'height_m': .020})
    for i in range(28):       # boards 12 mm thick at random headings
        x = rng.uniform(RUBBLE[0] + .3, RUBBLE[1] - .3); y = rng.uniform(-1.3, 1.3); yaw = rng.uniform(0, math.pi); size = (rng.uniform(.15, .40), rng.uniform(.04, .09), .012)
        shape(f'rubble_{i:02d}', ('box', size), f'{x:.4f} {y:.4f} 0.006 0 0 {yaw:.4f}', 'loose_gravel', '0.50 0.42 0.33 1')
        obstacles['rubble'].append({'x': round(x, 4), 'y': round(y, 4), 'yaw': round(yaw, 4), 'size_m': [round(v, 4) for v in size]})

    ET.indent(sdf); (PKG / 'worlds/tetra_arena.sdf').write_text("<?xml version='1.0' encoding='utf-8'?>\n" + ET.tostring(sdf, encoding='unicode') + '\n')
    tour_ramp = ramps[0]
    arena = {
        'world': 'morph_tetra', 'world_file': 'tetra_arena.sdf', 'spawn': [0.0, 0.0, 0.38], 'spawn_yaw': 0.0, 'half_width_m': HALF_WIDTH, 'trench_depth_m': SLAB,
        'x_start': X_START, 'x_end': X_END,
        'zones': [
            {'id': 'start', 'kind': 'flat', 'bounds': [X_START, 3.0, -HALF_WIDTH, HALF_WIDTH], 'surface': 'tarmac'},
            {'id': 'trenches', 'kind': 'trench', 'bounds': [3.0, 9.0, -HALF_WIDTH, HALF_WIDTH], 'surface': 'tarmac',
             'trenches': [{'x': x, 'width_m': w, 'depth_m': SLAB} for x, w in TRENCHES]},
            {'id': 'sand', 'kind': 'soil', 'bounds': [SAND[0], SAND[1], -HALF_WIDTH, HALF_WIDTH], 'surface': 'dry_sand', 'soil': 'loose_sand'},
            {'id': 'mud', 'kind': 'soil', 'bounds': [CLAY[0], CLAY[1], -HALF_WIDTH, HALF_WIDTH], 'surface': 'mud', 'soil': 'soft_clay'},
            {'id': 'ramps', 'kind': 'ramp', 'bounds': [19.0, 27.5, -HALF_WIDTH, HALF_WIDTH], 'surface': 'concrete', 'ramps': ramps},
            {'id': 'corridor', 'kind': 'passage', 'bounds': [KERB_LANE[0], KERB_LANE[1], -.5, .5], 'surface': 'tarmac', 'clear_width_m': KERB_LANE[2], 'wall_height_m': KERB_LANE[3], 'entry': 'head-on'},
            {'id': 'pinch', 'kind': 'passage', 'bounds': [PINCH[0], PINCH[1], -.5, .5], 'surface': 'tarmac', 'clear_width_m': PINCH[2], 'wall_height_m': PINCH[3],
             'entry': 'Compact, yawed 90 deg, crab, drive units inward'},
            {'id': 'rocks', 'kind': 'rough', 'bounds': [ROCKS[0], ROCKS[1], -1.6, 1.6], 'surface': 'tarmac', 'obstacle_height_m': .020, 'obstacles': obstacles['rocks']},
            {'id': 'rubble', 'kind': 'rough', 'bounds': [RUBBLE[0], RUBBLE[1], -1.6, 1.6], 'surface': 'tarmac', 'obstacle_height_m': .012, 'obstacles': obstacles['rubble']},
            {'id': 'finish', 'kind': 'flat', 'bounds': [RUBBLE[1], X_END, -HALF_WIDTH, HALF_WIDTH], 'surface': 'tarmac'}],
        'seed': SEED,
        'tour_waypoints': [[0., 0.], [3., 0.], [9., 0.], [SAND[0], 0.], [CLAY[1], 0.], [RAMP_X0, 0.], [tour_ramp['x_end'], 0.], [KERB_LANE[0], 0.], [KERB_LANE[1], 0.], [PINCH[0], 0.], [PINCH[1], 0.], [ROCKS[0], 0.], [ROCKS[1], 0.], [RUBBLE[0], 0.], [RUBBLE[1], 0.], [45., 0.]],
        'collision_surfaces': surfaces,
        'scope': 'Rigid static boxes; soil lanes get force-level terramechanics from the robot plugin; friction and soil constants are assumed'}
    (PKG / 'config/tetra_arena.json').write_text(json.dumps(arena, indent=2) + '\n')
    print(f"world morph_tetra: {len(surfaces)} boxes; ramps {[(r['id'], r['angle_deg'], round(r['run_m'], 3)) for r in ramps]}")


if __name__ == '__main__':
    main()
