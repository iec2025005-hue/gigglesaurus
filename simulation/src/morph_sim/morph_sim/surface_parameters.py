"""Apply the diagnostic surface table to local collision materials.

DART's Coulomb contact is assigned the kinetic coefficients; the separate static
and soil/tyre prediction models are not claimed to be native engine features.
"""
import json
from pathlib import Path
import xml.etree.ElementTree as ET


def synchronize_world(tree, share, params, wetness=0.):
    sectors = json.loads((Path(share)/'config/sectors.json').read_text())['sectors']
    by_name = {s['id']: s['surface'] for s in sectors}
    access = {'front_crossway': 'tarmac', 'back_crossway': 'tarmac',
              'west_service': 'concrete', 'east_village': 'short_grass',
              'depot_forecourt': 'concrete', 'north_landscape': 'short_grass'}
    records = []
    for model in tree.findall('./world/model'):
        for collision in model.findall('.//collision'):
            name = collision.get('name', '')
            base = name.removesuffix('_collision')
            surface_name = by_name.get(base, access.get(base))
            if base.startswith('mixed'):
                surface_name = by_name['mixed']
            if surface_name is None:
                # Flattened imported obstacles retain world poses in this world.
                pose = [float(v) for v in collision.findtext('pose', '0 0 0 0 0 0').split()]
                surface_name = next((s['surface'] for s in sectors
                    if s['bounds'][0] <= pose[0] < s['bounds'][1]
                    and s['bounds'][2] <= pose[1] < s['bounds'][3]),
                    params['environment']['default_surface'])
            surface = params['surfaces'][surface_name]
            values = [(1-wetness)*surface['mu_kinetic_'+axis] + wetness*surface['wet_mu_'+axis]
                      for axis in ('x', 'y')]
            parent = collision
            for tag in ('surface', 'friction', 'ode'):
                child = parent.find(tag)
                if child is None: child = ET.SubElement(parent, tag)
                parent = child
            for tag, value in zip(('mu', 'mu2'), values):
                child = parent.find(tag)
                if child is None: child = ET.SubElement(parent, tag)
                child.text = str(value)
            records.append({'model': model.get('name'), 'collision': name,
                            'surface': surface_name, 'mu': values[0], 'mu2': values[1]})
    return records
