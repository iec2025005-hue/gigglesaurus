"""Regression tests for implementation gaps found in the V2 Phase-0 audit (reports/audit_phase0.md)."""
from pathlib import Path
import xml.etree.ElementTree as ET
import pytest
import yaml
from morph_sim.assets import enrich_sdf
from morph_sim.control import LEGS

P = Path(__file__).resolve().parents[1]


def minimal_model():
    root = ET.fromstring('<sdf version="1.9"><model name="morph"><link name="base_link"><collision name="underbody_collision"/><visual name="base"/></link>'
                         + ''.join(f'<link name="{n}_wheel"><collision name="{n}_wheel_collision"/><visual name="{n}_tire"/></link>' for n in LEGS)
                         + '</model></sdf>')
    ET.SubElement(root.find('model'), 'frame', name='camera_link', attached_to='base_link')
    return root


def test_texture_paths_survive_a_workspace_path_with_spaces():
    # Path.as_uri() produced file:///...%20... which the renderer could not open (60 errors per launch).
    root = enrich_sdf(minimal_model(), Path('/tmp/space dir/share'))
    textures = [t.text for t in root.findall('.//pbr/metal/albedo_map')]
    assert textures and all('%20' not in t and 'space dir' in t for t in textures)


@pytest.mark.parametrize('bridge,world', [('bridge.yaml', 'multisector.sdf'), ('bridge_competition.yaml', 'competition.sdf')])
def test_contact_bridges_use_the_scoped_sensor_topic_of_their_world(bridge, world):
    # The gz contact system ignores <sensor><topic>; it publishes on the world-scoped sensor topic.
    name = ET.parse(P / 'worlds' / world).find('./world').attrib['name']
    entries = {e['ros_topic_name']: e['gz_topic_name'] for e in yaml.safe_load((P / 'config' / bridge).read_text())}
    for sensor, link in [('fl', 'fl_wheel'), ('fr', 'fr_wheel'), ('rl', 'rl_wheel'), ('rr', 'rr_wheel'), ('underbody', 'base_link')]:
        assert entries['/morph/contacts/' + sensor] == f'/world/{name}/model/morph/link/{link}/sensor/{sensor}_contact/contact'


def test_assumption_summary_matches_authoritative_parameters():
    summary = yaml.safe_load((P / 'config/model_assumptions.yaml').read_text()); params = yaml.safe_load((P / 'config/params.yaml').read_text())
    assert summary['wheel_radius_m'] == params['robot']['wheel_radius_m'] and summary['wheel_width_m'] == params['robot']['wheel_width_m']
    assert summary['nominal_mass_kg'] == params['robot']['mass_kg']
    assert summary['spring_default_N_per_m'] == params['suspension']['stiffness_N_m'] and summary['damper_default_Ns_per_m'] == params['suspension']['damping_Ns_m']
