"""Portable regressions for command guards and camera acceptance."""
import json
import math
from pathlib import Path
import sys
import pytest
from morph_sim.control import AdaptationCore
from morph_sim.safety import ACTUATED_JOINTS, decode_perception, fresh_snapshot
from morph_sim.timing import decode_command

sys.path.insert(0, str(Path(__file__).parents[1] / 'tools'))
from camera_acceptance import camera_acceptance


@pytest.mark.parametrize('payload', ['null', '[]', '3', 'true', '"text"', '{}',
    '{"values":3}', '{"values":[true,0,0]}'])
def test_malformed_timed_commands_raise_validation_error(payload):
    with pytest.raises(ValueError):
        decode_command(payload, 10**9, 10**9)


def test_boolean_velocity_is_not_accepted_as_numeric_motion():
    import socket
    payload = {'values': [True, 0, 0], 'source_id': 'test', 'sequence': 1,
               'source_sim_ns': 10**9, 'source_wall_ns': 10**9, 'host': socket.gethostname()}
    with pytest.raises(ValueError):
        decode_command(json.dumps(payload), 10**9, 10**9)


def test_feedback_outage_clears_accumulated_drive_and_recovery_ramps():
    core = AdaptationCore()
    complete = dict.fromkeys(ACTUATED_JOINTS, 0.)
    for _ in range(100):
        core.step(.01, [.2, 0, 0], complete)
    assert max(core.wheels) > 1
    partial = dict(complete)
    partial.pop(ACTUATED_JOINTS[0])
    for _ in range(100):
        out = core.step(.01, [.2, 0, 0], partial)
        assert out.wheels == [0.] * 4
        assert core.v == [0.] * 3
    assert max(core.step(.01, [.2, 0, 0], complete).wheels) <= .2


@pytest.mark.parametrize('index', range(4))
@pytest.mark.parametrize('value', [math.nan, math.inf, -math.inf])
def test_nonfinite_snapshot_clocks_fail_closed(index, value):
    args = [1., 100., 1.01, 100.01]
    args[index] = value
    assert not fresh_snapshot(*args)


def test_perception_uses_source_time_and_rejects_delayed_or_invalid_decisions():
    decision = {'valid': True, 'mode': 'track', 'speed_limit_m_s': .1, 'sim_time_s': 1.}
    assert decode_perception(json.dumps(decision), 1.1)['sim_time_s'] == 1.
    for now in (1.6, .9):
        with pytest.raises(ValueError):
            decode_perception(json.dumps(decision), now)
    for payload in ('null', '[]', '{}', json.dumps(dict(decision, speed_limit_m_s=-1))):
        with pytest.raises(ValueError):
            decode_perception(payload, 1.1)


def camera_fixture():
    manifest = {'launch': {'camera_enabled': 'true', 'camera_rate': '30', 'camera_width': '640'}}
    samples = [[i * 10**9 // 30, i * 10**9 // 30] for i in range(300)]
    checks = [{'passed': True, 'image': {'width': 640, 'height': 360}} for _ in samples]
    return manifest, {'camera': samples, 'camera_info': samples}, checks


def test_hq_requires_both_live_streams_and_full_interval_rate():
    manifest, samples, checks = camera_fixture()
    interval = ((0, 0), (10**10, 10**10))
    assert camera_acceptance(manifest, samples, checks, *interval)['passed']
    assert not camera_acceptance(manifest, {}, [], *interval)['passed']
    assert not camera_acceptance(manifest, {'camera': samples['camera']}, checks, *interval)['passed']
    # First two seconds at 30 Hz followed by eight silent seconds must fail.
    silent = {key: rows[:60] for key, rows in samples.items()}
    assert not camera_acceptance(manifest, silent, checks[:60], *interval)['passed']
    assert not camera_acceptance(manifest, samples, checks[:1], *interval)['passed']
    checks[0]['image']['width'] = 320
    assert not camera_acceptance(manifest, samples, checks, *interval)['passed']


def test_disabled_camera_is_not_claimed_as_passing_camera_evidence():
    result = camera_acceptance({'launch': {'camera_enabled': 'false'}}, {}, [], (0, 0), (1, 1))
    assert not result['required']
    assert result['passed'] is None
