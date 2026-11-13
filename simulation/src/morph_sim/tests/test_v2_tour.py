"""V2 commit 3: tour state machine against a kinematic stand-in for the robot (no simulator)."""
import json, math
from pathlib import Path
import pytest
from morph_sim import decision_core as dc
from morph_sim.parameters import resolve_profile
from morph_sim.tour_core import Tour, build_legs, POLICIES, wrap

P = Path(__file__).resolve().parents[1]
PARAMS = resolve_profile(P)[0]
ARENA = json.loads((P / 'config/tetra_arena.json').read_text())
EVENT_SHAPE = {'compact': 'COMPACT', 'normal': 'NORMAL', 'expand': 'EXTENDED'}


def run(policy, limit_s=900., dt=.05, blocked=None):
    """Integrate the tour's commands. Rails move at the measured rate; `blocked(leg, shape, x)` -> True holds the robot still."""
    tour = Tour(ARENA, PARAMS, policy); x, y, yaw, d = 0., 0., 0., 0.; target = 0.; t = 0.; events = []; pitch = 0.; speed = 0.; yaw_rate = 0.; skipped = []
    while t < limit_s and not tour.finished:
        state = dict(x=x, y=y, yaw=yaw, speed=speed, yaw_rate=yaw_rate, d=d, pitch_deg=pitch, settled=abs(target - d) < 5e-4, winding_C=25.)
        out = tour.step(t, state)
        for name in out['events']:
            events.append((round(t, 2), round(x, 2), name))
            if name in EVENT_SHAPE: target = dc.SHAPES[EVENT_SHAPE[name]]
        if out['skip_to'] is not None: skipped.append(out['leg']); x, y, yaw = out['skip_to'], 0., 0.
        vx, vy, wz = out['command']; leg = out['leg']
        if blocked and blocked(leg, out['shape'], x): vx = vy = 0.
        x += (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt; y += (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt; yaw = wrap(yaw + wz * dt)
        speed = math.hypot(vx, vy); yaw_rate = wz; d += max(-dc.STAGE_RATE * dt, min(dc.STAGE_RATE * dt, target - d))
        ramp = next(l for l in tour.legs if l['kind'] == 'ramp'); pitch = -ramp['slope_deg'] if ramp['x0'] + .8 < x < ramp['x_top'] else 0.
        t += dt
    return tour, events, t


def test_route_covers_every_zone_in_order():
    legs = build_legs(ARENA)
    assert [l['id'] for l in legs] == ['start', 'trenches', 'sand', 'mud', 'ramps', 'corridor', 'pinch', 'rocks', 'rubble', 'finish']
    assert all(a['x1'] <= b['x1'] for a, b in zip(legs, legs[1:])) and legs[4]['slope_deg'] == 16. and legs[6]['crab'] and not legs[5]['crab']


def test_proposed_policy_completes_the_tour_and_sequences_the_signature_manoeuvres():
    tour, events, elapsed = run('proposed')
    assert tour.finished and all(r['success'] for r in tour.results) and len(tour.results) == 10
    names = [e[2] for e in events]
    assert names[0] == 'morph_continuous'
    # Predictive: Extended is requested before the trench zone starts (x0 = 3.0 m), while driving.
    first_expand = next(e for e in events if e[2] == 'expand'); assert first_expand[1] < 3.0
    # One wheel at a time: 3 trenches x 4 wheels.
    assert sum(1 for n in names if n.startswith('lift_')) == 12 and set(n for n in names if n.startswith('lift_')) == {'lift_fl', 'lift_fr', 'lift_rl', 'lift_rr'}
    # Ramp: boost and nose-down pitch only while climbing.
    assert 'pitch_nose_down' in names and names.index('boost_on') < names.index('pitch_nose_down')
    # Pinch: Compact is requested ahead of the zone, the crab tuck only after the shape change has finished.
    compact = next(e for e in events if e[2] == 'compact'); tuck = next(e for e in events if e[2] == 'crab_tuck_on'); pinch = next(l for l in tour.legs if l['id'] == 'pinch')
    assert compact[1] < pinch['x0'] and tuck[0] - compact[0] >= dc.morph_time(dc.SHAPES['NORMAL'], 'COMPACT') - 8.      # may start from Extended or Normal
    assert names.index('crab_tuck_on') < names.index('crab_tuck_off')
    text = ' | '.join(entry['text'] for entry in tour.log)
    assert 'PINCH ZONE 380mm' in text and 'sideways crab-walk' in text and 'starting now' in text and 'IMU pitch' in text and 'TRENCH' in text


def test_reactive_policy_stops_to_change_stance_and_takes_longer():
    proposed, _, t_proposed = run('proposed'); reactive, events, t_reactive = run('reactive')
    assert reactive.finished and all(r['success'] for r in reactive.results)
    assert [e[2] for e in events][0] == 'morph_gated' and t_reactive > t_proposed
    stop = next(e for e in events if e[2] == 'expand'); assert stop[1] >= 3.0 - .05           # only at the boundary, not ahead of it


@pytest.mark.parametrize('policy,shape', [('fixed_compact', 'COMPACT'), ('fixed_extended', 'EXTENDED')])
def test_fixed_policies_never_change_stance_or_use_the_actuators(policy, shape):
    def blocked(leg, current, x):                                # the stand-in robot cannot enter fixtures its stance does not fit
        pinch = next(z for z in ARENA['zones'] if z['id'] == 'pinch')
        return leg == 'pinch' and current != 'COMPACT' and x >= pinch['bounds'][0] - .45
    tour, events, _ = run(policy, blocked=blocked)
    names = [e[2] for e in events]
    assert tour.finished and names.count({'COMPACT': 'compact', 'EXTENDED': 'expand'}[shape]) == 1 and not ({'compact', 'normal', 'expand'} - {{'COMPACT': 'compact', 'EXTENDED': 'expand'}[shape]}) & set(names)
    assert not any(n.startswith(('lift_', 'pitch_', 'boost_on')) for n in names)
    pinch = next(r for r in tour.results if r['zone'] == 'pinch')
    assert pinch['success'] == (shape == 'COMPACT')
    if shape == 'EXTENDED': assert pinch['reason'] in ('stalled', 'timeout') and 'FAILED' in ' '.join(e['text'] for e in tour.log)


def test_unknown_policy_is_rejected():
    with pytest.raises(ValueError): Tour(ARENA, PARAMS, 'ackermann')
    assert POLICIES == ('fixed_compact', 'fixed_extended', 'reactive', 'proposed')
