"""V2 commit 3: decision core - stability margin, cost terms, stance choice, predictive trigger, explanations."""
import json, math
from pathlib import Path
import pytest
from morph_sim import decision_core as dc
from morph_sim.parameters import resolve_profile
from morph_sim.trench_crossing import best_yaw

P = Path(__file__).resolve().parents[1]
PARAMS = resolve_profile(P)[0]
ARENA = json.loads((P / 'config/tetra_arena.json').read_text())
ZONES = {z['id']: z for z in ARENA['zones']}
CONTEXT = dict(mass_kg=PARAMS['robot']['mass_kg'], drive=PARAMS['drive'], current_d=0., winding_C=22.)


def test_static_margins_match_the_audit_and_respond_to_tilt():
    flat = {name: dc.stability_margins(dc.wheelbase(name)) for name in dc.SHAPES}
    assert flat['COMPACT']['front'] == pytest.approx(22.1, abs=.2) and flat['EXTENDED']['front'] == pytest.approx(49.1, abs=.2)
    assert flat['NORMAL']['left'] == pytest.approx(42.9, abs=.1) == flat['NORMAL']['right']
    nose_down = dc.stability_margins(.672, pitch=math.radians(10.)); nose_up = dc.stability_margins(.672, pitch=math.radians(-10.))
    assert nose_down['front'] == pytest.approx(flat['EXTENDED']['front'] - 10., abs=.01) and nose_up['rear'] == pytest.approx(flat['EXTENDED']['rear'] - 10., abs=.01)
    rolled = dc.stability_margins(.54, roll=math.radians(15.))
    assert min(rolled['left'], rolled['right']) == pytest.approx(42.9 - 15., abs=.1) and dc.stability_margin(.237, pitch=math.radians(-23.)) < 0


def test_gravity_vector_convention():
    assert dc.gravity_in_body(0., 0.) == pytest.approx((0., 0., -1.))
    assert dc.gravity_in_body(0., .2)[0] > 0            # nose down: gravity pulls towards the front of the body
    assert math.hypot(*dc.gravity_in_body(.3, -.4)[:2]) < 1


def test_axle_loads_shift_to_the_rear_on_a_climb_and_more_for_a_short_wheelbase():
    weight = PARAMS['robot']['mass_kg'] * dc.G
    for shape in dc.SHAPES:
        front, rear = dc.axle_loads(dc.wheelbase(shape), 15., PARAMS['robot']['mass_kg'])
        assert front + rear == pytest.approx(weight * math.cos(math.radians(15.))) and rear > front
    compact = dc.axle_loads(.237, 15., 10.); extended = dc.axle_loads(.672, 15., 10.)
    assert compact[1] / sum(compact) == pytest.approx(.83, abs=.01) and extended[1] / sum(extended) == pytest.approx(.616, abs=.01)


def test_ramp_choice_matches_the_measured_outcomes():
    ramp = dict(kind='ramp', id='ramp', slope_deg=16., mu=.7225)
    best, results = dc.choose(ramp, CONTEXT)
    assert best is None and all(not r['feasible'] for r in results)                 # 16 deg needs more than 4 x 0.5 Nm
    best, results = dc.choose(ramp, dict(CONTEXT, torque_boost=True))
    assert best['shape'] == 'EXTENDED' and not next(r for r in results if r['shape'] == 'COMPACT')['feasible']
    assert dc.choose(dict(ramp, slope_deg=22.), dict(CONTEXT, torque_boost=True))[0] is None   # out of reach, as measured and calculated
    # Compact fails on 15 deg through grip on the unloaded front axle, not through torque (measured: stalls at the foot).
    compact = dc.traction_utilisation(.237, 15., 10., .7225, .5, 2.2); extended = dc.traction_utilisation(.672, 15., 10., .7225, .5, 2.2)
    assert compact > extended and compact > 1.


def test_pinch_needs_compact_and_trenches_reject_it():
    best, results = dc.choose(dict(ZONES['pinch'], mu=.765), CONTEXT)
    assert best['shape'] == 'COMPACT' and [r['shape'] for r in results if not r['feasible']] == ['NORMAL', 'EXTENDED']
    assert best['morph_time_s'] == pytest.approx(.0379 / dc.STAGE_RATE)
    best, results = dc.choose(dict(ZONES['trenches'], mu=.765), CONTEXT)
    assert best['shape'] == 'EXTENDED' and not next(r for r in results if r['shape'] == 'COMPACT')['feasible']
    assert dc.choose(dict(ZONES['corridor'], mu=.765), CONTEXT)[0]['shape'] == 'NORMAL'         # kerbs: tyres fit, no change needed
    assert dc.choose(dict(kind='flat', id='road', mu=.765), CONTEXT)[0]['shape'] == 'NORMAL'


def test_envelope_constants_agree_with_the_stepping_geometry():
    for name, d in dc.SHAPES.items(): assert dc.STEP_STAGGER[name] == pytest.approx(best_yaw(d)[1], abs=.002)
    assert dc.CRAB_LENGTH['COMPACT'] < ZONES['pinch']['clear_width_m'] < dc.CRAB_LENGTH['NORMAL']
    assert dc.TYRE_WIDTH < ZONES['corridor']['clear_width_m'] < dc.BODY_WIDTH


def test_predictive_trigger_fires_one_margin_before_the_change_must_start():
    t_morph = dc.morph_time(0., 'COMPACT'); assert t_morph == pytest.approx(11.84, abs=.02)
    assert dc.T_FULL == pytest.approx(17.0, abs=.1)                                   # measured 16.7 s
    distance = dc.trigger_distance(.3, t_morph); assert distance == pytest.approx(.3 * (t_morph + 1.))
    assert not dc.lookahead_trigger(distance + .05, .3, t_morph) and dc.lookahead_trigger(distance - .05, .3, t_morph)
    assert not dc.lookahead_trigger(.1, .3, 0.)                                       # nothing to change
    assert not dc.lookahead_trigger(.5, 0., t_morph) and dc.lookahead_trigger(.2, 0., t_morph)    # at rest only when the zone is within reach of a crawl


def test_thermal_term_is_small_and_grows_with_force():
    drive = PARAMS['drive']
    assert dc.winding_rise(4 * .5 / .075, drive) == pytest.approx((.5 / .7) ** 2 * 3.2 * 2.5, rel=1e-6)      # 4.1 K at the torque limit
    assert dc.winding_rise(10., drive) < dc.winding_rise(20., drive) < 5.


def test_explanations_carry_the_deciding_numbers():
    pinch = dict(ZONES['pinch'], mu=.765, label='pinch zone 380mm'); best, results = dc.choose(pinch, CONTEXT)
    text = dc.explain(pinch, best, results, distance=4.2, speed=.3)
    assert text.startswith('[PINCH ZONE 380MM 4.2 m AHEAD]') and 'COMPACT (237 mm)' in text and '330 mm long' in text and 'rejected NORMAL (clearance), EXTENDED (clearance)' in text
    assert 'start within' in text
    none = dc.explain(dict(kind='ramp', id='ramp 22', slope_deg=22., mu=.7), *dc.choose(dict(kind='ramp', slope_deg=22., mu=.7), CONTEXT))
    assert 'no stance can pass' in none
    vector = dc.cost_vector(best); assert len(vector) == len(dc.COST_FIELDS) == 8 and vector[0] == pytest.approx(best['total'])
