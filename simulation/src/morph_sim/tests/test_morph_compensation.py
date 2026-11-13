"""Phase-0 fixes: measured-rate morph feedforward, wheel-angle lock and measured speed gate.

Found by the V2 baseline (reports/baseline.md): the position servo moves the stages at about
0.0032 m/s although 0.0045 m/s is commanded, the soft wheel speed loop did not follow the
feedforward, and the body crept up to 0.17 m during a standstill morph.
"""
import math
import pytest
from morph_sim.control import (AdaptationCore, STAGES, ARMS, LEGS, D_MIN, D_MAX, MORPH_SPEED_GATE, MORPH_STAGE_RATE,
                               TRANSITION_SPEED, WHEEL_LOCK_LIMIT, MORPH_DECAY_S, FLIP_MARGIN, STALL_TIME, FEEDFORWARD_FLOOR, JAM_TIME, optimized_wheel)

RADIUS = .075


def states():
    return {n: 0. for n in STAGES + ARMS + tuple(x + y for x in LEGS for y in ('_steer_joint', '_height_joint', '_wheel_joint'))}


def velocities(stage_rate=0., wheel_rates=(0., 0., 0., 0.)):
    v = {n: stage_rate for n in STAGES}
    v.update({leg + '_wheel_joint': w for leg, w in zip(LEGS, wheel_rates)})
    return v


def settle(core, s, v, steps=40, command=(0., 0., 0.)):
    out = None
    for _ in range(steps):
        out = core.step(.01, list(command), s, velocities=v)
    return out


def test_feedforward_follows_measured_stage_rate_not_commanded_rate():
    core = AdaptationCore(RADIUS); s = states(); core.event('expand')
    actual = .0032                                    # measured in simulation; commanded is 0.0045
    out = settle(core, s, velocities(actual), steps=60)
    assert out.status['morph_rate_m_s'] == pytest.approx(MORPH_STAGE_RATE)
    assert out.status['stage_rate_measured_m_s'] == pytest.approx(actual, rel=.02)
    # Wheel reference speed is the module speed 4*d_dot over the radius (front +, rear -); the
    # wheel-angle lock adds a bounded correction because these fake wheels never turn.
    assert core.wheel_reference[0] == pytest.approx(sum([4 * actual / RADIUS * .01] * 60), rel=.25)
    assert out.wheels[0] > 0 > out.wheels[2]


def test_without_velocity_feedback_the_legacy_commanded_feedforward_is_kept():
    core = AdaptationCore(RADIUS); s = states(); core.event('expand')
    for _ in range(60): out = core.step(.01, [0., 0., 0.], s)
    assert out.wheels[0] == pytest.approx(4 * MORPH_STAGE_RATE / RADIUS, rel=1e-6)
    assert out.wheels[2] == pytest.approx(-4 * MORPH_STAGE_RATE / RADIUS, rel=1e-6)
    assert out.status['stage_rate_measured_m_s'] is None and not out.status['wheel_angle_lock']


def test_no_residual_wheel_velocity_after_stages_stop():
    core = AdaptationCore(RADIUS); s = states(); core.event('expand')
    settle(core, s, velocities(.0032), steps=60)
    for name in STAGES: s[name] = D_MAX              # morph complete, stages at rest
    out = settle(core, s, velocities(0.), steps=60)
    assert out.wheels == [0.] * 4 and not out.status['wheel_angle_lock']
    assert out.status['stage_rate_measured_m_s'] == pytest.approx(0., abs=1e-4)


def test_wheel_angle_lock_pulls_a_lagging_wheel_and_is_bounded():
    core = AdaptationCore(RADIUS); s = states(); core.event('expand'); v = velocities(.0032)
    corrections = []
    for _ in range(400):                              # wheels are held still: tracking error grows
        out = core.step(.01, [0., 0., 0.], s, velocities=v)
        corrections.append(out.wheels[0] - 4 * out.status['stage_rate_measured_m_s'] / RADIUS)
    assert corrections[-1] > corrections[20] > 0      # correction grows with the angle error
    assert max(corrections) <= WHEEL_LOCK_LIMIT + 1e-9
    # A wheel that follows its reference exactly needs no correction.
    core = AdaptationCore(RADIUS); s = states(); core.event('expand')
    for _ in range(200):
        out = core.step(.01, [0., 0., 0.], s, velocities=v)
        for i, leg in enumerate(LEGS): s[leg + '_wheel_joint'] = core.wheel_reference[i]
    assert out.wheels[0] == pytest.approx(4 * .0032 / RADIUS, rel=.02)


def test_morph_gate_uses_measured_body_speed():
    core = AdaptationCore(RADIUS); s = states(); core.event('expand')
    fast = (MORPH_SPEED_GATE + .03) / RADIUS          # all four wheels rolling forward above the gate
    out = settle(core, s, velocities(0., [fast] * 4))
    assert out.status['body_speed_estimate'] == pytest.approx(MORPH_SPEED_GATE + .03)
    assert out.status['morph_rate_m_s'] == 0 and out.status['transition_pending']
    slow = (MORPH_SPEED_GATE - .03) / RADIUS
    out = settle(core, s, velocities(0., [slow] * 4))
    assert out.status['morph_rate_m_s'] == pytest.approx(MORPH_STAGE_RATE)


def test_body_speed_estimate_ignores_spin_and_stage_motion():
    core = AdaptationCore(RADIUS); s = states()
    # Pure in-place spin: wheels tangential, equal and opposite contributions cancel in the mean.
    angle = math.atan2(.27, .27)
    for leg, a in zip(LEGS, (-angle, angle, angle, -angle)): s[leg + '_steer_joint'] = a
    out = core.step(.01, [0., 0., 0.], s, velocities=velocities(0., (-3., 3., -3., 3.)))
    assert out.status['body_speed_estimate'] == pytest.approx(0., abs=1e-9)
    # Stage motion at standstill: front wheels forward, rear wheels backward.
    s = states(); out = core.step(.01, [0., 0., 0.], s, velocities=velocities(.0032, (.2, .2, -.2, -.2)))
    assert out.status['body_speed_estimate'] == pytest.approx(0., abs=1e-9)


def test_pending_transition_bounds_the_planar_speed_vector():
    core = AdaptationCore(RADIUS); s = states(); core.event('grass'); core.event('expand')
    out = settle(core, s, velocities(0.), steps=300, command=(.36, .36, 0.))
    assert out.status['transition_pending']
    assert out.status['command_speed'] <= TRANSITION_SPEED + 1e-9
    assert core.v[0] == pytest.approx(core.v[1])     # direction of travel is preserved


def test_feedforward_envelope_blocks_stage_noise_when_no_morph_is_commanded():
    # Regression: measured stage-velocity noise during crab travel flipped steering targets by 180 deg.
    core = AdaptationCore(RADIUS); s = states()
    for leg in LEGS: s[leg + '_steer_joint'] = math.pi / 2
    core.steering = [math.pi / 2] * 4
    for k in range(300):
        noise = .0006 * (1 if k % 2 else -1)
        out = core.step(.01, [0., .3, 0.], s, velocities=velocities(noise))
        assert all(a == pytest.approx(math.pi / 2, abs=1e-6) for a in out.steering)
    assert all(w > 0 for w in out.wheels)


def test_feedforward_decays_over_the_envelope_after_the_command_ends():
    core = AdaptationCore(RADIUS); s = states(); core.event('expand')
    settle(core, s, velocities(.0032), steps=60)
    for name in STAGES: s[name] = D_MAX              # target reached: stage command ends
    v = velocities(.0032)                            # pretend the stages are still coasting
    first = core.step(.01, [0., 0., 0.], s, velocities=v); assert 0 < core.morph_envelope < 1
    for _ in range(int(MORPH_DECAY_S / .01) + 2): out = core.step(.01, [0., 0., 0.], s, velocities=v)
    assert core.morph_envelope == 0 and not out.status['wheel_angle_lock']


@pytest.mark.parametrize('previous,vx,vy,expected', [
    (math.pi / 2, -.01, .15, math.pi / 2),           # crab +y with a small backward component: stay at +90
    (-math.pi / 2, -.01, -.15, -math.pi / 2),
    (math.pi / 2, -.10, .15, None),                  # far past the margin: canonical re-steer
    (0., -.01, .15, None)])                          # wheel not near that limit: canonical
def test_flip_hysteresis_at_the_steering_limit(previous, vx, vy, expected):
    canonical = optimized_wheel(vx, vy, RADIUS)
    theta, speed = optimized_wheel(vx, vy, RADIUS, previous)
    if expected is None:
        assert (theta, speed) == canonical
    else:
        assert theta == pytest.approx(expected) and speed == pytest.approx(-canonical[1])
        assert abs(theta - previous) < FLIP_MARGIN + 1e-9 and speed > 0


def test_completion_latch_survives_servo_rest_error_and_releases_on_a_new_target():
    core = AdaptationCore(RADIUS); s = states(); core.event('compact')
    for name in STAGES: s[name] = D_MIN + .0004      # inside SETTLE_IN: latches
    out = settle(core, s, velocities(0.), steps=5); assert not out.status['transition_pending'] and out.status['transition_settled']
    for name in STAGES: s[name] = D_MIN + .0008      # servo relaxes past SETTLE_IN but inside SETTLE_OUT
    out = settle(core, s, velocities(0.), steps=5, command=(.6, 0., 0.)); assert not out.status['transition_pending']
    core.event('expand')
    out = core.step(.01, [0., 0., 0.], s, velocities=velocities(0.)); assert out.status['transition_pending'] and not out.status['transition_settled']


def test_stalled_stage_inside_the_outer_band_counts_as_complete():
    core = AdaptationCore(RADIUS); s = states(); core.event('compact')
    for name in STAGES: s[name] = D_MIN + .0009      # never reaches SETTLE_IN
    out = settle(core, s, velocities(0.), steps=20); assert out.status['transition_pending']
    out = settle(core, s, velocities(0.), steps=int(STALL_TIME / .01) + 5); assert not out.status['transition_pending']


def test_wheels_pointing_sideways_are_steered_before_a_standstill_shape_change():
    # Regression: with a measured-only feedforward the wheels stayed at 90 deg and blocked the stages.
    core = AdaptationCore(RADIUS); s = states()
    for leg in LEGS: s[leg + '_steer_joint'] = math.pi / 2
    core.steering = [math.pi / 2] * 4; core.event('expand')
    out = core.step(.01, [0., 0., 0.], s, velocities=velocities(0.))
    assert out.status['morph_rate_m_s'] == 0                     # stages wait for alignment
    assert all(a < math.pi / 2 for a in out.steering)            # wheels start turning towards 0
    assert out.wheels == [0.] * 4                                # and do not roll while they wait
    for leg in LEGS: s[leg + '_steer_joint'] = 0.
    core.steering = [0.] * 4
    out = settle(core, s, velocities(0.), steps=5)
    assert out.status['morph_rate_m_s'] == pytest.approx(MORPH_STAGE_RATE)
    assert out.wheels[0] > 0 > out.wheels[2]                     # led by the commanded-direction floor


def test_crab_travel_with_a_shape_change_steers_front_and_rear_apart():
    core = AdaptationCore(RADIUS); s = states(); core.event('expand')
    for leg in LEGS: s[leg + '_steer_joint'] = math.pi / 2
    core.steering = [math.pi / 2] * 4; core.v = [0., .15, 0.]
    core.step(.01, [0., .15, 0.], s, velocities=velocities(0., [2.] * 4))
    front = math.atan2(.15, 4 * FEEDFORWARD_FLOOR)
    targets_differ = core.steering[0] < math.pi / 2 and core.steering[2] < math.pi / 2
    assert targets_differ and front == pytest.approx(math.pi / 2 - 4 * FEEDFORWARD_FLOOR / .15, abs=1e-3)


def test_feedforward_floor_cannot_run_away_once_the_stage_rests_near_the_target():
    core = AdaptationCore(RADIUS); s = states(); core.event('compact')
    for name in STAGES: s[name] = D_MIN + .0003      # servo rest error: above the 0.1 mm command threshold
    for _ in range(int(STALL_TIME / .01) + 30): out = core.step(.01, [0., 0., 0.], s, velocities=velocities(0.))
    assert out.status['morph_rate_m_s'] == 0 and not out.status['wheel_angle_lock']
    assert out.wheels == [0.] * 4 and not out.status['transition_pending']


def test_jammed_stage_raises_morph_stalled_and_stops_leading_the_wheels():
    core = AdaptationCore(RADIUS); s = states(); core.event('expand')   # stages never move
    for _ in range(int(JAM_TIME / .01) + 30): out = core.step(.01, [0., 0., 0.], s, velocities=velocities(0.))
    assert out.status['morph_stalled'] and out.status['transition_pending']
    for _ in range(100): out = core.step(.01, [0., 0., 0.], s, velocities=velocities(0.))
    assert out.wheels == [0.] * 4                    # no floor feedforward while the fault is set
    out = settle(core, s, velocities(.0032), steps=20)
    assert not out.status['morph_stalled']           # clears when the stages move again
