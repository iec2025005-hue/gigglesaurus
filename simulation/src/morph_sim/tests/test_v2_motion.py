"""V2 commit 2: coupled swerve + variable-wheelbase kinematics, on-the-fly morphing, rail holding,
corner-height suspension commands and the thermally governed torque boost."""
import math
from pathlib import Path
import pytest
from morph_sim.control import (MOTION, STEER_RATE, jerk_limited_step, friction_acceleration_limit, AdaptationCore, STAGES, ARMS, LEGS, D_MIN, D_MAX, MORPH_STAGE_RATE, MORPH_SPEED_GATE, TRANSITION_SPEED,
                               SETTLE_OUT, STALL_TIME, HEIGHT_LIMIT, SUSPENSION_PRESETS, wheel_position, wheel_velocity)
from morph_sim.design_physics import PowerPlant, load_parameters

P = Path(__file__).resolve().parents[1]
RADIUS = .075


def states(d=0.):
    s = {n: 0. for n in ARMS + tuple(x + y for x in LEGS for y in ('_steer_joint', '_height_joint', '_wheel_joint'))}
    s.update({n: d for n in STAGES}); return s


def velocities(stage_rate=0., wheel_rates=(0., 0., 0., 0.)):
    v = {n: stage_rate for n in STAGES}; v.update({leg + '_wheel_joint': w for leg, w in zip(LEGS, wheel_rates)}); return v


# --- kinematics --------------------------------------------------------------------------------
@pytest.mark.parametrize('d', [D_MIN, 0., D_MAX])
def test_wheel_positions_give_wheelbase_and_fixed_track(d):
    points = [wheel_position(i, d) for i in range(4)]
    assert points[0][0] - points[2][0] == pytest.approx(.54 + 8 * d) and points[0][1] - points[1][1] == pytest.approx(.54)
    assert sum(p[0] for p in points) == pytest.approx(0.) and sum(p[1] for p in points) == pytest.approx(0.)


@pytest.mark.parametrize('twist,d,rate', [((.3, 0., 0.), 0., .0032), ((.2, .1, .4), -.02, -.0045), ((0., 0., .7), .01, .006), ((-.3, .2, -.3), D_MIN, 0.)])
def test_wheel_velocity_is_the_time_derivative_of_the_moving_contact_point(twist, d, rate):
    # v_i = v + omega x r_i(L) + dr_i/dL * L_dot, checked against a finite difference of r_i.
    h = 1e-6
    for i in range(4):
        x, y = wheel_position(i, d); x2, y2 = wheel_position(i, d + rate * h)
        expected = (twist[0] - twist[2] * y + (x2 - x) / h, twist[1] + twist[2] * x + (y2 - y) / h)
        assert wheel_velocity(i, twist, d, rate) == pytest.approx(expected, abs=1e-6)


def test_straight_driving_with_moving_rails_only_changes_wheel_speeds_not_steering():
    core = AdaptationCore(RADIUS); core.event('morph_continuous'); core.event('expand'); s = states(); core.v = [.3, 0., 0.]
    for _ in range(120):
        out = core.step(.01, [.3, 0., 0.], s, velocities=velocities(.0032, [4.] * 4))
        for i, leg in enumerate(LEGS):                           # wheels that follow their angle reference exactly
            if core.wheel_reference[i] is not None: s[leg + '_wheel_joint'] = core.wheel_reference[i]
    assert out.steering == pytest.approx([0.] * 4, abs=1e-9)
    # front wheels run 4*d_dot faster than the body, rear wheels 4*d_dot slower
    assert out.wheels[0] == pytest.approx((.3 + 4 * .0032) / RADIUS, rel=.01) and out.wheels[2] == pytest.approx((.3 - 4 * .0032) / RADIUS, rel=.01)


# --- on-the-fly morphing policy ------------------------------------------------------------------
def test_gated_policy_is_the_default_and_keeps_the_low_speed_gate():
    core = AdaptationCore(RADIUS); core.event('expand'); s = states(); assert core.morph_policy == 'gated'
    fast = [.5 / RADIUS] * 4
    for _ in range(100): out = core.step(.01, [.5, 0., 0.], s, velocities=velocities(0., fast))
    assert out.status['transition_pending'] and out.status['command_speed'] <= TRANSITION_SPEED + 1e-9
    assert out.status['morph_rate_m_s'] == 0                     # encoder speed is above the 0.22 m/s gate


def test_continuous_policy_morphs_at_driving_speed_without_slowing_down():
    core = AdaptationCore(RADIUS); core.event('morph_continuous'); core.event('expand'); s = states()
    fast = [.5 / RADIUS] * 4
    for _ in range(100): out = core.step(.01, [.5, 0., 0.], s, velocities=velocities(.0032, fast))
    assert out.status['command_speed'] == pytest.approx(.5) and .5 > MORPH_SPEED_GATE
    assert out.status['morph_rate_m_s'] == pytest.approx(MORPH_STAGE_RATE) and out.status['morph_policy'] == 'continuous'
    assert out.shape[0] > 0                                      # the rails are being commanded outward


# --- rail holding --------------------------------------------------------------------------------
def test_rails_hold_their_commanded_length_instead_of_following_a_disturbance():
    core = AdaptationCore(RADIUS); core.event('expand'); s = states(D_MAX)
    for _ in range(int(STALL_TIME / .01) + 50): out = core.step(.01, [0., 0., 0.], s, velocities=velocities(0.))
    assert not out.status['transition_pending']
    pushed = D_MAX - .010                                        # load pushes every stage in by 10 mm
    out = core.step(.01, [0., 0., 0.], states(pushed), velocities=velocities(-.002))
    assert out.shape[0] == pytest.approx(D_MAX)                  # the command stays at the held length: full restoring effort
    assert out.status['transition_pending']                      # and the completion latch has released
    for _ in range(20): out = core.step(.01, [0., 0., 0.], states(pushed), velocities=velocities(0.))
    assert out.status['morph_rate_m_s'] == pytest.approx(MORPH_STAGE_RATE)   # actively driving back out


def test_rails_keep_holding_when_the_speed_gate_blocks_a_shape_change():
    core = AdaptationCore(RADIUS); s = states(0.)
    for _ in range(30): core.step(.01, [0., 0., 0.], s, velocities=velocities(0.))
    fast = [.5 / RADIUS] * 4; core.v = [.5, 0., 0.]
    out = core.step(.01, [.5, 0., 0.], states(-.006), velocities=velocities(0., fast))   # pushed in while driving fast
    assert out.status['morph_rate_m_s'] == 0 and out.shape[0] == pytest.approx(0.)        # still commanding the held length


# --- suspension ----------------------------------------------------------------------------------
def test_suspension_presets_and_release():
    core = AdaptationCore(RADIUS); s = states()
    core.event('pitch_nose_up'); out = core.step(.01, [0., 0., 0.], s)
    assert out.heights == [HEIGHT_LIMIT, HEIGHT_LIMIT, -HEIGHT_LIMIT, -HEIGHT_LIMIT] and out.status['suspension_override']
    core.event('pitch_nose_down'); assert core.step(.01, [0., 0., 0.], s).heights == [-HEIGHT_LIMIT, -HEIGHT_LIMIT, HEIGHT_LIMIT, HEIGHT_LIMIT]
    core.event('lift_rr'); assert core.step(.01, [0., 0., 0.], s).heights == [0., HEIGHT_LIMIT, HEIGHT_LIMIT, -HEIGHT_LIMIT]   # leans towards fl
    core.event('suspension_level'); out = core.step(.01, [0., 0., 0.], s)
    assert out.heights == [pytest.approx(-.008)] * 4 and not out.status['suspension_override']      # back to the track-mode height
    assert all(len(v) == 4 and max(abs(x) for x in v) == HEIGHT_LIMIT for v in SUSPENSION_PRESETS.values())
    for i, leg in enumerate(LEGS):                               # lifted corner down, opposite corner at mid travel, neighbours up
        preset = SUSPENSION_PRESETS['lift_' + leg]; assert preset[i] == -HEIGHT_LIMIT and preset[3 - i] == 0. and sorted(preset)[2:] == [HEIGHT_LIMIT] * 2


def test_corner_targets_are_clamped_to_actuator_travel_and_estop_still_wins():
    core = AdaptationCore(RADIUS); s = states(); core.corner_target = [.05, -.05, .01, 0.]
    assert core.step(.01, [0., 0., 0.], s).heights == [HEIGHT_LIMIT, -HEIGHT_LIMIT, .01, 0.]
    s['fl_height_joint'] = .004; core.event('estop')
    assert core.step(.01, [0., 0., 0.], s).heights[0] == pytest.approx(.004)       # held at the measured height


# --- torque boost --------------------------------------------------------------------------------
def test_peak_torque_boost_is_requested_bounded_and_fades_with_temperature():
    params = load_parameters(P / 'config/params.yaml'); plant = PowerPlant(params); d = params['drive']
    assert plant.boost_factor(False) == [1.] * 4
    assert plant.boost_factor(True) == [pytest.approx(1 + d['peak_boost_fraction'])] * 4 and d['peak_boost_fraction'] == .2
    plant.temp = [d['peak_boost_derate_C'], (d['peak_boost_derate_C'] + d['derate_C']) / 2, d['derate_C'], 22.]; plant.driver_temp = [22., 22., 22., d['derate_C'] + 5]
    assert plant.boost_factor(True) == pytest.approx([1.2, 1.1, 1., 1.])


def test_boost_raises_stall_torque_by_twenty_percent_only_when_requested():
    params = load_parameters(P / 'config/params.yaml'); limit = params['drive']['output_torque_limit_Nm']
    def stall(boost):
        plant = PowerPlant(params)
        for _ in range(400): out = plant.step(.005, [8.] * 4, [0.] * 4, boost=boost)       # wheels held still, full speed demanded
        return out
    plain, boosted = stall(False), stall(True)
    assert max(plain['torques_Nm']) == pytest.approx(limit) and max(boosted['torques_Nm']) == pytest.approx(1.2 * limit)
    assert boosted['motor_current_A'][0] > plain['motor_current_A'][0] and boosted['winding_C'][0] > plain['winding_C'][0]


def test_core_reports_the_boost_request_for_the_motor_model():
    core = AdaptationCore(RADIUS); s = states()
    assert not core.step(.01, [0., 0., 0.], s).status['torque_boost']
    core.event('boost_on'); assert core.step(.01, [0., 0., 0.], s).status['torque_boost']
    core.event('boost_off'); assert not core.step(.01, [0., 0., 0.], s).status['torque_boost']


# --- one-wheel-at-a-time trench stepping ---------------------------------------------------------
def test_stepping_geometry_depends_on_wheelbase():
    from morph_sim.trench_crossing import best_yaw, feasible, minimum_stagger, wheel_offsets
    stagger = {name: best_yaw(d)[1] for name, d in (('compact', D_MIN), ('normal', 0.), ('extended', D_MAX))}
    assert stagger['compact'] == pytest.approx(.178, abs=.002) and stagger['normal'] == pytest.approx(.241, abs=.002) and stagger['extended'] == pytest.approx(.284, abs=.002)
    yaw_c, yaw_e = best_yaw(D_MIN)[0], best_yaw(D_MAX)[0]
    assert [feasible(D_MIN, yaw_c, w) for w in (.100, .130, .155)] == [True, False, False]
    assert [feasible(D_MAX, yaw_e, w) for w in (.100, .130, .155)] == [True, True, True]
    assert minimum_stagger(D_MAX, 0.) == pytest.approx(0.)        # head-on: both wheels of an axle arrive together
    assert sorted(wheel_offsets(0., 0.)) == pytest.approx([-.27, -.27, .27, .27])


def test_stepping_plan_lifts_one_wheel_at_a_time_and_flags_conflicts():
    from morph_sim.trench_crossing import plan, wheel_offsets, best_yaw
    yaw = best_yaw(D_MAX)[0]; trenches = [(5., .155)]; offsets = wheel_offsets(D_MAX, yaw); lifted = []
    x = 4.0
    while x < 6.0:
        p = plan(x, D_MAX, yaw, trenches); assert not p['conflict']
        if p['lift'] and (not lifted or lifted[-1] != p['lift']): lifted.append(p['lift'])
        x += .002
    order = [LEGS[i] for i in sorted(range(4), key=lambda i: -offsets[i])]       # leading wheel first
    assert lifted == order
    assert plan(4.0, D_MAX, yaw, trenches) == {'lift': None, 'trench': None, 'inside': False, 'conflict': False}
    x_fr = 5. - offsets[1]                                         # fr centred on the trench
    assert plan(x_fr, D_MAX, yaw, trenches)['lift'] == 'fr' and plan(x_fr, D_MAX, yaw, trenches, done={('fr', 0)})['lift'] != 'fr'
    head_on = plan(5. - .27 - 4 * D_MAX, D_MAX, 0., trenches); assert head_on['conflict']      # both front wheels in the window


# --- S-curve command filter ------------------------------------------------------------------------
def profile(target, steps=400, dt=.01, a_max=1.2, jerk=6.0, start=(0., 0.)):
    v, a = list(start), [0., 0.]; out = []
    for _ in range(steps):
        v, a = jerk_limited_step(v, a, list(target), dt, a_max, jerk); out.append((v[0], v[1]))
    return out


@pytest.mark.parametrize('target,a_max', [((.6, 0.), 1.2), ((.3, .3), 1.2), ((.05, 0.), 1.2), ((.6, 0.), .86), ((-.4, .2), 2.2)])
def test_jerk_and_acceleration_of_the_commanded_velocity_stay_within_limits(target, a_max):
    dt = .01; path = profile(target, a_max=a_max); speeds = [(0., 0.)] + path
    accel = [((b[0] - a[0]) / dt, (b[1] - a[1]) / dt) for a, b in zip(speeds, speeds[1:])]
    jerk = [math.hypot(b[0] - a[0], b[1] - a[1]) / dt for a, b in zip([(0., 0.)] + accel, accel)]
    assert max(math.hypot(*a) for a in accel) <= a_max + 1e-9
    assert max(jerk) <= 6.0 + 1e-6                               # within J_max itself (acceptance allows 5 % more)
    assert path[-1] == pytest.approx(target, abs=1e-9)           # reaches the target exactly, no residual
    along = [p[0] * target[0] + p[1] * target[1] for p in path]
    assert all(b >= a - 1e-12 for a, b in zip(along, along[1:])) and max(along) <= target[0] ** 2 + target[1] ** 2 + 1e-9   # monotonic, no overshoot
    if target[1]: assert all(p[1] * target[0] == pytest.approx(p[0] * target[1], abs=1e-9) for p in path)       # direction of travel preserved


def test_reversal_passes_through_zero_without_exceeding_the_jerk_limit():
    dt = .01; v, a = [.5, 0.], [0., 0.]; previous_a = 0.; worst = 0.; previous_v = .5
    for _ in range(600):
        v, a = jerk_limited_step(v, a, [-.5, 0.], dt, 1.2, 6.0)
        acc = (v[0] - previous_v) / dt; worst = max(worst, abs(acc - previous_a) / dt); previous_a, previous_v = acc, v[0]
    assert v[0] == pytest.approx(-.5) and worst <= 6.0 + 1e-6


def test_core_uses_the_filter_and_estop_is_flagged_as_not_jerk_limited():
    core = AdaptationCore(RADIUS); s = states(); speeds = []
    for _ in range(150): out = core.step(.01, [.6, 0., 0.], s); speeds.append(out.status['command_speed'])
    accel = [(b - a) / .01 for a, b in zip([0.] + speeds, speeds)]; jerk = [(b - a) / .01 for a, b in zip([0.] + accel, accel)]
    assert max(abs(j) for j in jerk) <= MOTION['jerk_limit_m_s3'] + 1e-6 and max(accel) <= MOTION['acceleration_limit_m_s2'] + 1e-9
    assert speeds[-1] == pytest.approx(.6) and out.status['jerk_limited']
    core.event('estop'); out = core.step(.01, [.6, 0., 0.], s)
    assert not out.status['jerk_limited'] and max(out.wheels) < .6 / RADIUS      # wheels are being braked at their own rate limit


def test_acceleration_limit_follows_surface_friction():
    assert friction_acceleration_limit(None) == MOTION['acceleration_limit_m_s2'] == 1.2
    assert friction_acceleration_limit(.35) == pytest.approx(.35 * .35 * 9.81)          # sand: 1.20 m/s^2
    assert friction_acceleration_limit(.25) == pytest.approx(.35 * .25 * 9.81)          # mud: 0.86 m/s^2
    assert friction_acceleration_limit(.765) == MOTION['acceleration_ceiling_m_s2']     # tarmac: torque-limited ceiling
    assert friction_acceleration_limit(.05) == MOTION['acceleration_floor_m_s2'] and MOTION['jerk_limit_m_s3'] == 6.0


# --- steering -------------------------------------------------------------------------------------
def test_drive_is_scaled_by_cos_squared_of_the_steering_error():
    for error in (0., .2, .6, 1.2, math.pi / 2, 2.0):
        core = AdaptationCore(RADIUS); core.v = [.3, 0., 0.]; s = states()
        for leg in LEGS: s[leg + '_steer_joint'] = error
        for _ in range(300): out = core.step(.01, [.3, 0., 0.], s)   # measured steering error held fixed
        assert out.wheels[0] == pytest.approx(.3 / RADIUS * max(0., math.cos(error)) ** 2, abs=1e-9)


def test_steering_slews_at_the_stepper_limit_so_a_crab_turn_takes_under_half_a_second():
    assert STEER_RATE == 3.2
    core = AdaptationCore(RADIUS); s = states(); core.v = [0., .3, 0.]; steps = 0
    while abs(core.steering[0]) < math.pi / 2 - 1e-9 and steps < 200:
        core.step(.01, [0., .3, 0.], s); steps += 1
    assert steps * .01 <= .50 and all(abs(a) == pytest.approx(math.pi / 2) for a in core.steering)


# --- holonomic key maps ---------------------------------------------------------------------------
def test_keyboard_maps_are_holonomic_not_car_like():
    import ast
    source = (P / 'morph_sim/gazebo_keyboard.py').read_text(); tree = ast.parse(source)
    table = next(ast.literal_eval(compile(ast.Expression(n.value), '', 'eval').body if False else n.value) for n in tree.body
                 if isinstance(n, ast.Assign) and getattr(n.targets[0], 'id', '') == 'KEY_VELOCITIES') if False else None
    namespace = {}
    for node in tree.body:
        if isinstance(node, ast.Assign) and getattr(node.targets[0], 'id', '') == 'KEY_VELOCITIES':
            exec(compile(ast.Module([node], []), '', 'exec'), namespace)
    keys = namespace['KEY_VELOCITIES']
    assert keys[ord('W')] == (.30, 0., 0.) and keys[ord('S')] == (-.30, 0., 0.)                 # pure longitudinal
    assert keys[ord('A')][0] == 0 and keys[ord('A')][1] > 0 and keys[ord('A')][2] == 0          # pure lateral, no yaw
    assert keys[ord('D')][0] == 0 and keys[ord('D')][1] < 0 and keys[ord('D')][2] == 0
    assert keys[ord('Q')] == (0., 0., .50) and keys[ord('E')] == (0., 0., -.50)                 # pure yaw
    assert keys[16777234][2] > 0 and keys[16777236][2] < 0                                      # Left / Right arrows yaw
    console = (P / 'morph_sim/control_console.py').read_text()
    assert "msg.linear.y=speed*(int('a' in keys)-int('d' in keys))" in console and "int('q' in keys or 'left' in keys)-int('e' in keys or 'right' in keys)" in console
    assert "'z':'expand'" in console and "'e':'expand'" not in console


@pytest.mark.parametrize('command,check', [((0., .3, 0.), 'strafe'), ((0., 0., .5), 'spin'), ((.3, 0., 0.), 'forward'), ((.2, .2, .3), 'compound')])
def test_three_degree_of_freedom_wheel_commands(command, check):
    core = AdaptationCore(RADIUS); s = states()
    for _ in range(300):
        out = core.step(.01, list(command), s)
        for i, leg in enumerate(LEGS): s[leg + '_steer_joint'] = out.steering[i]
    # Reconstruct the body twist from the four wheel vectors (least squares over the kinematic model).
    import numpy as np
    rows, values = [], []
    for i in range(4):
        x, y = wheel_position(i, 0.); c, sn = math.cos(out.steering[i]), math.sin(out.steering[i]); speed = out.wheels[i] * RADIUS
        rows += [[1., 0., -y], [0., 1., x]]; values += [speed * c, speed * sn]
    twist = np.linalg.lstsq(np.array(rows), np.array(values), rcond=None)[0]
    assert twist == pytest.approx(command, abs=2e-3)
    if check == 'strafe': assert all(abs(a) == pytest.approx(math.pi / 2) for a in out.steering)      # all four modules at 90 degrees
    if check == 'spin': assert sorted(round(abs(math.degrees(a))) for a in out.steering) == [45, 45, 45, 45]


def test_auto_level_uses_the_full_actuator_travel():
    core = AdaptationCore(RADIUS); s = states(); core.pitch = .2; core.event('suspension_auto_level')
    out = core.step(.01, [0., 0., 0.], s, imu_fresh=True)
    assert out.status['auto_level'] and max(abs(h) for h in out.heights) == pytest.approx(HEIGHT_LIMIT)
    assert out.heights[0] > out.heights[2]                       # nose-down pitch: front corners extend, rear retract
    core.event('suspension_level'); assert not core.step(.01, [0., 0., 0.], s, imu_fresh=True).status['auto_level']
