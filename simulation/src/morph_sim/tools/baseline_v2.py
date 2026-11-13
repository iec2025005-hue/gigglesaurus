#!/usr/bin/env python3
"""V2 Phase-0 baseline protocol: RTF, speed, stopping, jerk, spin and morph timing.

Run under run_benchmark_case.py ownership (it launches and shuts the simulation down):

    MORPH_PROTOCOL_SCRIPT=src/morph_sim/tools/baseline_v2.py \
      python3 src/morph_sim/tools/run_benchmark_case.py NAME headless:=true rviz:=false

MORPH_BASELINE_MODE=full  full motion protocol (default)
MORPH_BASELINE_MODE=rtf   >=60 s sim-time RTF sample only (20 s driving + idle)
MORPH_BASELINE_MODE=ramp  launch with sector:=ramps; drive up the 4.9/9.8/14.8/19.7/24.6 deg ramp row
                          in the shape named by MORPH_BASELINE_SHAPE (expand|compact|normal)
MORPH_BASELINE_MODE=slip  launch with sector:=mud|snow|sand; full-speed step and stop, wheel slip logged
MORPH_BASELINE_MODE=fixture  V2 arena fixture run: take MORPH_BASELINE_SHAPE, then drive +x at MORPH_BASELINE_SPEED
                          with a heading/lateral hold on y = MORPH_BASELINE_Y until x >= MORPH_BASELINE_GOAL_X,
                          e-stop, or a 6 s stall. Logs soil-model and motor telemetry.
                          MORPH_BASELINE_EVENTS: comma list of /morph/event names sent before driving.
                          MORPH_BASELINE_YAW: body heading in degrees held while travelling along +x (oblique/crab).
                          MORPH_BASELINE_STEP=1: lift each wheel over the arena trenches (morph_sim.trench_crossing).
MORPH_BASELINE_MODE=pivot  from rest with wheels straight: pure strafe, then pure spin, then strafe with the crab tuck; steering angles logged at 100 Hz
MORPH_BASELINE_MODE=onfly  open-loop pairs of identical drives with and without a shape change (continuous policy):
                          straight, arc, and straight with a full Normal->Compact change. No pose feedback.

Raw samples are written unfiltered; tools/baseline_v2_report.py derives the metrics.
Every number is a simulation measurement of an ASSUMED-parameter model.
"""
import json, math, os, sys, time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState

LEGS = ('fl', 'fr', 'rl', 'rr')


class Baseline(Node):
    def __init__(self):
        super().__init__('baseline_v2', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.status = {}; self.status_rows = []; self.odom_rows = []; self.imu_rows = []
        self.phases = []; self.events = []; self.pose = None
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 1)
        self.event = self.create_publisher(String, '/morph/event', 10)
        self.manual = self.create_publisher(String, '/morph/manual_joint', 10)
        self.create_subscription(String, '/morph/status', self.on_status, 10)
        self.create_subscription(Odometry, '/morph/ground_truth/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Imu, '/morph/imu', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states', self.on_joints, qos_profile_sensor_data)
        self.wheel_rows = []; self.soil_rows = []; self.power_rows = []; self.segments = []; self.steer_rows = []
        self.create_subscription(String, '/morph/terramechanics/status', self.on_soil, 10)
        self.create_subscription(String, '/morph/physics/status', self.on_power, 10)
        self.next_pub = 0.

    def sim(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_status(self, m):
        try: s = json.loads(m.data)
        except ValueError: return
        self.status = s
        self.status_rows.append([self.sim(), s.get('command_speed'), s.get('actual_speed'), s.get('stage_offset'),
                                 s.get('target_stage_offset'), s.get('morph_rate_m_s'), s.get('steering_alignment_rad'),
                                 bool(s.get('transition_pending')), bool(s.get('estop')), s.get('pitch_deg'), s.get('roll_deg'),
                                 s.get('steering_measured_degrees'), s.get('mode')])

    def on_odom(self, m):
        p = m.pose.pose.position; q = m.pose.pose.orientation; v = m.twist.twist
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw)
        self.odom_rows.append([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, time.monotonic(), p.x, p.y, p.z, yaw,
                               v.linear.x, v.linear.y, v.linear.z, v.angular.z])

    def on_joints(self, m):
        v = dict(zip(m.name, m.velocity))
        if all(leg + '_wheel_joint' in v for leg in LEGS):
            self.wheel_rows.append([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9] + [v[leg + '_wheel_joint'] for leg in LEGS])
            q = dict(zip(m.name, m.position)); self.steer_rows.append([self.wheel_rows[-1][0]] + [q.get(leg + '_steer_joint') for leg in LEGS])

    def on_soil(self, m):
        try: d = json.loads(m.data)
        except ValueError: return
        w = d['wheels']
        self.soil_rows.append([d['sim_time_s']] + [w[leg][key] for leg in LEGS for key in ('normal_N', 'sinkage_m', 'soil_slip', 'compaction_N', 'thrust_N')] + [w[LEGS[0]]['zone']])

    def on_power(self, m):
        try: d = json.loads(m.data); power = d['power']
        except (ValueError, KeyError): return
        self.power_rows.append([d['sim_time_s'], power['battery_V'], power['battery_current_A'], max(power['winding_C'])] + list(power['torques_Nm']) + list(power['motor_current_A']))

    def on_imu(self, m):
        a = m.linear_acceleration; w = m.angular_velocity
        self.imu_rows.append([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, a.x, a.y, a.z, w.x, w.y, w.z])

    def send(self, name):
        self.event.publish(String(data=name)); self.events.append({'event': name, 'sim_s': self.sim(), 'wall_s': time.monotonic()})

    def step(self, command):
        if time.monotonic() >= self.next_pub:
            msg = Twist(); msg.linear.x, msg.linear.y, msg.angular.z = (float(v) for v in command)
            self.cmd.publish(msg); self.next_pub = time.monotonic() + .02
        rclpy.spin_once(self, timeout_sec=.003)

    def run(self, name, duration, command=(0., 0., 0.), until=None, timeout=None):
        """Hold a command for `duration` sim seconds, or until `until()` holds (bounded by timeout)."""
        start = self.sim(); wall = time.monotonic(); limit = timeout if until else duration
        deadline = wall + max(120., limit * 20.)
        while self.sim() - start < limit:
            if time.monotonic() > deadline: raise RuntimeError('Clock stalled in phase ' + name)
            self.step(command)
            if until and until(): break
        end = self.sim(); wall_end = time.monotonic()
        row = {'phase': name, 'command': list(command), 'sim_start_s': start, 'sim_end_s': end, 'wall_s': wall_end - wall,
               'rtf': (end - start) / max(wall_end - wall, 1e-9), 'timed_out': bool(until) and end - start >= limit}
        self.phases.append(row); print(json.dumps(row), flush=True)
        return row

    def morph_settled(self):
        s = self.status
        if s.get('stage_offset') is None or s.get('target_stage_offset') is None: return False
        close = abs(s['stage_offset'] - s['target_stage_offset']) < .0005 or s.get('transition_settled')
        return bool(close) and not s.get('transition_pending')

    def morph(self, name, event, command=(0., 0., 0.)):
        self.run(name + '_pre', .5, command)
        self.send(event)
        self.run(name + '_latch', .3, command)   # let the manager publish the new target before testing for settle
        return self.run(name, 0., command, until=self.morph_settled, timeout=45.)


def main():
    rclpy.init(); n = Baseline(); mode = os.environ.get('MORPH_BASELINE_MODE', 'full')
    out = Path(os.environ.get('MORPH_BENCH_OUTPUT', 'validation/performance/baseline_v2.json'))
    error = None
    try:
        wall = time.monotonic()
        while (not n.status.get('feedback_ready') or n.pose is None) and time.monotonic() - wall < 90:
            rclpy.spin_once(n, timeout_sec=.02)
        if not n.status.get('feedback_ready'): raise RuntimeError('No feedback from terrain manager')
        n.run('settle', 4.)
        if mode == 'rtf':
            n.run('rtf_drive', 20., (.3, 0., 0.)); n.run('rtf_stop', 3.); n.run('rtf_idle', 40.)
        elif mode == 'ramp':
            shape = os.environ.get('MORPH_BASELINE_SHAPE', 'expand')
            n.morph('ramp_shape_' + shape, shape)
            stalled = {'since': None}
            def finished():
                s = n.status
                if s.get('estop'): return True
                slow = (s.get('actual_speed') or 0.) < .01
                stalled['since'] = (stalled['since'] or n.sim()) if slow else None
                return stalled['since'] is not None and n.sim() - stalled['since'] > 6.
            n.run('ramp_drive_0p12', 0., (.12, 0., 0.), until=finished, timeout=150.); n.run('ramp_stop', 2.)
        elif mode == 'fixture':
            shape = os.environ.get('MORPH_BASELINE_SHAPE', 'normal'); speed = float(os.environ.get('MORPH_BASELINE_SPEED', '0.3'))
            goal = float(os.environ.get('MORPH_BASELINE_GOAL_X', '5')); lane = float(os.environ.get('MORPH_BASELINE_Y', '0'))
            if shape != 'normal': n.morph('fixture_shape_' + shape, shape)
            heading = math.radians(float(os.environ.get('MORPH_BASELINE_YAW', '0')))
            for name in [e for e in os.environ.get('MORPH_BASELINE_EVENTS', '').split(',') if e]: n.send(name)
            n.run('fixture_events_settle', 3.)
            stalled = {'since': None}; command = [speed, 0., 0.]
            stepping = os.environ.get('MORPH_BASELINE_STEP', '') == '1'; step = {'event': 'suspension_level', 'hold_until': 0., 'late': 0, 'conflicts': 0, 'lifts': [], 'current': None, 'done': []}
            if stepping:
                sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
                from morph_sim.trench_crossing import plan as step_plan
                arena = json.loads((Path(__file__).resolve().parents[1] / 'config/tetra_arena.json').read_text())
                trenches = [(t['x'], t['width_m']) for z in arena['zones'] if z['kind'] == 'trench' for t in z['trenches']]
            def finished():
                x, y, yaw = n.pose; s = n.status
                # Hold the lane with small body-frame corrections (ground-truth pose; this is a test rig, not the controller).
                lateral = max(-.08, min(.08, -1.0 * (y - lane)))            # world-frame correction towards the lane
                command[0] = speed * math.cos(yaw) + lateral * math.sin(yaw); command[1] = -speed * math.sin(yaw) + lateral * math.cos(yaw)
                command[2] = max(-.3, min(.3, -1.5 * math.atan2(math.sin(yaw - heading), math.cos(yaw - heading))))
                if stepping:
                    p = step_plan(x, s.get('stage_offset') or 0., yaw, trenches, done={tuple(v) for v in step['done']}); wanted = 'lift_' + p['lift'] if p['lift'] else 'suspension_level'
                    key = [p['lift'], p['trench']] if p['lift'] else None
                    if key != step['current']:
                        if step['current'] is not None: step['done'].append(step['current'])      # that wheel has left its window
                        step['current'] = key
                    if wanted != step['event']:
                        n.send(wanted); step['event'] = wanted; step['hold_until'] = n.sim() + (3.0 if p['lift'] else 0.)
                        if p['lift']: step['lifts'].append([p['lift'], round(x, 3)])
                        if p['inside']: step['late'] += 1
                    step['conflicts'] += int(p['conflict'])
                    if n.sim() < step['hold_until']: command[0] = command[1] = 0.      # wait for the corner heights before the wheel enters
                if x >= goal or s.get('estop'): return True
                if stepping and n.sim() < step['hold_until']: stalled['since'] = None; return False
                slow = (s.get('actual_speed') or 0.) < .01
                stalled['since'] = (stalled['since'] or n.sim()) if slow else None
                return stalled['since'] is not None and n.sim() - stalled['since'] > 6.
            n.run('fixture_drive', 0., command, until=finished, timeout=float(os.environ.get('MORPH_BASELINE_TIMEOUT', '120'))); n.run('fixture_stop', 2.)
            n.segments.append({'name': 'stepping', **step})
        elif mode == 'pivot':
            for name, command in (('pivot_forward', (.3, 0., 0.)), ('pivot_strafe', (0., .3, 0.)), ('pivot_forward_2', (.3, 0., 0.)), ('pivot_spin', (0., 0., .5)), ('pivot_forward_3', (.3, 0., 0.)), ('pivot_compound', (.2, .2, .3))):
                n.run(name, 4., command); n.run(name + '_stop', 2.)
            n.send('crab_tuck_on'); n.run('pivot_forward_4', 3., (.3, 0., 0.)); n.run('pivot_forward_4_stop', 2.)
            n.run('pivot_strafe_tucked', 4., (0., .3, 0.)); n.run('pivot_strafe_tucked_stop', 2.); n.send('crab_tuck_off')
        elif mode == 'onfly':
            n.send('morph_continuous'); n.run('onfly_settle', 2.)
            def segment(name, command, duration, events=()):
                pending = sorted(events); start = n.sim(); n.segments.append({'name': name, 'start_s': start, 'start_pose': list(n.pose), 'command': list(command), 'events': list(events)})
                def tick():
                    while pending and n.sim() - start >= pending[0][0]: n.send(pending.pop(0)[1])
                    return False
                n.run(name, duration, command, until=tick, timeout=duration); n.segments[-1]['end_s'] = n.sim()
                n.run(name + '_stop', 2.5)
            speed = float(os.environ.get('MORPH_BASELINE_SPEED', '0.3')); change = [(2., 'expand'), (8., 'normal')]
            segment('straight_ref', (speed, 0., 0.), 14.); segment('straight_morph', (speed, 0., 0.), 14., change)
            segment('straight_long_ref', (speed, 0., 0.), 16.); segment('straight_compact', (speed, 0., 0.), 16., [(2., 'compact')])
            n.morph('onfly_back_to_normal', 'normal')
            segment('arc_ref', (speed, 0., .2), 14.); segment('arc_morph', (speed, 0., .2), 14., change)
            segment('crab_ref', (0., speed, 0.), 14.); segment('crab_morph', (0., speed, 0.), 14., change)
        elif mode == 'slip':
            n.run('slip_step_0p6', 5., (.6, 0., 0.)); n.run('slip_stop', 3.)
            n.run('slip_step_rev_0p6', 5., (-.6, 0., 0.)); n.run('slip_stop_2', 3.)
        else:
            n.run('fwd_0p6', 5., (.6, 0., 0.)); n.run('fwd_0p6_stop', 3.)
            n.run('fwd_overcap_2p0', 5., (2., 0., 0.)); n.run('fwd_overcap_stop', 3.)
            n.run('rev_0p6', 4., (-.6, 0., 0.)); n.run('rev_stop', 3.)
            n.run('estop_approach', 4., (.6, 0., 0.)); n.send('estop'); n.run('estop_hold', 3., (.6, 0., 0.))
            n.send('reset'); n.run('estop_reset', 1.5)
            for leg in LEGS:
                n.manual.publish(String(data=json.dumps({'joint': leg + '_wheel_joint', 'value': 12., 'duration': 8.})))
            n.run('motor_limit_12rad_s', 8.); n.run('motor_limit_stop', 3.)
            n.run('spin_0p7', 6., (0., 0., .7)); n.run('spin_stop', 3.)
            n.run('crab_0p3', 5., (0., .3, 0.)); n.run('crab_stop', 3.)
            n.run('fwd_to_rev_flip', 4., (.4, 0., 0.)); n.run('fwd_to_rev_flip_rev', 4., (-.4, 0., 0.)); n.run('flip_stop', 3.)
            n.morph('morph_normal_to_extended', 'expand'); n.morph('morph_extended_to_compact', 'compact')
            n.morph('morph_compact_to_extended', 'expand'); n.morph('morph_extended_to_normal', 'normal')
            n.run('moving_pre', 4., (.6, 0., 0.)); n.morph('morph_moving_normal_to_compact', 'compact', (.6, 0., 0.))
            n.run('moving_post', 3., (.6, 0., 0.)); n.morph('morph_moving_compact_to_normal', 'normal', (.6, 0., 0.))
            n.run('final_stop', 3.)
            # Added after the first baseline: crab travel with yaw and with a shape change.
            n.run('crab_yaw_0p15', 6., (0., .15, .08)); n.run('crab_yaw_stop', 2.)
            n.run('crab_pre', 4., (0., .15, 0.)); n.morph('morph_crab_normal_to_compact', 'compact', (0., .15, 0.))
            n.run('crab_compact_post', 4., (0., .3, 0.)); n.morph('morph_crab_compact_to_normal', 'normal', (0., .15, 0.))
            n.run('crab_final_stop', 3.)
    except Exception as exc:  # keep whatever was recorded; the report states the failure
        error = repr(exc); print('PROTOCOL ERROR ' + error, flush=True)
    finally:
        n.cmd.publish(Twist())
        sim = sum(p['sim_end_s'] - p['sim_start_s'] for p in n.phases); wall_s = sum(p['wall_s'] for p in n.phases)
        result = {'protocol': 'baseline_v2', 'mode': mode, 'error': error, 'sim_s': sim, 'wall_s': wall_s,
                  'mean_rtf': sim / wall_s if wall_s else None, 'phases': n.phases, 'events': n.events, 'segments': n.segments,
                  'columns': {'odom': ['sim_s', 'wall_s', 'x', 'y', 'z', 'yaw', 'vx_body', 'vy_body', 'vz_body', 'wz'],
                              'imu': ['sim_s', 'ax', 'ay', 'az', 'wx', 'wy', 'wz'], 'wheels': ['sim_s', 'fl', 'fr', 'rl', 'rr'],
                              'soil': ['sim_s'] + [f'{leg}_{key}' for leg in LEGS for key in ('normal_N', 'sinkage_m', 'soil_slip', 'compaction_N', 'thrust_N')] + ['fl_zone'],
                              'power': ['sim_s', 'battery_V', 'battery_current_A', 'max_winding_C', 'tq_fl', 'tq_fr', 'tq_rl', 'tq_rr', 'i_fl', 'i_fr', 'i_rl', 'i_rr'],
                              'status': ['sim_s', 'command_speed', 'actual_speed', 'stage_offset', 'target_stage_offset', 'morph_rate_m_s',
                                         'steering_alignment_rad', 'transition_pending', 'estop', 'pitch_deg', 'roll_deg',
                                         'steering_measured_degrees', 'mode']},
                  'samples': {'odom': n.odom_rows, 'imu': n.imu_rows, 'status': n.status_rows, 'wheels': n.wheel_rows, 'soil': n.soil_rows, 'power': n.power_rows, 'steer': n.steer_rows},
                  'scope': 'Simulation of an assumed-parameter model; not hardware data'}
        out.parent.mkdir(parents=True, exist_ok=True); out.write_text(json.dumps(result))
        n.destroy_node(); rclpy.shutdown()
    raise SystemExit(1 if error else 0)


if __name__ == '__main__':
    main()
