"""Autonomous Tetra-Arena tour node. Drives tour_core.Tour, publishes the decisions and logs the run.

Pose and speed come from ground-truth odometry (/morph/ground_truth/odom): this is a simulation benchmark
driver, not a localisation stack. A zone that the robot cannot pass is recorded as failed and the robot is
repositioned past it with the Gazebo set_pose service so the remaining zones are still measured.

Outputs: /cmd_vel, /morph/event, /morph/surface_friction, /morph/decision_explanation, /morph/tour/status,
and at the end a JSON summary plus a CSV time series (parameter `output_file`, or $MORPH_BENCH_OUTPUT).
"""
import csv, json, math, os, subprocess
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float64
from ament_index_python.packages import get_package_share_directory
from . import lifecycle
from . import decision_core as dc
from .control import LEGS, wheel_velocity
from .design_physics import load_parameters
from .tour_core import Tour, POLICIES

COLUMNS = ['t', 'x', 'y', 'yaw', 'speed', 'wheelbase_m', 'pitch_deg', 'roll_deg', 'stability_margin_deg', 'winding_max_C', 'battery_V', 'battery_A', 'torque_max_Nm',
           'slip_long', 'slip_lat', 'soil_slip_max', 'soil_resistance_N', 'leg', 'phase', 'shape', 'cost_total', 'cost_stability', 'cost_thermal', 'cost_slip', 'cost_clearance', 'cost_time',
           'camera_class', 'camera_boundary_m', 'lidar_slope_deg']


class AutonomousTour(Node):
    def __init__(self):
        super().__init__('morph_autonomous_tour'); share = Path(get_package_share_directory('morph_sim'))
        self.declare_parameter('policy', os.environ.get('MORPH_TOUR_POLICY', 'proposed')); self.declare_parameter('arena_file', str(share / 'config/tetra_arena.json'))
        self.declare_parameter('params_file', str(share / 'config/params.yaml')); self.declare_parameter('output_file', os.environ.get('MORPH_BENCH_OUTPUT', ''))
        self.declare_parameter('exit_on_finish', bool(os.environ.get('MORPH_BENCH_OUTPUT')))
        policy = self.get_parameter('policy').value
        if policy not in POLICIES: raise ValueError('policy must be one of ' + ', '.join(POLICIES))
        self.arena = json.loads(Path(self.get_parameter('arena_file').value).read_text()); self.params = load_parameters(self.get_parameter('params_file').value)
        self.tour = Tour(self.arena, self.params, policy)
        self.status = {}; self.physics = {}; self.soil = {}; self.vision = {}; self.lidar = {}; self.odom = None; self.joints = {}; self.joint_rates = {}
        self.rows = []; self.energy = 0.; self.distance = 0.; self.last_power_t = None; self.last_xy = None; self.last_log = -1e9; self.start_time = None; self.teleports = []
        self.done_written = False; self.hold_until = 0.; self.command = (0., 0., 0.); self.published_text = ''
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 1); self.event = self.create_publisher(String, '/morph/event', 20)
        self.friction = self.create_publisher(Float64, '/morph/surface_friction', 5); self.explain = self.create_publisher(String, '/morph/decision_explanation', 20)
        self.state_pub = self.create_publisher(String, '/morph/tour/status', 5)
        for topic, target in (('/morph/status', 'status'), ('/morph/physics/status', 'physics'), ('/morph/terramechanics/status', 'soil'), ('/morph/vision/telemetry', 'vision'), ('/morph/perception/decision', 'lidar')):
            self.create_subscription(String, topic, lambda m, t=target: self.store(t, m), 10)
        self.create_subscription(Odometry, '/morph/ground_truth/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states', self.on_joints, qos_profile_sensor_data)
        self.create_timer(.02, self.tick)
        self.get_logger().info(f'Autonomous tour, policy {policy}; ground-truth pose; {len(self.tour.legs)} zones')

    def store(self, target, m):
        try: setattr(self, target, json.loads(m.data))
        except ValueError: pass

    def on_odom(self, m): self.odom = m

    def on_joints(self, m): self.joints = dict(zip(m.name, m.position)); self.joint_rates = dict(zip(m.name, m.velocity))

    def now(self): return self.get_clock().now().nanoseconds * 1e-9

    def send(self, name): self.event.publish(String(data=name))

    def teleport(self, x):
        world = self.arena['world']; request = f'name: "morph", position: {{x: {x:.3f}, y: 0.0, z: 0.42}}, orientation: {{x: 0, y: 0, z: 0, w: 1}}'
        result = subprocess.run(['gz', 'service', '-s', f'/world/{world}/set_pose', '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req', request], capture_output=True, text=True, check=False)
        ok = 'true' in result.stdout.lower(); self.teleports.append({'t': self.now(), 'x': x, 'ok': ok})
        self.get_logger().warning(f'Zone failed: repositioned to x = {x:.2f} m ({"ok" if ok else "set_pose failed: " + result.stderr.strip()[:120]})')
        for name in ('suspension_level', 'boost_off', 'reset'): self.send(name)
        self.hold_until = self.now() + 3.

    def slip(self, vx, vy, wz, d):
        """Kinematic slip per wheel against ground truth: longitudinal (wheel rim speed vs contact speed) and lateral."""
        radius = self.params['robot']['wheel_radius_m']; longs = []; lats = []
        for i, leg in enumerate(LEGS):
            steer = self.joints.get(leg + '_steer_joint'); rate = self.joint_rates.get(leg + '_wheel_joint')
            if steer is None or rate is None: return float('nan'), float('nan')
            cx, cy = wheel_velocity(i, (vx, vy, wz), d, self.status.get('stage_rate_measured_m_s') or 0.)
            along = cx * math.cos(steer) + cy * math.sin(steer); across = -cx * math.sin(steer) + cy * math.cos(steer); rim = rate * radius
            scale = max(abs(rim), abs(along), .05); longs.append((rim - along) / scale); lats.append(across / scale)
        return sorted(longs)[1:3][0] * .5 + sorted(longs)[1:3][1] * .5, sorted(lats, key=abs)[-1]

    def tick(self):
        now = self.now(); s = self.status
        if self.odom is None or not s.get('feedback_ready'): return
        if self.start_time is None: self.start_time = now
        p = self.odom.pose.pose.position; q = self.odom.pose.pose.orientation; v = self.odom.twist.twist
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)); speed = math.hypot(v.linear.x, v.linear.y); d = s.get('stage_offset') or 0.
        power = self.physics.get('power', {}); winding = max(power.get('winding_C', [22.]) or [22.]); wheels = self.soil.get('wheels', {})
        if self.last_xy is not None: self.distance += math.hypot(p.x - self.last_xy[0], p.y - self.last_xy[1])
        self.last_xy = (p.x, p.y)
        t_power = self.physics.get('sim_time_s')
        if t_power is not None and t_power != self.last_power_t:
            if self.last_power_t is not None and 0 < t_power - self.last_power_t < 1.: self.energy += power.get('battery_V', 0.) * power.get('battery_current_A', 0.) * (t_power - self.last_power_t)
            self.last_power_t = t_power
        state = dict(x=p.x, y=p.y, yaw=yaw, speed=speed, yaw_rate=v.angular.z, d=d, pitch_deg=s.get('pitch_deg', 0.), settled=not s.get('transition_pending', False), winding_C=winding,
                     soil_resistance_N=sum(w.get('compaction_N', 0.) for w in wheels.values()))
        if now < self.hold_until: out = self.tour.output((0., 0., 0.), None)
        else: out = self.tour.step(now, state)
        for name in out['events']: self.send(name)
        if out['skip_to'] is not None: self.teleport(out['skip_to'])
        if s.get('estop') and now >= self.hold_until:               # an underbody hit latches the e-stop; a stopped robot is a failed leg
            self.send('reset')
        self.command = out['command'] if now >= self.hold_until else (0., 0., 0.)
        msg = Twist(); msg.linear.x, msg.linear.y, msg.angular.z = (float(c) for c in self.command); self.cmd.publish(msg)
        if out['friction'] is not None: self.friction.publish(Float64(data=float(out['friction'])))
        if out['explanation'] != self.published_text: self.published_text = out['explanation']; self.explain.publish(String(data=out['explanation']))
        if now - self.last_log >= .1:
            self.last_log = now; margin = dc.stability_margin(dc.wheelbase(d), math.radians(s.get('roll_deg', 0.)), math.radians(s.get('pitch_deg', 0.)))
            slip_long, slip_lat = self.slip(v.linear.x, v.linear.y, v.angular.z, d); cost = [c if math.isfinite(c) else -1. for c in out['cost']]
            self.rows.append([now - self.start_time, p.x, p.y, yaw, speed, dc.wheelbase(d), s.get('pitch_deg', 0.), s.get('roll_deg', 0.), margin, winding, power.get('battery_V'), power.get('battery_current_A'),
                              max([abs(t) for t in power.get('torques_Nm', [0.])] or [0.]), slip_long, slip_lat, max([w.get('soil_slip', 0.) for w in wheels.values()] or [0.]), state['soil_resistance_N'],
                              out['leg'], out['phase'], out['shape']] + cost[:6] + [self.vision.get('class_filtered'), self.vision.get('boundary_distance_m'), self.lidar.get('slope_deg')])
            self.state_pub.publish(String(data=json.dumps({'sim_time_s': now, 'policy': self.tour.policy, 'leg': out['leg'], 'phase': out['phase'], 'shape': out['shape'], 'explanation': out['explanation'],
                                                           'results': self.tour.results, 'finished': out['finished'], 'pose_source': 'ground truth'})))
        if out['finished'] and not self.done_written: self.finish(now)

    def finish(self, now):
        self.done_written = True; self.cmd.publish(Twist())
        for name in ('suspension_level', 'boost_off'): self.send(name)
        import numpy as np
        rows = self.rows; col = {name: i for i, name in enumerate(COLUMNS)}
        def series(name): return np.array([r[col[name]] if r[col[name]] is not None else np.nan for r in rows], dtype=float)
        moving = series('speed') > .03; mass = self.params['robot']['mass_kg']; results = self.tour.results
        summary = {'passed': True, 'protocol': 'autonomous_tour', 'policy': self.tour.policy, 'pose_source': 'ground-truth odometry', 'scope': 'Simulation of an assumed-parameter model; single run',
                   'mission_time_s': now - self.start_time, 'distance_m': self.distance, 'energy_J': self.energy,
                   'cost_of_transport': self.energy / (mass * dc.G * self.distance) if self.distance > 0 else None,
                   'min_stability_margin_deg': float(np.nanmin(series('stability_margin_deg'))), 'peak_winding_C': float(np.nanmax(series('winding_max_C'))),
                   'rms_longitudinal_slip_percent': float(100 * np.sqrt(np.nanmean(series('slip_long')[moving] ** 2))) if moving.any() else None,
                   'rms_lateral_slip_percent': float(100 * np.sqrt(np.nanmean(series('slip_lat')[moving] ** 2))) if moving.any() else None,
                   'max_soil_slip_percent': float(100 * np.nanmax(series('soil_slip_max'))), 'zones': results, 'zones_passed': sum(r['success'] for r in results), 'zones_total': len(results),
                   'success_rate_percent': 100. * sum(r['success'] for r in results) / max(len(results), 1), 'task_success': all(r['success'] for r in results),
                   'teleports': self.teleports, 'decisions': self.tour.log, 'columns': COLUMNS}
        target = self.get_parameter('output_file').value
        if target:
            path = Path(target); path.parent.mkdir(parents=True, exist_ok=True); path.write_text(json.dumps(summary, indent=1))
            with path.with_suffix('.csv').open('w', newline='') as handle:
                writer = csv.writer(handle); writer.writerow(COLUMNS); writer.writerows(rows)
        self.get_logger().info(f"Tour finished: {summary['zones_passed']}/{summary['zones_total']} zones, {summary['mission_time_s']:.0f} s, CoT {summary['cost_of_transport'] or float('nan'):.2f}")
        if self.get_parameter('exit_on_finish').value: raise SystemExit(0)


def main():
    lifecycle.initialize(); node = AutonomousTour()
    try: lifecycle.spin(node)
    except (KeyboardInterrupt, SystemExit): pass
    finally:
        if rclpy.ok(): node.cmd.publish(Twist())
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
