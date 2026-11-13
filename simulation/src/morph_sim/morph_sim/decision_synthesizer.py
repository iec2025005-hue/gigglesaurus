"""Explainer node: /morph/cost_breakdown at 20 Hz and /morph/decision_explanation on every change.

Advisory. It evaluates the stance cost for the zone the robot is in and the zone ahead from the arena file
and the measured state, and turns every change in mode, stance target, suspension override, torque boost
or speed cap into one line of text with the numbers behind it. When the autonomous tour is running its
own decision lines are relayed unchanged (they are the decisions actually taken).

/morph/cost_breakdown layout (Float64MultiArray): decision_core.COST_FIELDS for the stance in use, then the
measured values [stability_margin_deg_now, thermal_headroom_C, slip_percent, clearance_margin_m, time_to_next_zone_s].
"""
import json, math
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float64MultiArray
from ament_index_python.packages import get_package_share_directory
from . import lifecycle
from . import decision_core as dc
from .design_physics import load_parameters
from .tour_core import build_legs


def shape_name(d):
    return min(dc.SHAPES, key=lambda name: abs(dc.SHAPES[name] - d))


class DecisionSynthesizer(Node):
    def __init__(self):
        super().__init__('morph_decision_synthesizer'); share = Path(get_package_share_directory('morph_sim'))
        self.declare_parameter('arena_file', str(share / 'config/tetra_arena.json')); self.declare_parameter('params_file', str(share / 'config/params.yaml'))
        self.params = load_parameters(self.get_parameter('params_file').value)
        self.legs = build_legs(json.loads(Path(self.get_parameter('arena_file').value).read_text()))
        self.status = {}; self.physics = {}; self.soil = {}; self.vision = {}; self.lidar = {}; self.pose = None; self.speed = 0.; self.tour = {}; self.tour_time = -1e9
        self.previous = {}; self.last_text = ''
        self.explain = self.create_publisher(String, '/morph/decision_explanation', 20); self.cost = self.create_publisher(Float64MultiArray, '/morph/cost_breakdown', 20)
        for topic, target in (('/morph/status', 'status'), ('/morph/physics/status', 'physics'), ('/morph/terramechanics/status', 'soil'), ('/morph/vision/telemetry', 'vision'), ('/morph/perception/decision', 'lidar')):
            self.create_subscription(String, topic, lambda m, t=target: self.store(t, m), 10)
        self.create_subscription(String, '/morph/tour/status', self.on_tour, 10)
        self.create_subscription(Odometry, '/morph/ground_truth/odom', self.on_odom, qos_profile_sensor_data)
        self.create_timer(.05, self.tick)

    def store(self, target, m):
        try: setattr(self, target, json.loads(m.data))
        except ValueError: pass

    def on_tour(self, m):
        try: self.tour = json.loads(m.data); self.tour_time = self.now()
        except ValueError: pass

    def on_odom(self, m):
        p = m.pose.pose.position; v = m.twist.twist.linear; self.pose = (p.x, p.y); self.speed = math.hypot(v.x, v.y)

    def now(self): return self.get_clock().now().nanoseconds * 1e-9

    def say(self, text):
        if text and text != self.last_text: self.last_text = text; self.explain.publish(String(data=text))

    def tick(self):
        s = self.status
        if not s or self.pose is None: return
        d = s.get('stage_offset') or 0.; shape = shape_name(d); x = self.pose[0]
        leg = next((l for l in self.legs if l['x0'] <= x < l['x1']), self.legs[-1]); ahead = next((l for l in self.legs if l['x0'] > x), None)
        winding = max(self.physics.get('power', {}).get('winding_C', [22.]) or [22.]); wheels = self.soil.get('wheels', {})
        context = dict(mass_kg=self.params['robot']['mass_kg'], drive=self.params['drive'], current_d=d, winding_C=winding, torque_boost=bool(s.get('torque_boost')),
                       soil_resistance_N=sum(w.get('compaction_N', 0.) for w in wheels.values()))
        current = dc.evaluate(shape, leg['zone'], context)
        margin_now = dc.stability_margin(dc.wheelbase(d), math.radians(s.get('roll_deg', 0.)), math.radians(s.get('pitch_deg', 0.)))
        slip = 100. * max([w.get('soil_slip', 0.) for w in wheels.values()] or [0.])
        clearance = (leg['zone']['clear_width_m'] - (dc.CRAB_LENGTH[shape] if leg.get('crab') else dc.TYRE_WIDTH)) / 2 if leg['kind'] == 'passage' else float('nan')
        time_to_next = (ahead['x0'] - x) / max(self.speed, .02) if ahead else float('nan')
        vector = [v if math.isfinite(v) else -1. for v in dc.cost_vector(current)] + [margin_now, self.params['drive']['derate_C'] - winding, slip, clearance, time_to_next]
        self.cost.publish(Float64MultiArray(data=[float(v) for v in vector]))
        if self.now() - self.tour_time < 1.:                       # the tour's own decisions are authoritative
            self.say(self.tour.get('explanation', '')); return
        watched = {'mode': s.get('mode'), 'target': round(s.get('target_stage_offset') or 0., 4), 'suspension': bool(s.get('suspension_override')), 'boost': bool(s.get('torque_boost')),
                   'estop': bool(s.get('estop')), 'policy': s.get('morph_policy'), 'zone': leg['id'], 'stalled': bool(s.get('morph_stalled'))}
        changed = [k for k, v in watched.items() if self.previous.get(k) != v]
        if self.previous and changed:
            key = changed[0]; label = leg['zone'].get('label', leg['id']).upper(); best, results = dc.choose(leg['zone'], context)
            if key == 'zone': text = dc.explain(leg['zone'], best, results, speed=max(self.speed, .05)) + f" | now {shape}, tip margin {margin_now:.0f} deg"
            elif key == 'target': text = f"[{label}] stance target {dc.wheelbase(watched['target']) * 1e3:.0f} mm requested (now {dc.wheelbase(d) * 1e3:.0f} mm): shape change {abs(watched['target'] - d) / dc.STAGE_RATE:.1f} s at the measured rail rate; cost now {current['total'] if current['feasible'] else float('inf'):.2f}"
            elif key == 'suspension': text = f"[{label}] corner heights {'overridden: ' + str([round(1e3 * h) for h in s.get('height', [])]) + ' mm' if watched['suspension'] else 'released to the mode height'}; tip margin {margin_now:.0f} deg at pitch {s.get('pitch_deg', 0.):.0f} deg"
            elif key == 'boost': text = f"[{label}] torque boost {'ON (+20 %)' if watched['boost'] else 'off'}: needs {current['required_force_N']:.0f} N, traction use {current['traction_utilisation'] * 100:.0f} %; winding {winding:.0f} C, headroom {self.params['drive']['derate_C'] - winding:.0f} C"
            elif key == 'estop': text = f"[{label}] EMERGENCY STOP {'latched' if watched['estop'] else 'released'}"
            elif key == 'stalled': text = f"[{label}] rails commanded but not moving: shape change stalled" if watched['stalled'] else ''
            elif key == 'mode': text = f"[{label}] drive mode {watched['mode']}: speed cap {self.params['control_modes'].get(watched['mode'], {}).get('speed', float('nan')):.2f} m/s"
            else: text = f"[{label}] morph policy {watched['policy']}"
            camera = self.vision.get('class_filtered'); boundary = self.vision.get('boundary_distance_m')
            if text and camera: text += f" | camera: {camera}" + (f", {self.vision.get('class_ahead')} in {boundary:.1f} m" if boundary else '')
            self.say(text)
        self.previous = watched


def main():
    lifecycle.initialize(); node = DecisionSynthesizer()
    try: lifecycle.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
