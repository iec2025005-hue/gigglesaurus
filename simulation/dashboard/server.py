#!/usr/bin/env python3
"""MORPH V2 web dashboard: Flask + Server-Sent Events + MJPEG camera.

    cd simulation && source /opt/ros/<distro>/setup.bash && source install/setup.bash && source transport_env.sh
    python3 dashboard/server.py [--port 8080] [--host 127.0.0.1]

Open http://127.0.0.1:8080. The page is one static file (dashboard/index.html), no build step.
ROS runs in its own thread; the web server never blocks it. By default the server listens on localhost
only (another team's ROS system shares this network); pass --host 0.0.0.0 to open it to the LAN.
FastAPI/WebSocket packages are not installed on this machine, so the live channel is SSE over Flask.
"""
import argparse, json, math, threading, time
from collections import deque
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from flask import Flask, Response, jsonify, request, send_from_directory
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String, Float64MultiArray

HERE = Path(__file__).resolve().parent
ALLOWED_EVENTS = {'estop', 'reset', 'compact', 'normal', 'expand', 'morph_gated', 'morph_continuous', 'suspension_level', 'suspension_auto_level', 'pitch_nose_up', 'pitch_nose_down',
                  'boost_on', 'boost_off', 'crab_tuck_on', 'crab_tuck_off', 'track', 'sand', 'climb', 'rocky'}
COST_FIELDS = ['total', 'stability', 'thermal', 'slip', 'clearance', 'time', 'stability_margin_deg_planned', 'traction_utilisation', 'stability_margin_deg', 'thermal_headroom_C', 'slip_percent', 'clearance_margin_m', 'time_to_next_zone_s']


def radar_from_cloud(m, sectors=14, fov=1.1, sensor_height=.455):
    """Polar occupancy over the LiDAR field of view: FREE / BLOCKED / SLOPE / DROP / UNKNOWN per azimuth sector and the nearest range."""
    fields = {f.name: f for f in m.fields}
    if not all(k in fields for k in ('x', 'y', 'z')): return []
    dtype = np.dtype({'names': ['x', 'y', 'z'], 'formats': ['<f4'] * 3, 'offsets': [fields[k].offset for k in ('x', 'y', 'z')], 'itemsize': m.point_step})
    a = np.frombuffer(m.data, dtype=dtype, count=m.width * m.height); p = np.column_stack([a['x'], a['y'], a['z']]).astype(float); p = p[np.isfinite(p).all(axis=1)]
    out = []; azimuth = np.arctan2(p[:, 1], p[:, 0]) if len(p) else np.array([]); rng = np.hypot(p[:, 0], p[:, 1]) if len(p) else np.array([])
    for i in range(sectors):
        lo = -fov + 2 * fov * i / sectors; hi = lo + 2 * fov / sectors; sel = (azimuth >= lo) & (azimuth < hi)
        if sel.sum() < 3: out.append({'a0': lo, 'a1': hi, 'state': 'UNKNOWN', 'range': None}); continue
        z = p[sel, 2] + sensor_height; r = rng[sel]; obstacle = (z > .18) & (r < 3.)
        ground = (np.abs(z) < .08) & (r < 2.5)
        raised = (z > .03) & (r < 3.)
        if obstacle.sum() >= 2 and raised.sum() >= 6:           # a ramp is a plane through the raised points, a wall is not
            design = np.column_stack([r[raised], np.ones(int(raised.sum()))]); fit = np.linalg.lstsq(design, z[raised], rcond=None)[0]
            grade = math.degrees(math.atan(fit[0])); rms = float(np.sqrt(np.mean((design @ fit - z[raised]) ** 2)))
            if 3. < grade < 30. and rms < .03: out.append({'a0': lo, 'a1': hi, 'state': 'SLOPE', 'range': float(r[raised].min()), 'grade_deg': grade}); continue
        if obstacle.sum() >= 2: out.append({'a0': lo, 'a1': hi, 'state': 'BLOCKED', 'range': float(r[obstacle].min())})
        elif ((z < -.10) & (r < 2.5)).sum() >= 2 or ground.sum() < 2: out.append({'a0': lo, 'a1': hi, 'state': 'DROP', 'range': float(r[(z < -.10)].min()) if (z < -.10).any() else None})
        else: out.append({'a0': lo, 'a1': hi, 'state': 'FREE', 'range': float(r[ground].max())})
    return out


class Bridge(Node):
    def __init__(self):
        super().__init__('morph_dashboard', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.lock = threading.Lock(); self.state = {}; self.log = deque(maxlen=50); self.jpeg = None; self.jpeg_wall = 0.; self.last_cloud = 0.; self.history = deque(maxlen=600)
        for topic, key in (('/morph/status', 'status'), ('/morph/physics/status', 'physics'), ('/morph/terramechanics/status', 'soil'), ('/morph/vision/telemetry', 'vision'),
                           ('/morph/perception/decision', 'lidar'), ('/morph/tour/status', 'tour'), ('/morph/watchdog/status', 'watchdog')):
            self.create_subscription(String, topic, lambda m, k=key: self.store(k, m), 10)
        self.create_subscription(String, '/morph/decision_explanation', self.on_explanation, 20)
        self.create_subscription(Float64MultiArray, '/morph/cost_breakdown', self.on_cost, 20)
        self.create_subscription(Odometry, '/morph/ground_truth/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Image, '/morph/camera/image', self.on_image, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/morph/perception/points', self.on_cloud, qos_profile_sensor_data)
        self.events = self.create_publisher(String, '/morph/event', 10)

    def sim(self): return self.get_clock().now().nanoseconds * 1e-9

    def store(self, key, m):
        try: value = json.loads(m.data)
        except ValueError: return
        with self.lock: self.state[key] = value; self.state[key + '_wall'] = time.monotonic()

    def on_explanation(self, m):
        with self.lock: self.log.append({'t': round(self.sim(), 2), 'text': m.data})

    def on_cost(self, m):
        with self.lock: self.state['cost'] = dict(zip(COST_FIELDS, [None if not math.isfinite(v) else v for v in m.data]))

    def on_odom(self, m):
        p = m.pose.pose.position; q = m.pose.pose.orientation; v = m.twist.twist
        with self.lock: self.state['pose'] = {'x': p.x, 'y': p.y, 'yaw': math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)), 'vx': v.linear.x, 'vy': v.linear.y, 'wz': v.angular.z}

    def on_image(self, m):
        if time.monotonic() - self.jpeg_wall < .1 or m.encoding not in ('rgb8', 'bgr8'): return
        import cv2
        image = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.step // 3, 3)[:, :m.width]
        if m.encoding == 'rgb8': image = image[:, :, ::-1]
        ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if ok:
            with self.lock: self.jpeg = encoded.tobytes(); self.jpeg_wall = time.monotonic(); self.state['camera_latency_ms'] = 1000 * (self.sim() - (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9))

    def on_cloud(self, m):
        if time.monotonic() - self.last_cloud < .2: return
        self.last_cloud = time.monotonic()
        try: radar = radar_from_cloud(m)
        except (ValueError, TypeError): return
        with self.lock: self.state['radar'] = radar

    def snapshot(self):
        with self.lock:
            s = dict(self.state); log = list(self.log)
        status = s.get('status', {}); power = s.get('physics', {}).get('power', {}); wheels = s.get('soil', {}).get('wheels', {}); now = time.monotonic()
        d = status.get('stage_offset') or 0.
        snap = {'sim_time_s': status.get('sim_time_s'), 'link_ok': now - s.get('status_wall', 0) < 1., 'wheelbase_mm': 1e3 * (.54 + 8 * d), 'target_wheelbase_mm': 1e3 * (.54 + 8 * (status.get('target_stage_offset') or 0.)), 'track_mm': 540,
                'steering_deg': status.get('steering_measured_degrees'), 'corner_heights_mm': [1e3 * h for h in status.get('height', [])], 'mode': status.get('mode'), 'estop': status.get('estop'),
                'morph_policy': status.get('morph_policy'), 'transition_pending': status.get('transition_pending'), 'morph_stalled': status.get('morph_stalled'), 'torque_boost': status.get('torque_boost'),
                'suspension_override': status.get('suspension_override'), 'crab_tuck': status.get('crab_tuck'), 'pitch_deg': status.get('pitch_deg'), 'roll_deg': status.get('roll_deg'), 'command_speed': status.get('command_speed'),
                'command_accel': status.get('command_accel_m_s2'), 'wheel_contacts': status.get('wheel_contacts'), 'pose': s.get('pose'), 'winding_C': power.get('winding_C'), 'battery_V': power.get('battery_V'),
                'battery_A': power.get('battery_current_A'), 'soc': power.get('soc'), 'torques_Nm': power.get('torques_Nm'), 'boost_factor': power.get('torque_boost_factor'),
                'soil': {k: {'zone': w.get('zone'), 'sinkage_mm': 1e3 * w.get('sinkage_m', 0.), 'slip': w.get('soil_slip'), 'load_N': w.get('normal_N')} for k, w in wheels.items()},
                'vision': {k: s.get('vision', {}).get(k) for k in ('class_filtered', 'class_ahead', 'boundary_distance_m', 'confidence', 'hsv_here')}, 'lidar': {k: s.get('lidar', {}).get(k) for k in ('mode', 'reason', 'slope_deg', 'roughness_m', 'nearest_obstacle_m')},
                'radar': s.get('radar', []), 'cost': s.get('cost'), 'tour': {k: s.get('tour', {}).get(k) for k in ('policy', 'leg', 'phase', 'shape', 'finished', 'results')}, 'log': log,
                'camera': self.jpeg is not None, 'camera_latency_ms': s.get('camera_latency_ms'), 'pose_source': 'ground truth'}
        return snap


def create_app(bridge):
    app = Flask(__name__, static_folder=None)

    @app.get('/')
    def index(): return send_from_directory(HERE, 'index.html')

    @app.get('/api/state')
    def state(): return jsonify(bridge.snapshot())

    @app.get('/stream')
    def stream():
        def generate():
            while True:
                yield 'data: ' + json.dumps(bridge.snapshot()) + '\n\n'; time.sleep(.1)
        return Response(generate(), mimetype='text/event-stream', headers={'Cache-Control': 'no-cache', 'X-Accel-Buffering': 'no'})

    @app.get('/camera.mjpg')
    def camera():
        def generate():
            last = None
            while True:
                with bridge.lock: frame = bridge.jpeg
                if frame is not None and frame is not last: last = frame; yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n'
                time.sleep(.05)
        return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

    @app.post('/api/event')
    def event():
        name = (request.get_json(silent=True) or {}).get('name', '')
        if name not in ALLOWED_EVENTS: return jsonify({'ok': False, 'error': 'event not allowed'}), 400
        bridge.events.publish(String(data=name)); return jsonify({'ok': True, 'event': name})
    return app


def main():
    parser = argparse.ArgumentParser(); parser.add_argument('--port', type=int, default=8080); parser.add_argument('--host', default='127.0.0.1'); args = parser.parse_args()
    rclpy.init(); bridge = Bridge(); threading.Thread(target=rclpy.spin, args=(bridge,), daemon=True).start()
    try: create_app(bridge).run(host=args.host, port=args.port, threaded=True, debug=False, use_reloader=False)
    finally:
        bridge.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
