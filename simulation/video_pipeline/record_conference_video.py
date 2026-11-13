#!/usr/bin/env python3
"""Conference video of one autonomous Tetra-Arena tour, composed frame by frame in SIMULATION time.

    cd simulation && source install/setup.bash
    python3 video_pipeline/record_conference_video.py                # proposed policy, 2x playback
    python3 video_pipeline/record_conference_video.py --speed 1      # real-time playback (needs about 2x the bitrate)

Layout (1920 x 1080, 60 fps):
    left   960 x 880   Gazebo camera riding with the robot (chase / side at the trenches / overhead in the pinch)
    right  960 x 880   analytical view drawn from telemetry: LiDAR points, wheel poses and steering angles,
                       support polygon, projected centre of mass, corner heights, stance cost terms
    bottom 1920 x 200  decision explanation and strip charts of wheelbase L(t), tip-over margin, winding temperature

Timing. The cameras render at 60 / speed Hz of SIMULATION time and every image carries its simulation stamp.
Output frame k shows simulation time t0 + k * speed / 60, so the video is the same whatever real-time factor the
computer reached. If a camera frame is missing the next one that arrives fills the gap and is counted
(`repeated_frames` in the metadata); nothing is interpolated. The playback speed is printed on every frame.

Encoding. H.264 in MP4. `ffmpeg` is used when it is installed; otherwise GStreamer's x264 encoder. A first pass
writes a near-lossless intermediate while the simulator runs, a second pass sets the bitrate from the real
duration so that the file stays under --max-mb.

Two modes in one file: started by hand it launches the simulator through tools/run_benchmark_case.py with itself
as the protocol script; started by that runner ($MORPH_BENCH_OUTPUT set) it drives the tour and records.
"""
import argparse, json, math, os, shutil, subprocess, sys, time
from collections import deque
from pathlib import Path

WS = Path(__file__).resolve().parents[1]
VIDEOS = WS / 'videos'
WIDTH, HEIGHT, FPS = 1920, 1080, 60
PANE_W, PANE_H, HUD_H = 960, 880, 200
CAMERA_BY_LEG = {'trenches': 'side', 'pinch': 'top'}         # every other zone uses the chase camera
CAMERA_LABEL = {'chase': 'CHASE CAMERA', 'side': 'SIDE CAMERA', 'top': 'OVERHEAD CAMERA'}
LIDAR_OFFSET = (.22, .12 + .335)                             # sensor ahead of the body origin, and above the ground [m]
SELF_RETURN_M = .30                                           # LiDAR returns closer than this are the robot's own telescope and are not drawn
TAIL_S = 2.0                                                 # simulation seconds recorded after the tour ends
HISTORY_S = 60.                                              # strip chart window, simulation seconds
SCALE = 190.                                                 # px per metre in the top-down view
ORIGIN = (480, 560)                                          # body origin in the right pane [px]
BG, PANEL, GRID, TEXT, DIM = (24, 20, 17), (38, 32, 27), (62, 54, 46), (235, 235, 235), (150, 150, 150)
GREEN, AMBER, RED, BLUE, CYAN = (127, 191, 63), (60, 178, 230), (82, 82, 224), (234, 158, 74), (220, 200, 60)


# ---------------------------------------------------------------- launcher mode
def encode_final(raw, target, duration_s, max_mb, log):
    """Second pass: constant-bitrate H.264 sized for max_mb. Returns (encoder name, bitrate kbit/s)."""
    kbps = int(min(8000, .93 * max_mb * 8 * 1024 * 1024 / 1000 / max(duration_s, 1.)))
    for attempt in range(5):
        if shutil.which('ffmpeg'):
            name = 'ffmpeg libx264'
            command = ['ffmpeg', '-y', '-loglevel', 'error', '-i', str(raw), '-c:v', 'libx264', '-preset', 'slow', '-b:v', f'{kbps}k', '-maxrate', f'{int(kbps * 1.5)}k', '-bufsize', f'{kbps * 2}k',
                       '-pix_fmt', 'yuv420p', '-r', str(FPS), '-movflags', '+faststart', '-an', str(target)]
        else:
            name = 'GStreamer x264enc'
            command = ['gst-launch-1.0', '-q', 'filesrc', f'location={raw}', '!', 'qtdemux', '!', 'avdec_h264', '!', 'videoconvert', '!', 'x264enc', f'bitrate={kbps}', 'speed-preset=slow', 'key-int-max=120',
                       '!', 'video/x-h264,stream-format=avc,profile=high', '!', 'mp4mux', 'faststart=true', '!', 'filesink', f'location={target}']
        result = subprocess.run(command, capture_output=True, text=True); log.write(f'{" ".join(command)}\n{result.stdout}{result.stderr}\n')
        if result.returncode or not target.exists(): raise RuntimeError(f'{name} failed: {result.stderr[-400:]}')
        size_mb = target.stat().st_size / 1024 / 1024
        if size_mb < max_mb: return name, kbps, size_mb
        kbps = int(kbps * .9 * max_mb / size_mb)
    raise RuntimeError(f'could not reach {max_mb} MB; last size {size_mb:.1f} MB')


def launch(args):
    name = args.name or f'v2video_{args.policy}_{time.strftime("%Y%m%d_%H%M%S")}'
    directory = WS / 'validation/performance' / name; VIDEOS.mkdir(exist_ok=True); (WS / 'log').mkdir(exist_ok=True)
    env = dict(os.environ, MORPH_PROTOCOL_SCRIPT=str(Path(__file__).resolve()), MORPH_TOUR_POLICY=args.policy, MORPH_VIDEO_SPEED=str(args.speed), MORPH_TOUR_WALL_LIMIT_S=str(args.wall_limit),
               MORPH_LOADED_SOAK_WALL_SECONDS=str(args.wall_limit), ROS_AUTOMATIC_DISCOVERY_RANGE=os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'), FASTDDS_BUILTIN_TRANSPORTS='UDPv4')
    command = ['python3', 'src/morph_sim/tools/run_benchmark_case.py', name, 'headless:=true', 'rviz:=false', 'arena:=tetra', 'camera_enabled:=true', 'camera_width:=320',
               'chase_cameras:=true', f'chase_camera_rate:={FPS / args.speed:g}']
    with (WS / 'log' / f'{name}.log').open('w') as log:
        code = subprocess.run(command, cwd=WS, env=env, stdout=log, stderr=subprocess.STDOUT, check=False, timeout=args.wall_limit + 600).returncode
    raw = directory / 'video_raw.mp4'; meta_file = directory / 'video.json'
    if not raw.exists() or not meta_file.exists(): raise SystemExit(f'recording failed (runner exit {code}); see {directory}/benchmark.log')
    meta = json.loads(meta_file.read_text()); target = Path(args.output) if args.output else VIDEOS / 'morph_conference_video_60fps.mp4'
    with (directory / 'encode.log').open('w') as log: encoder, kbps, size_mb = encode_final(raw, target, meta['duration_s'], args.max_mb, log)
    meta.update(file=str(target.relative_to(WS)) if target.is_relative_to(WS) else str(target), encoder=encoder, bitrate_kbit_s=kbps, size_MB=round(size_mb, 2), max_MB=args.max_mb, run=name)
    target.with_suffix('.json').write_text(json.dumps(meta, indent=1)); meta_file.write_text(json.dumps(meta, indent=1))
    if not args.keep_raw: raw.unlink()
    print(json.dumps({k: meta[k] for k in ('file', 'frames', 'duration_s', 'playback_speed', 'repeated_frames', 'size_MB', 'encoder', 'bitrate_kbit_s', 'policy', 'zones_passed', 'zones_total')}, indent=1))


# ---------------------------------------------------------------- protocol mode
def record():
    import cv2, numpy as np, rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    import queue
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from gz.transport import Node as GzNode
    from gz.msgs.image_pb2 import Image as GzImage
    from std_msgs.msg import String, Float64MultiArray
    from morph_sim import autonomous_tour, decision_core as dc
    from morph_sim.control import LEGS, wheel_position

    speed = float(os.environ.get('MORPH_VIDEO_SPEED', '2')); output = Path(os.environ['MORPH_BENCH_OUTPUT']).parent
    font = cv2.FONT_HERSHEY_SIMPLEX

    def put(img, text, org, scale=.5, colour=TEXT, thick=1): cv2.putText(img, text, org, font, scale, colour, thick, cv2.LINE_AA)

    def px(x, y): return int(round(ORIGIN[0] - y * SCALE)), int(round(ORIGIN[1] - x * SCALE))          # body x up, body y left

    class Recorder(Node):
        def __init__(self):
            super().__init__('morph_video_recorder', parameter_overrides=[Parameter('use_sim_time', value=True)])
            self.status = {}; self.physics = {}; self.tour = {}; self.cost = None; self.explanation = ''; self.cloud = None; self.speed_now = 0.
            self.history = deque(); self.t0 = None; self.next_frame = 0; self.repeated = 0; self.source_frames = 0; self.last_stamp = None; self.end_time = None; self.closed = False
            self.camera_frames = {name: 0 for name in CAMERA_LABEL}; self.active = 'chase'; self.canvas = np.zeros((HEIGHT, WIDTH, 3), np.uint8); self.static = self.background()
            pipeline = (f'appsrc ! videoconvert ! video/x-raw,format=I420 ! x264enc pass=quant quantizer=16 speed-preset=veryfast key-int-max=60 ! video/x-h264,stream-format=avc ! mp4mux ! '
                        f'filesink location="{output / "video_raw.mp4"}"')                  # quoted: the workspace path may contain spaces
            self.writer = cv2.VideoWriter(pipeline, cv2.CAP_GSTREAMER, 0, float(FPS), (WIDTH, HEIGHT), True); self.encoder = 'GStreamer x264enc (intermediate, quantizer 16)'
            if not self.writer.isOpened():
                self.writer = cv2.VideoWriter(str(output / 'video_raw.mp4'), cv2.VideoWriter_fourcc(*'avc1'), float(FPS), (WIDTH, HEIGHT)); self.encoder = 'OpenCV avc1 (intermediate)'
            if not self.writer.isOpened(): raise RuntimeError('no H.264 writer available (GStreamer x264enc and OpenCV avc1 both failed)')
            for topic, key in (('/morph/status', 'status'), ('/morph/physics/status', 'physics'), ('/morph/tour/status', 'tour')):
                self.create_subscription(String, topic, lambda m, k=key: self.store(k, m), 10)
            self.create_subscription(String, '/morph/decision_explanation', self.on_explanation, 20)
            self.create_subscription(Float64MultiArray, '/morph/cost_breakdown', lambda m: setattr(self, 'cost', list(m.data)), 10)
            self.create_subscription(Odometry, '/morph/ground_truth/odom', lambda m: setattr(self, 'speed_now', math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)), qos_profile_sensor_data)
            self.create_subscription(PointCloud2, '/morph/perception/points', self.on_cloud, qos_profile_sensor_data)
            # The three 2.5 MB image streams are read straight from Gazebo transport. Over ROS best-effort UDP most of
            # them were lost (first test: 2457 of 10786 frames arrived). Frames of the camera in use are queued and
            # composed in the main loop; the queue is bounded so a slow encoder shows up as repeated frames, not memory.
            self.frames = queue.Queue(maxsize=240); self.dropped_queue = 0; self.gz = GzNode()
            for name in CAMERA_LABEL: self.gz.subscribe(GzImage, '/morph/video/' + name, lambda m, n=name: self.on_gz_image(n, m))

        def on_gz_image(self, name, m):
            if self.closed or name != self.active: return
            try: self.frames.put_nowait((name, m))
            except queue.Full: self.dropped_queue += 1

        def drain(self, budget_s=.2):
            end = time.monotonic() + budget_s
            while not self.closed and time.monotonic() < end:
                try: name, m = self.frames.get_nowait()
                except queue.Empty: return
                self.on_image(name, m)

        # ---- telemetry
        def store(self, key, m):
            try: data = json.loads(m.data)
            except ValueError: return
            setattr(self, key, data)
            if key == 'tour':
                self.active = CAMERA_BY_LEG.get(data.get('leg'), 'chase')
                if data.get('explanation'): self.explanation = data['explanation']
                if data.get('finished') and self.end_time is None and data.get('sim_time_s') is not None: self.end_time = data['sim_time_s'] + TAIL_S
            if key == 'status' and data.get('sim_time_s') is not None:
                d = data.get('stage_offset') or 0.; winding = max(self.physics.get('power', {}).get('winding_C', [22.]) or [22.])
                margin = dc.stability_margin(dc.wheelbase(d), math.radians(data.get('roll_deg', 0.)), math.radians(data.get('pitch_deg', 0.)))
                self.history.append((data['sim_time_s'], 1e3 * dc.wheelbase(d), 1e3 * dc.wheelbase(data.get('target_stage_offset') or 0.), margin, winding))
                while self.history and self.history[0][0] < data['sim_time_s'] - HISTORY_S: self.history.popleft()

        def on_explanation(self, m):
            if not self.tour.get('explanation'): self.explanation = m.data

        def on_cloud(self, m):
            fields = {f.name: f for f in m.fields}
            if not all(k in fields for k in 'xyz'): return
            dtype = np.dtype({'names': ['x', 'y', 'z'], 'formats': ['<f4'] * 3, 'offsets': [fields[k].offset for k in 'xyz'], 'itemsize': m.point_step})
            a = np.frombuffer(m.data, dtype=dtype, count=m.width * m.height); p = np.column_stack([a['x'], a['y'], a['z']]); p = p[np.isfinite(p).all(axis=1)]
            u = np.rint(ORIGIN[0] - p[:, 1] * SCALE).astype(int); v = np.rint(ORIGIN[1] - (p[:, 0] + LIDAR_OFFSET[0]) * SCALE).astype(int); z = p[:, 2] + LIDAR_OFFSET[1]
            keep = (u > 2) & (u < PANE_W - 3) & (v > 42) & (v < PANE_H - 3) & (np.hypot(p[:, 0], p[:, 1]) > SELF_RETURN_M); colour = np.full((len(p), 3), 135, np.uint8); colour[z > .05] = RED; colour[z < -.05] = AMBER
            self.cloud = (u[keep], v[keep], colour[keep])

        # ---- drawing
        def background(self):
            """Right pane and HUD parts that never change."""
            img = np.full((HEIGHT, WIDTH, 3), BG, np.uint8); pane = img[:PANE_H, PANE_W:]
            for radius in (.5, 1., 1.5, 2., 2.5): cv2.circle(pane, ORIGIN, int(radius * SCALE), GRID, 1, cv2.LINE_AA); put(pane, f'{radius:g} m', (ORIGIN[0] + 4, ORIGIN[1] - int(radius * SCALE) - 4), .38, DIM)
            cv2.line(pane, (ORIGIN[0], 44), (ORIGIN[0], PANE_H - 4), GRID, 1); cv2.line(pane, (4, ORIGIN[1]), (PANE_W - 4, ORIGIN[1]), GRID, 1)
            cv2.rectangle(pane, (0, 0), (PANE_W - 1, 38), PANEL, -1); put(pane, 'ANALYTICAL VIEW  (body frame, from telemetry)', (14, 26), .6)
            for label, colour, x in (('LiDAR ground', (135, 135, 135), 560), ('raised', RED, 700), ('below', AMBER, 780), ('CoM', CYAN, 860)):
                cv2.circle(pane, (x, 20), 5, colour, -1); put(pane, label, (x + 10, 25), .42, DIM)
            cv2.rectangle(img, (0, PANE_H), (WIDTH - 1, HEIGHT - 1), PANEL, -1); cv2.line(img, (0, PANE_H), (WIDTH, PANE_H), GRID, 1); cv2.line(img, (PANE_W, 0), (PANE_W, PANE_H), GRID, 1)
            return img

        def draw_robot(self, pane):
            s = self.status; d = s.get('stage_offset') or 0.; steering = s.get('steering_measured_degrees') or [0.] * 4; heights = s.get('height') or [0.] * 4
            corners = [wheel_position(i, d) for i in range(4)]; order = [0, 1, 3, 2]                # fl, fr, rr, rl around the polygon
            polygon = np.array([px(*corners[i]) for i in order], np.int32)
            overlay = pane.copy(); cv2.fillPoly(overlay, [polygon], (70, 90, 60)); cv2.addWeighted(overlay, .35, pane, .65, 0, pane); cv2.polylines(pane, [polygon], True, GREEN, 2, cv2.LINE_AA)
            half = .165                                                                              # deck is the fixed 330 mm centre section
            cv2.rectangle(pane, px(half, .20), px(-half, -.20), (90, 90, 90), 1, cv2.LINE_AA)
            for i, (x, y) in enumerate(corners):
                angle = math.radians(steering[i] if i < len(steering) else 0.); c, sn = math.cos(angle), math.sin(angle)
                box = np.array([px(x + a * c - b * sn, y + a * sn + b * c) for a, b in ((.075, .0225), (.075, -.0225), (-.075, -.0225), (-.075, .0225))], np.int32)
                cv2.fillPoly(pane, [box], (200, 200, 200)); cv2.arrowedLine(pane, px(x, y), px(x + .2 * c, y + .2 * sn), BLUE, 2, cv2.LINE_AA, tipLength=.3)
                h = 1e3 * heights[i] if i < len(heights) else 0.; u, v = px(x, y); side = -1 if y > 0 else 1
                put(pane, f'{LEGS[i].upper()} {steering[i] if i < len(steering) else 0.:+.0f} deg', (u + (18 if side > 0 else -112), v - 6), .45)
                put(pane, f'height {h:+.0f} mm', (u + (18 if side > 0 else -128), v + 12), .42, AMBER if abs(h) > 1 else DIM)
            cv2.arrowedLine(pane, px(0, 0), px(.3, 0), (60, 60, 230), 2, cv2.LINE_AA, tipLength=.2); cv2.arrowedLine(pane, px(0, 0), px(0, .3), (60, 200, 60), 2, cv2.LINE_AA, tipLength=.2)
            put(pane, 'x', px(.33, -.01), .45, (60, 60, 230)); put(pane, 'y', px(.02, .36), .45, (60, 200, 60))
            roll, pitch = math.radians(s.get('roll_deg', 0.)), math.radians(s.get('pitch_deg', 0.)); gx, gy, gz = dc.gravity_in_body(roll, pitch)
            com = (dc.COM_HEIGHT * gx / -gz, dc.COM_HEIGHT * gy / -gz) if gz < -1e-3 else (0., 0.); margin = dc.stability_margin(dc.wheelbase(d), roll, pitch)
            colour = GREEN if margin > 15 else AMBER if margin > 5 else RED
            cv2.circle(pane, px(*com), 9, colour, -1, cv2.LINE_AA); cv2.circle(pane, px(*com), 9, (255, 255, 255), 1, cv2.LINE_AA); cv2.drawMarker(pane, px(0, 0), CYAN, cv2.MARKER_CROSS, 10, 1)
            return d, margin, colour

        def draw_panel(self, pane, d, margin, colour):
            s = self.status; x0, y0 = 14, 60
            rows = [(f'Wheelbase L   {1e3 * dc.wheelbase(d):5.0f} mm', TEXT), (f'target        {1e3 * dc.wheelbase(s.get("target_stage_offset") or 0.):5.0f} mm', DIM), ('Track           540 mm (fixed)', DIM),
                    (f'Tip margin    {margin:5.1f} deg', colour), (f'Pitch {s.get("pitch_deg", 0.):+5.1f}  roll {s.get("roll_deg", 0.):+5.1f} deg', TEXT), (f'Speed         {self.speed_now:5.2f} m/s', TEXT),
                    (f'Torque boost  {"ON" if s.get("torque_boost") else "off"}', AMBER if s.get('torque_boost') else DIM), (f'Crab tuck     {"ON" if s.get("crab_tuck") else "off"}', AMBER if s.get('crab_tuck') else DIM)]
            for i, (text, tone) in enumerate(rows): put(pane, text, (x0, y0 + 24 * i), .52, tone)
            names = ('stability', 'thermal', 'slip', 'clearance', 'time'); x1, y1 = PANE_W - 300, PANE_H - 150
            put(pane, 'Stance cost J for the zone ahead', (x1, y1 - 12), .48)
            cost = self.cost or []
            for i, name in enumerate(names):
                value = cost[i + 1] if len(cost) > i + 1 else None; y = y1 + 10 + 24 * i; put(pane, name, (x1, y + 12), .45, DIM)
                cv2.rectangle(pane, (x1 + 90, y), (x1 + 250, y + 14), GRID, 1)
                if value is not None and math.isfinite(value) and value >= 0: cv2.rectangle(pane, (x1 + 90, y), (x1 + 90 + int(160 * min(1., value / 2.)), y + 14), BLUE, -1); put(pane, f'{value:.2f}', (x1 + 256, y + 12), .42)
                elif value is not None: put(pane, 'infeasible', (x1 + 96, y + 12), .42, RED)

        def strip(self, img, x0, y0, w, h, title, column, lo, hi, colour, extra=None, lines=()):
            cv2.rectangle(img, (x0, y0), (x0 + w, y0 + h), GRID, 1); put(img, title, (x0 + 6, y0 + 15), .45)
            if not self.history: return
            now = self.history[-1][0]; data = np.array(self.history, dtype=float)
            def to_xy(t, value): return np.column_stack([x0 + w - (now - t) / HISTORY_S * w, y0 + h - 2 - (np.clip(value, lo, hi) - lo) / (hi - lo) * (h - 22)]).astype(np.int32)
            for value, tone, label in lines:
                y = int(y0 + h - 2 - (value - lo) / (hi - lo) * (h - 22)); cv2.line(img, (x0 + 1, y), (x0 + w - 1, y), tone, 1); put(img, label, (x0 + 6, y + 12), .36, DIM if tone == GRID else tone)
            if extra is not None: cv2.polylines(img, [to_xy(data[:, 0], data[:, extra])], False, DIM, 1, cv2.LINE_AA)
            cv2.polylines(img, [to_xy(data[:, 0], data[:, column])], False, colour, 2, cv2.LINE_AA)
            put(img, f'{data[-1, column]:.1f}', (x0 + w - 70, y0 + 15), .48, colour)

        def compose(self, image, name, stamp):
            canvas = self.canvas; np.copyto(canvas, self.static)
            frame = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.step // 3, 3)[:, :image.width, ::-1]      # gz sends RGB
            if frame.shape[:2] != (PANE_H, PANE_W): frame = cv2.resize(frame, (PANE_W, PANE_H), interpolation=cv2.INTER_AREA)
            canvas[:PANE_H, :PANE_W] = frame; left = canvas[:PANE_H, :PANE_W]
            cv2.rectangle(left, (0, 0), (PANE_W - 1, 38), PANEL, -1)
            leg = (self.tour.get('leg') or '').upper(); put(left, f'GAZEBO  {CAMERA_LABEL[name]}', (14, 26), .6); put(left, f'ZONE: {leg}   STANCE: {self.tour.get("shape") or "-"}   PHASE: {self.tour.get("phase") or "-"}', (330, 26), .55, CYAN)
            pane = canvas[:PANE_H, PANE_W:]
            if self.cloud is not None:
                u, v, colour = self.cloud
                for du, dv in ((0, 0), (1, 0), (0, 1), (1, 1)): pane[v + dv, u + du] = colour
            d, margin, colour = self.draw_robot(pane); self.draw_panel(pane, d, margin, colour)
            hud = canvas[PANE_H:]; text = self.explanation.encode('ascii', 'replace').decode().replace('?', ' ')
            lines = [text[:172], text[172:344]] if len(text) > 172 else [text]
            put(hud, 'WHY:', (14, 24), .55, CYAN)
            for i, line in enumerate(lines): put(hud, line, (70, 24 + 20 * i), .5)
            self.strip(hud, 14, 56, 560, 132, 'Wheelbase L(t) [mm]  (grey: commanded)', 1, 200., 700., GREEN, extra=2, lines=((237., GRID, '237'), (540., GRID, '540'), (672., GRID, '672')))
            self.strip(hud, 594, 56, 560, 132, 'Static tip-over margin [deg]', 3, -10., 50., CYAN, lines=((0., RED, 'tip'),))
            self.strip(hud, 1174, 56, 440, 132, 'Hottest winding [C]', 4, 20., 80., AMBER, lines=((60., GRID, 'boost fades 60'), (75., RED, 'derate 75')))
            elapsed = stamp - self.t0
            put(hud, f'sim time {elapsed:6.1f} s', (1640, 84), .6); put(hud, f'playback {speed:g}x', (1640, 110), .6, AMBER if speed != 1 else TEXT)
            put(hud, f'policy: {self.tour.get("policy") or "-"}', (1640, 136), .5, DIM); put(hud, 'simulation, assumed parameters', (1640, 160), .42, DIM); put(hud, 'pose: ground truth', (1640, 180), .42, DIM)
            return canvas

        def on_image(self, name, m):
            if self.closed or not self.status.get('feedback_ready') or not self.tour: return
            stamp = m.header.stamp.sec + m.header.stamp.nsec * 1e-9
            if self.t0 is None: self.t0 = stamp
            if self.end_time is not None and stamp > self.end_time: self.close(); return
            target = int(round((stamp - self.t0) * FPS / speed))
            if target < self.next_frame: return                                   # a late duplicate from a camera change
            canvas = self.compose(m, name, stamp); self.source_frames += 1; self.camera_frames[name] += 1
            while self.next_frame <= target:
                self.writer.write(canvas)
                if self.next_frame < target: self.repeated += 1
                self.next_frame += 1
            self.last_stamp = stamp

        def close(self):
            if self.closed: return
            self.closed = True; self.writer.release()
            for name in CAMERA_LABEL: self.gz.unsubscribe('/morph/video/' + name)          # before interpreter teardown: a late callback aborts the process
            meta = {'frames': self.next_frame, 'fps': FPS, 'width': WIDTH, 'height': HEIGHT, 'duration_s': self.next_frame / FPS, 'playback_speed': speed, 'sim_time_covered_s': (self.last_stamp - self.t0) if self.t0 is not None and self.last_stamp else 0.,
                    'source_frames': self.source_frames, 'repeated_frames': self.repeated, 'frames_dropped_by_full_queue': self.dropped_queue, 'image_transport': 'Gazebo transport (direct)', 'frames_by_camera': self.camera_frames, 'camera_rate_sim_Hz': FPS / speed, 'intermediate_encoder': self.encoder,
                    'timing': 'output frame k shows simulation time t0 + k * speed / fps; a missing camera frame is filled by the next one that arrives, never interpolated', 'scope': 'simulation of an assumed-parameter model; one run'}
            (output / 'video.json').write_text(json.dumps(meta, indent=1))

    autonomous_tour.lifecycle.initialize()
    tour = autonomous_tour.AutonomousTour(); tour.set_parameters([Parameter('use_sim_time', value=True), Parameter('exit_on_finish', value=False)])
    recorder = Recorder(); executor = SingleThreadedExecutor(); executor.add_node(tour); executor.add_node(recorder)
    limit = float(os.environ.get('MORPH_TOUR_WALL_LIMIT_S', '2400')); start = time.monotonic()
    try:
        while rclpy.ok() and not recorder.closed and time.monotonic() - start < limit:
            executor.spin_once(timeout_sec=.005); recorder.drain()
    finally:
        if not tour.done_written and tour.start_time is not None: tour.finish(tour.now())
        recorder.close()
        summary_file = Path(os.environ['MORPH_BENCH_OUTPUT'])
        if summary_file.exists():
            summary = json.loads(summary_file.read_text()); meta = json.loads((output / 'video.json').read_text())
            meta.update(policy=summary.get('policy'), zones_passed=summary.get('zones_passed'), zones_total=summary.get('zones_total'), mission_time_s=summary.get('mission_time_s')); (output / 'video.json').write_text(json.dumps(meta, indent=1))
        executor.shutdown(); tour.destroy_node(); recorder.destroy_node(); rclpy.shutdown()


def main():
    if os.environ.get('MORPH_BENCH_OUTPUT'): record(); return
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--policy', default='proposed', choices=('fixed_compact', 'fixed_extended', 'reactive', 'proposed'))
    parser.add_argument('--speed', type=float, default=2., help='simulation seconds shown per video second (default 2)')
    parser.add_argument('--max-mb', type=float, default=45., help='size cap of the final file (ICRA 2027 allowed 20 MB and 180 s)'); parser.add_argument('--name', default='', help='run directory name under validation/performance')
    parser.add_argument('--output', default='', help='final MP4 (default videos/morph_conference_video_60fps.mp4)'); parser.add_argument('--wall-limit', type=float, default=2400.)
    parser.add_argument('--keep-raw', action='store_true', help='keep the near-lossless intermediate')
    args = parser.parse_args()
    if args.speed <= 0 or FPS / args.speed > 60: parser.error('speed must be at least 1 (the cameras render at 60 / speed Hz)')
    launch(args)


if __name__ == '__main__':
    main()
