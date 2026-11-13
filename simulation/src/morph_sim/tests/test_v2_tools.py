"""V2 commits 4-5: dashboard radar, benchmark reducers, video recorder configuration, fixed-stance explanations."""
import importlib.util, json, math, sys, types
from pathlib import Path
import numpy as np
import pytest
from morph_sim import decision_core as dc, trench_crossing as tc
from morph_sim.design_physics import load_parameters

P = Path(__file__).resolve().parents[1]
WS = P.parents[1]


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path); module = importlib.util.module_from_spec(spec); sys.modules[name] = module; spec.loader.exec_module(module); return module


def cloud(points):
    data = np.asarray(points, dtype='<f4'); field = lambda name, offset: types.SimpleNamespace(name=name, offset=offset)
    return types.SimpleNamespace(fields=[field('x', 0), field('y', 4), field('z', 8)], point_step=12, width=len(data), height=1, data=data.tobytes())


def scan(height_of):
    """Points on a fan of rays from a sensor 0.455 m above the ground, in the sensor frame; height_of(x) is the terrain."""
    points = []
    for azimuth in np.linspace(-1.05, 1.05, 85):
        for distance in np.linspace(.4, 2.8, 40):
            x, y = distance * math.cos(azimuth), distance * math.sin(azimuth); points.append((x, y, height_of(x) - .455))
    return points


def test_dashboard_radar_tells_flat_ground_a_ramp_and_a_wall_apart():
    pytest.importorskip('flask'); server = load(WS / 'dashboard/server.py', 'morph_dashboard_server')
    flat = server.radar_from_cloud(cloud(scan(lambda x: 0.)))
    assert {s['state'] for s in flat} == {'FREE'}
    ramp = server.radar_from_cloud(cloud(scan(lambda x: max(0., (x - 1.) * math.tan(math.radians(16))))))
    middle = ramp[len(ramp) // 2]; assert middle['state'] == 'SLOPE' and middle['grade_deg'] == pytest.approx(16., abs=1.5)
    wall = scan(lambda x: 0.) + [(1.5, y, z - .455) for y in np.linspace(-1., 1., 60) for z in np.linspace(.2, .6, 6)]
    assert server.radar_from_cloud(cloud(wall))[len(ramp) // 2]['state'] == 'BLOCKED'
    drop = server.radar_from_cloud(cloud(scan(lambda x: -.25 if x > 1. else 0.)))
    assert drop[len(drop) // 2]['state'] == 'DROP'
    assert server.radar_from_cloud(cloud([])) and all(s['state'] == 'UNKNOWN' for s in server.radar_from_cloud(cloud([])))


def test_dashboard_only_forwards_whitelisted_events():
    pytest.importorskip('flask'); server = load(WS / 'dashboard/server.py', 'morph_dashboard_server')
    assert {'estop', 'reset', 'compact', 'normal', 'expand'} <= set(server.ALLOWED_EVENTS) and 'shutdown' not in server.ALLOWED_EVENTS


@pytest.mark.parametrize('shape', ['COMPACT', 'NORMAL', 'EXTENDED'])
def test_best_trench_stagger_matches_the_closed_form(shape):
    length = dc.wheelbase(shape); track = tc.TRACK
    closed = length * track / math.sqrt(4 * min(length, track) ** 2 + max(length, track) ** 2)
    yaw = math.atan(length / (2 * track) if length >= track else 2 * length / track)
    found_yaw, found = tc.best_yaw(dc.SHAPES[shape])
    assert found == pytest.approx(closed, abs=.0015) and found_yaw == pytest.approx(yaw, abs=math.radians(.3))
    assert tc.minimum_stagger(dc.SHAPES[shape], yaw) == pytest.approx(closed, abs=1e-9)
    assert dc.STEP_STAGGER[shape] == pytest.approx(closed, abs=.0015)


def test_a_fixed_stance_that_cannot_pass_is_explained_as_such():
    arena = json.loads((P / 'config/tetra_arena.json').read_text()); zone = next(z for z in arena['zones'] if z['id'] == 'trenches'); params = load_parameters(str(P / 'config/params.yaml'))
    context = dict(mass_kg=params['robot']['mass_kg'], drive=params['drive'], current_d=dc.SHAPES['COMPACT'])
    best, results = dc.choose(zone, context); fixed = [r for r in results if r['shape'] == 'COMPACT']
    text = dc.explain(zone, None, fixed)
    assert 'fixed stance COMPACT cannot pass' in text and 'clearance' in text and '178 mm' in text and 'NORMAL' not in text
    assert best['shape'] == 'EXTENDED' and 'rejected COMPACT' in dc.explain(zone, best, results)


def test_video_recorder_layout_and_camera_plan_are_consistent():
    recorder = load(WS / 'video_pipeline/record_conference_video.py', 'morph_video_recorder'); arena = json.loads((P / 'config/tetra_arena.json').read_text())
    assert (recorder.WIDTH, recorder.HEIGHT, recorder.FPS) == (1920, 1080, 60)
    assert 2 * recorder.PANE_W == recorder.WIDTH and recorder.PANE_H + recorder.HUD_H == recorder.HEIGHT
    zones = {z['id'] for z in arena['zones']}; assert set(recorder.CAMERA_BY_LEG) <= zones and set(recorder.CAMERA_BY_LEG.values()) <= set(recorder.CAMERA_LABEL)
    from morph_sim import assets
    import inspect
    source = inspect.getsource(assets.enrich_sdf); assert all(f"('{name}'," in source for name in recorder.CAMERA_LABEL)       # every camera the recorder uses exists in the model
    bridge = (P / 'config/bridge.yaml').read_text(); assert all(f'/morph/video/{name}' in bridge for name in recorder.CAMERA_LABEL)


def test_ablation_aggregate_reports_mean_std_and_zone_counts():
    suite = load(WS / 'benchmarks/run_ablation_suite.py', 'morph_ablation_suite')
    def run(policy, time_s, passed):
        row = {m: 1. for m in suite.METRICS}; row.update(policy=policy, mission_time_s=time_s, zones=[{'zone': 'ramps', 'success': passed, 'time_s': 10.}]); return row
    summary = suite.aggregate([run('proposed', 100., True), run('proposed', 110., True), run('fixed_compact', 90., False)])
    assert list(summary) == ['fixed_compact', 'proposed'] and summary['proposed']['mission_time_s']['mean'] == 105. and summary['proposed']['mission_time_s']['std'] == pytest.approx(math.sqrt(50.))
    assert summary['proposed']['zones']['ramps'] == {'passed': 2, 'runs': 2, 'mean_time_s': 10.} and summary['fixed_compact']['zones']['ramps']['passed'] == 0
    assert summary['fixed_compact']['mission_time_s']['std'] == 0.
