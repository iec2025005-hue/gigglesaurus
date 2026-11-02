"""Camera acceptance over the whole measured interval, including silent tails."""
import math


def camera_acceptance(manifest, samples, checks, start, end):
    launch = manifest['launch']
    if str(launch['camera_enabled']).lower() == 'false':
        return {'required': False, 'passed': None, 'scope': 'Camera disabled; no camera performance claim'}
    if str(launch['camera_enabled']).lower() != 'true':
        raise ValueError('Invalid camera_enabled in run manifest')
    rate = float(launch['camera_rate'])
    width = int(launch['camera_width'])
    if not math.isfinite(rate) or not 1 <= rate <= 30 or width not in (320, 480, 640):
        raise ValueError('Invalid camera configuration in run manifest')
    wall_s = (end[0] - start[0]) / 1e9
    sim_s = (end[1] - start[1]) / 1e9
    streams = {}
    for name in ('camera', 'camera_info'):
        received = [r for r in samples.get(name, []) if start[0] <= r[0] <= end[0]]
        rows = [r for r in received if start[1] <= r[1] <= end[1]]
        sim_hz = len(rows) / sim_s if sim_s > 0 else 0.
        wall_hz = len(rows) / wall_s if wall_s > 0 else 0.
        increasing = all(b[1] > a[1] for a, b in zip(rows, rows[1:]))
        streams[name] = {'messages': len(rows), 'interval_sim_hz': sim_hz,
                         'interval_wall_hz': wall_hz, 'increasing_stamps': increasing,
                         'passed': len(rows) == len(received) and len(rows) >= 20 and increasing and sim_hz >= .9 * rate and wall_hz >= .9 * .95 * rate}
    geometry = bool(checks) and all(c.get('passed') and
        c['image']['width'] == width and c['image']['height'] == int(width * 9 / 16)
        for c in checks)
    matched = len(checks) >= max(20, .9 * streams['camera']['messages'])
    return {'required': True, 'requested_hz': rate, 'requested_width': width,
            'criteria': {'rate_fraction_min': .9, 'wall_rtf_factor': .95,
                         'matched_fraction_min': .9, 'messages_min': 20},
            'streams': streams, 'matched_pairs': len(checks), 'geometry_passed': geometry,
            'passed': geometry and matched and all(s['passed'] for s in streams.values()),
            'scope': 'Proposed software camera gate; full interval rates include silence; no upstream-drop or GPU-performance claim'}
