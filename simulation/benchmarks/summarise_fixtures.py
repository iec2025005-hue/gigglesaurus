#!/usr/bin/env python3
"""Reduce the single-fixture runs (ramps, trenches, 3-DOF command test) to one table of outcomes.

    cd simulation && source install/setup.bash && python3 benchmarks/summarise_fixtures.py [--prefix v2s3_]

Reads validation/performance/<prefix>{ramp,trench}*/raw.json (protocol tools/baseline_v2.py, mode `fixture`)
and writes benchmarks/results/fixture_results.json. Outcome rules, applied to the drive phase only:

    ramp     climbed  when the body rose by at least 90 % of the ramp's rise
    trench   crossed  for every trench whose far edge is behind the rearmost wheel at the end of the run

Every row is one simulation run of an assumed-parameter model.
"""
import argparse, json, math
from pathlib import Path
import numpy as np
from morph_sim.trench_crossing import wheel_offsets

WS = Path(__file__).resolve().parents[1]


def argument(args, key, default=0.):
    for item in args:
        if item.startswith(key + ':='): return float(item.split(':=')[1])
    return default


def reduce(directory, arena):
    raw = json.loads((directory / 'raw.json').read_text()); summary = json.loads((directory / 'summary.json').read_text()) if (directory / 'summary.json').exists() else {}
    launch = summary.get('launch_args', []); phases = {p['phase']: p for p in raw['phases']}; drive = phases.get('fixture_drive')
    if drive is None: return None
    odom = np.array(raw['samples']['odom'], dtype=float); status = raw['samples']['status']; power = np.array(raw['samples']['power'], dtype=float)
    t0, t1 = drive['sim_start_s'], drive['sim_end_s']; o = odom[(odom[:, 0] >= t0) & (odom[:, 0] <= t1)]; s = [r for r in status if t0 <= r[0] <= t1]; p = power[(power[:, 0] >= t0) & (power[:, 0] <= t1)]
    wheelbase = np.array([.54 + 8 * r[3] for r in s]); pitch = np.array([r[9] for r in s]); roll = np.array([r[10] for r in s])
    energy = float(np.sum(p[:-1, 1] * p[:-1, 2] * np.diff(p[:, 0]))) if len(p) > 1 else None
    row = {'run': directory.name, 'spawn': [argument(launch, 'spawn_x'), argument(launch, 'spawn_y'), argument(launch, 'spawn_yaw')], 'events': [e['event'] for e in raw['events']],
           'command': [round(c, 3) for c in drive['command']], 'drive_time_s': round(t1 - t0, 1), 'timed_out': drive['timed_out'], 'x_start_m': round(float(o[0, 2]), 3), 'x_end_m': round(float(o[-1, 2]), 3),
           'height_gain_m': round(float(o[:, 4].max() - o[0, 4]), 3), 'peak_pitch_deg': round(float(np.abs(pitch).max()), 1), 'peak_roll_deg': round(float(np.abs(roll).max()), 1),
           'wheelbase_start_mm': round(1e3 * float(wheelbase[0])), 'wheelbase_min_mm': round(1e3 * float(wheelbase.min())), 'wheelbase_end_mm': round(1e3 * float(wheelbase[-1])),
           'peak_winding_C': round(float(p[:, 3].max()), 1) if len(p) else None, 'peak_torque_Nm': round(float(np.abs(p[:, 4:8]).max()), 3) if len(p) else None, 'energy_J': round(energy) if energy is not None else None,
           'final_speed_m_s': round(float(np.hypot(o[-25:, 6], o[-25:, 7]).mean()), 3), 'estop_at_end': bool(s[-1][8]) if s else None}
    if 'ramp' in directory.name:
        lane = next(z for z in arena['zones'] if z['id'] == 'ramps')['ramps']; match = min(lane, key=lambda r: abs(r['y'] - row['spawn'][1]))
        row.update(kind='ramp', slope_deg=match['angle_deg'], climbed=bool(row['height_gain_m'] >= .9 * match['rise_m']))
    else:
        trenches = next(z for z in arena['zones'] if z['id'] == 'trenches')['trenches']; yaw = float(o[-1, 5]); d = (float(wheelbase[-1]) - .54) / 8
        rear = float(o[-1, 2]) + min(wheel_offsets(d, yaw))
        row.update(kind='trench', yaw_deg=round(math.degrees(yaw), 1), crossed_mm=[round(1e3 * t['width_m']) for t in trenches if rear > t['x'] + t['width_m'] / 2 + .02], offered_mm=[round(1e3 * t['width_m']) for t in trenches])
    return row


def holonomic(directory):
    """Per-axis response of the 3-DOF command test (protocol mode `pivot`): what moved, and what leaked into the other axes."""
    raw = json.loads((directory / 'raw.json').read_text()); odom = np.array(raw['samples']['odom'], dtype=float); status = raw['samples']['status']; rows = []
    for phase in raw['phases']:
        command = phase['command']
        if not any(abs(c) > 1e-3 for c in command): continue
        t0, t1 = phase['sim_start_s'], phase['sim_end_s']; o = odom[(odom[:, 0] >= t0) & (odom[:, 0] <= t1)]; steady = o[o[:, 0] >= t1 - 1.5]; s = [r for r in status if t0 <= r[0] <= t1]
        yaw = np.unwrap(o[:, 5]); c, sn = math.cos(yaw[0]), math.sin(yaw[0]); dx, dy = o[-1, 2] - o[0, 2], o[-1, 3] - o[0, 3]
        steering = np.array([r[11] for r in s], dtype=float); final = steering[-1]; error = np.abs(steering - final).max(axis=1); times = np.array([r[0] for r in s]) - t0
        def settle(band):
            outside = np.nonzero(error > band)[0]; return round(float(times[min(outside[-1] + 1, len(times) - 1)]), 2) if len(outside) else 0.
        rows.append({'phase': phase['phase'], 'command_vx_vy_wz': [round(v, 3) for v in command], 'steady_vx_vy_wz': [round(float(steady[:, k].mean()), 3) for k in (6, 7, 9)],
                     'displacement_body_m': [round(float(c * dx + sn * dy), 3), round(float(-sn * dx + c * dy), 3)], 'heading_change_deg': round(math.degrees(float(yaw[-1] - yaw[0])), 2),
                     'steering_final_deg': [round(float(a), 1) for a in final], 'steering_within_10deg_s': settle(10.), 'steering_within_2deg_s': settle(2.)})
    return rows


def main():
    parser = argparse.ArgumentParser(); parser.add_argument('--prefix', default='v2s3_'); args = parser.parse_args()
    arena = json.loads((WS / 'src/morph_sim/config/tetra_arena.json').read_text()); rows = []
    for directory in sorted((WS / 'validation/performance').glob(args.prefix + '*')):
        if not any(k in directory.name for k in ('ramp', 'trench')) or not (directory / 'raw.json').exists(): continue
        row = reduce(directory, arena)
        if row: rows.append(row); print(row['run'], row.get('slope_deg', ''), 'climbed' if row.get('climbed') else row.get('crossed_mm', 'not climbed'), row['events'], f"L {row['wheelbase_start_mm']}->{row['wheelbase_min_mm']} mm, pitch {row['peak_pitch_deg']}")
    pivot = [holonomic(d) for d in sorted((WS / 'validation/performance').glob(args.prefix + 'pivot*')) if (d / 'raw.json').exists()]
    for row in (pivot[-1] if pivot else []): print(row['phase'], row['command_vx_vy_wz'], '->', row['steady_vx_vy_wz'], 'heading', row['heading_change_deg'], 'steer 10/2 deg', row['steering_within_10deg_s'], row['steering_within_2deg_s'], row['steering_final_deg'])
    out = WS / 'benchmarks/results/fixture_results.json'; out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps({'scope': 'Simulation of an assumed-parameter model; one run per row', 'rules': __doc__.split('Outcome rules')[1].split('Every row')[0].strip(), 'rows': rows, 'holonomic': pivot[-1] if pivot else []}, indent=1))
    print('wrote', out.relative_to(WS))


if __name__ == '__main__':
    main()
