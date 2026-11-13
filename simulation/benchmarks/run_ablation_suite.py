#!/usr/bin/env python3
"""Four-policy ablation over the autonomous Tetra-Arena tour.

    cd simulation && python3 benchmarks/run_ablation_suite.py [--repeats 5] [--prefix v2abl] [--policies ...]

Policies (morph_sim/tour_core.py): fixed_compact, fixed_extended, reactive, proposed.
Runs are interleaved (repeat 1 of every policy, then repeat 2, ...) so machine drift affects all policies
alike and an interrupted suite still has balanced data. Each run is a fresh simulator launch through
src/morph_sim/tools/run_benchmark_case.py at real-time factor 1.0 (the physics step is not changed: the
launch only accepts the validated 1 or 2 ms steps). Existing runs with the same name are kept and reused.

Writes benchmarks/results/ablation_runs.csv, ablation_zones.csv and ablation_summary.json.
Every number is a simulation result for an assumed-parameter model.
"""
import argparse, csv, json, os, statistics, subprocess, sys
from pathlib import Path

WS = Path(__file__).resolve().parents[1]
RESULTS = WS / 'benchmarks/results'
METRICS = ['task_success', 'success_rate_percent', 'zones_passed', 'mission_time_s', 'distance_m', 'energy_J', 'cost_of_transport', 'min_stability_margin_deg',
           'peak_winding_C', 'rms_longitudinal_slip_percent', 'rms_lateral_slip_percent', 'max_soil_slip_percent']
POLICIES = ['fixed_compact', 'fixed_extended', 'reactive', 'proposed']


def run_one(name, policy, wall_limit):
    directory = WS / 'validation/performance' / name
    if not (directory / 'raw.json').exists():
        if directory.exists(): raise SystemExit(f'{directory} exists without a result; remove it or choose another --prefix')
        env = dict(os.environ, MORPH_PROTOCOL_SCRIPT='src/morph_sim/tools/run_tour.py', MORPH_TOUR_POLICY=policy, MORPH_TOUR_WALL_LIMIT_S=str(wall_limit), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST', FASTDDS_BUILTIN_TRANSPORTS='UDPv4')
        with (WS / 'log' / f'{name}.log').open('w') as log:
            subprocess.run([sys.executable, 'src/morph_sim/tools/run_benchmark_case.py', name, 'headless:=true', 'rviz:=false', 'arena:=tetra', 'camera_enabled:=true', 'camera_width:=320'],
                           cwd=WS, env=env, stdout=log, stderr=subprocess.STDOUT, check=False, timeout=wall_limit + 400)
    if not (directory / 'raw.json').exists(): return None
    return json.loads((directory / 'raw.json').read_text())


def aggregate(rows):
    summary = {}
    for policy in sorted({r['policy'] for r in rows}, key=POLICIES.index):
        mine = [r for r in rows if r['policy'] == policy]; entry = {'runs': len(mine)}
        for metric in METRICS:
            values = [float(r[metric]) for r in mine if r.get(metric) is not None]
            entry[metric] = {'mean': statistics.fmean(values), 'std': statistics.stdev(values) if len(values) > 1 else 0., 'min': min(values), 'max': max(values), 'n': len(values)} if values else None
        zones = {}
        for r in mine:
            for z in r['zones']: zones.setdefault(z['zone'], []).append(z)
        entry['zones'] = {zone: {'passed': sum(z['success'] for z in items), 'runs': len(items), 'mean_time_s': statistics.fmean(z['time_s'] for z in items)} for zone, items in zones.items()}
        summary[policy] = entry
    return summary


def main():
    parser = argparse.ArgumentParser(); parser.add_argument('--repeats', type=int, default=5); parser.add_argument('--prefix', default='v2abl')
    parser.add_argument('--policies', nargs='+', default=POLICIES, choices=POLICIES); parser.add_argument('--wall-limit', type=float, default=1300.)
    parser.add_argument('--collect-only', action='store_true', help='rebuild the result files from finished runs without launching anything')
    args = parser.parse_args(); RESULTS.mkdir(parents=True, exist_ok=True); (WS / 'log').mkdir(exist_ok=True); rows = []
    for repeat in range(1, args.repeats + 1):
        for policy in args.policies:
            name = f'{args.prefix}_{policy}_{repeat}'; directory = WS / 'validation/performance' / name
            data = json.loads((directory / 'raw.json').read_text()) if args.collect_only and (directory / 'raw.json').exists() else None if args.collect_only else run_one(name, policy, args.wall_limit)
            if data is None or 'zones' not in data:
                print(f'{name}: no result', flush=True); continue
            cleanup = json.loads((directory / 'cleanup.json').read_text()) if (directory / 'cleanup.json').exists() else {}
            rows.append(dict(data, run=name, repeat=repeat, clean_exit=cleanup.get('clean_exit')))
            print(f"{name}: zones {data['zones_passed']}/{data['zones_total']}, {data['mission_time_s']:.0f} s, CoT {data['cost_of_transport']:.3f}, min margin {data['min_stability_margin_deg']:.1f} deg, peak {data['peak_winding_C']:.1f} C", flush=True)
            with (RESULTS / 'ablation_runs.csv').open('w', newline='') as handle:
                writer = csv.writer(handle); writer.writerow(['run', 'policy', 'repeat', 'clean_exit'] + METRICS)
                for r in rows: writer.writerow([r['run'], r['policy'], r['repeat'], r['clean_exit']] + [r.get(m) for m in METRICS])
            with (RESULTS / 'ablation_zones.csv').open('w', newline='') as handle:
                writer = csv.writer(handle); writer.writerow(['run', 'policy', 'repeat', 'zone', 'kind', 'success', 'reason', 'time_s'])
                for r in rows:
                    for z in r['zones']: writer.writerow([r['run'], r['policy'], r['repeat'], z['zone'], z['kind'], z['success'], z['reason'], round(z['time_s'], 2)])
            (RESULTS / 'ablation_summary.json').write_text(json.dumps({'scope': 'Simulation of an assumed-parameter model; pose from ground truth; real-time factor 1.0', 'repeats_requested': args.repeats,
                                                                       'runs': [r['run'] for r in rows], 'policies': aggregate(rows)}, indent=1))
    print('ABLATION_DONE', len(rows), 'runs')


if __name__ == '__main__':
    main()
