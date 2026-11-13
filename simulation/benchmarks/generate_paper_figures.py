#!/usr/bin/env python3
"""IEEE-format figures and the LaTeX ablation table from the benchmark logs.

    cd simulation && python3 benchmarks/generate_paper_figures.py [--prefix v2abl] [--repeat 1]

Reads benchmarks/results/ablation_summary.json, the per-run time series validation/performance/<run>/raw.csv,
the on-the-fly morph run and the Phase 0 baseline. Writes PDF and 300-dpi PNG to benchmarks/figures/ and
reports/table1_ablation.tex. Single-column width 3.5 in, serif fonts.
Figures show simulation results for an assumed-parameter model; the captions written beside each figure
state the number of runs.
"""
import argparse, csv, json, math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

WS = Path(__file__).resolve().parents[1]
OUT = WS / 'benchmarks/figures'
POLICIES = ['fixed_compact', 'fixed_extended', 'reactive', 'proposed']
LABEL = {'fixed_compact': 'Fixed Compact', 'fixed_extended': 'Fixed Extended', 'reactive': 'Reactive stop-morph', 'proposed': 'Proposed predictive'}
COLOUR = {'fixed_compact': '#b2182b', 'fixed_extended': '#2166ac', 'reactive': '#e08214', 'proposed': '#1b7837'}
STYLE = {'fixed_compact': ':', 'fixed_extended': '--', 'reactive': '-.', 'proposed': '-'}
ZONE_SHADE = {'trench': '#d9d9d9', 'soil': '#f1e2b8', 'ramp': '#cfe3f1', 'passage': '#f6cfcf', 'rough': '#dcd0c0'}
plt.rcParams.update({'font.family': 'serif', 'font.serif': ['DejaVu Serif', 'Times New Roman', 'Times'], 'font.size': 7, 'axes.labelsize': 7, 'axes.titlesize': 7, 'legend.fontsize': 6,
                     'xtick.labelsize': 6, 'ytick.labelsize': 6, 'lines.linewidth': .9, 'axes.linewidth': .5, 'savefig.dpi': 300, 'pdf.fonttype': 42, 'figure.dpi': 150})
WIDTH = 3.5


def series(run):
    path = WS / 'validation/performance' / run / 'raw.csv'
    if not path.exists(): return None
    with path.open() as handle:
        reader = csv.DictReader(handle); rows = list(reader)
    def column(name, kind=float):
        out = []
        for r in rows:
            try: out.append(kind(r[name]) if r[name] not in ('', 'None') else (math.nan if kind is float else ''))
            except ValueError: out.append(math.nan if kind is float else '')
        return np.array(out) if kind is float else out
    return {'rows': rows, 'col': column}


def save(fig, name, caption):
    OUT.mkdir(parents=True, exist_ok=True); fig.savefig(OUT / f'{name}.pdf', bbox_inches='tight', pad_inches=.02); fig.savefig(OUT / f'{name}.png', bbox_inches='tight', pad_inches=.02); plt.close(fig)
    (OUT / f'{name}.caption.txt').write_text(caption + '\n'); print('wrote', name)


def shade(ax, arena, axis='x'):
    for zone in arena['zones']:
        if zone['kind'] in ZONE_SHADE: ax.axvspan(zone['bounds'][0], zone['bounds'][1], color=ZONE_SHADE[zone['kind']], lw=0, zorder=0)


def fig_trajectory(arena, run):
    data = series(run)
    if data is None: return
    x, y, L = data['col']('x'), data['col']('y'), data['col']('wheelbase_m') * 1e3; yaw = data['col']('yaw')
    fig, (top, bottom) = plt.subplots(2, 1, figsize=(WIDTH, 2.5), sharex=True, gridspec_kw={'height_ratios': [1.25, 1], 'hspace': .12})
    shade(top, arena); shade(bottom, arena)
    points = np.column_stack([x, y]).reshape(-1, 1, 2); segments = np.concatenate([points[:-1], points[1:]], axis=1)
    lc = LineCollection(segments, cmap='viridis', norm=plt.Normalize(237, 672), linewidth=1.4); lc.set_array(L[:-1]); top.add_collection(lc)
    step = max(1, len(x) // 26)
    for i in range(0, len(x), step):                       # body heading ticks show the yawed and sideways legs
        top.plot([x[i], x[i] + .35 * math.cos(yaw[i])], [y[i], y[i] + .35 * math.sin(yaw[i])], color='k', lw=.35)
    top.set_ylim(-.35, .62); top.set_ylabel('y [m]'); cb = fig.colorbar(lc, ax=top, pad=.01, fraction=.035); cb.set_label('wheelbase [mm]', fontsize=6)
    for zone in arena['zones']:
        if zone['kind'] in ZONE_SHADE: top.text((zone['bounds'][0] + zone['bounds'][1]) / 2, .5 if zone['id'] != 'pinch' else .58, {'corridor': 'corr.', 'trenches': 'trenches'}.get(zone['id'], zone['id']), ha='center', va='center', fontsize=4.5)
    bottom.plot(x, L, color='k'); bottom.set_ylabel('wheelbase $L$ [mm]'); bottom.set_xlabel('distance along the course $x$ [m]'); bottom.set_ylim(200, 700); bottom.set_xlim(arena['spawn'][0] - .5, arena['x_end'] - .5)
    fig.colorbar(lc, ax=bottom, pad=.01, fraction=.035).ax.set_visible(False)
    save(fig, 'fig1_arena_trajectory', f'Path of the proposed policy through the Tetra-Arena, coloured by wheelbase; short black ticks show body heading (yawed over the trenches, sideways through the 380 mm pinch). Run {run}; simulation, n = 1.')


def fig_stability(arena, runs):
    fig, axes = plt.subplots(4, 1, figsize=(WIDTH, 4.3), sharex=True, gridspec_kw={'hspace': .12})
    names = [('stability_margin_deg', 'tip margin [deg]'), ('pitch_deg', 'pitch [deg]'), ('winding_max_C', 'winding [°C]'), ('wheelbase_m', 'wheelbase [mm]')]
    for ax in axes: shade(ax, arena)
    for policy, run in runs.items():
        data = series(run)
        if data is None: continue
        x = data['col']('x')
        for ax, (name, label) in zip(axes, names):
            v = data['col'](name) * (1e3 if name == 'wheelbase_m' else 1.)
            ax.plot(x, v, color=COLOUR[policy], ls=STYLE[policy], label=LABEL[policy]); ax.set_ylabel(label)
    axes[0].axhline(0, color='k', lw=.4); axes[-1].set_xlabel('distance along the course $x$ [m]'); axes[0].legend(ncol=2, loc='lower left', frameon=False, handlelength=2.2)
    save(fig, 'fig2_stability_thermal_curves', 'Static tip-over margin, body pitch, hottest motor winding and wheelbase along the course for the four policies (one run each). A policy that fails a zone is repositioned past it, which shows as a jump.')


def fig_drift():
    onfly = WS / 'reports/baseline_data/v2s3_onfly_metrics.json'; base = WS / 'reports/baseline_data/full_headless_1/metrics.json'; fixed = WS / 'reports/baseline_data/full_headless_v2s3/metrics.json'
    if not (onfly.exists() and base.exists() and fixed.exists()): return
    o = json.loads(onfly.read_text()); b = json.loads(base.read_text()); f = json.loads(fixed.read_text())
    fig, (left, right) = plt.subplots(1, 2, figsize=(WIDTH, 1.9), gridspec_kw={'wspace': .6, 'top': .72})
    cases = [('morph_normal_to_extended', '540→672'), ('morph_extended_to_compact', '672→237'), ('morph_compact_to_extended', '237→672'), ('morph_extended_to_normal', '672→540')]
    idx = np.arange(len(cases)); before = [1e3 * b[k]['body_displacement_m'] for k, _ in cases]; after = [1e3 * f[k]['body_displacement_m'] for k, _ in cases]
    left.bar(idx - .2, before, .4, color='#b2182b', label='commanded-rate feedforward'); left.bar(idx + .2, after, .4, color='#1b7837', label='measured-rate + wheel-angle lock')
    left.set_yscale('log'); left.set_ylim(.05, 400); left.set_xticks(idx); left.set_xticklabels([c[1] for c in cases], rotation=30, ha='right'); left.set_ylabel('body creep at rest [mm]'); left.legend(frameon=False, fontsize=4.6, loc='lower left', bbox_to_anchor=(-.32, 1.13), handlelength=1.)
    left.set_title('(a) shape change at rest', pad=2)
    moving = [('straight_morph', 'line\n540↔672'), ('straight_compact', 'line\n540→237'), ('arc_morph', 'arc\n540↔672')]
    lateral = [o[k]['lateral_deviation_max_mm'] if o[k]['lateral_deviation_max_mm'] is not None else math.nan for k, _ in moving]; along = [o[k]['path_difference_max_mm'] for k, _ in moving]
    j = np.arange(len(moving)); right.bar(j - .2, [0 if math.isnan(v) else v for v in lateral], .4, color='#2166ac', label='lateral deviation'); right.bar(j + .2, along, .4, color='#e08214', label='path difference vs. no-morph run')
    right.axhline(5, color='k', lw=.5, ls='--'); right.text(len(moving) - .55, 6.5, '5 mm', fontsize=5, ha='right')
    right.set_yscale('log'); right.set_ylim(.1, 400); right.set_xticks(j); right.set_xticklabels([m[1] for m in moving]); right.set_ylabel('deviation at 0.3 m/s [mm]'); right.legend(frameon=False, fontsize=4.6, loc='lower left', bbox_to_anchor=(-.32, 1.13), handlelength=1.); right.set_title('(b) shape change while driving', pad=2)
    for axis in (left, right): axis.tick_params(axis='x', labelsize=5)
    save(fig, 'fig3_morph_drift_comparison', 'Parasitic motion during a wheelbase change. (a) Body creep at standstill before and after the measured-rate feedforward and wheel-angle lock. (b) Open-loop drives at 0.3 m/s with a shape change against the same drive without one: lateral deviation stays below 1 mm; the path difference is along the track (the morphing run tracks commanded speed more closely). Simulation, n = 1.')


def fig_cost(arena, run):
    data = series(run)
    if data is None: return
    x = data['col']('x'); terms = [('cost_stability', 'stability', '#2166ac'), ('cost_thermal', 'thermal', '#b2182b'), ('cost_slip', 'traction', '#e08214'), ('cost_clearance', 'clearance', '#762a83'), ('cost_time', 'morph time', '#7f7f7f')]
    values = [np.clip(np.nan_to_num(data['col'](name), nan=0.), 0, None) for name, _, _ in terms]
    fig, (top, bottom) = plt.subplots(2, 1, figsize=(WIDTH, 2.6), sharex=True, gridspec_kw={'height_ratios': [1.6, 1], 'hspace': .1})
    shade(top, arena); shade(bottom, arena)
    top.stackplot(x, values, colors=[t[2] for t in terms], labels=[t[1] for t in terms], linewidth=0, alpha=.9); top.set_ylabel('stance cost $J$'); top.legend(ncol=5, frameon=False, loc='lower left', bbox_to_anchor=(0., 1.0), fontsize=5.2, handlelength=1., columnspacing=.8)
    bottom.plot(x, data['col']('wheelbase_m') * 1e3, color='k'); bottom.set_ylabel('wheelbase [mm]'); bottom.set_xlabel('distance along the course $x$ [m]'); bottom.set_ylim(200, 700)
    save(fig, 'fig4_cost_breakdown', f'Terms of the stance cost for the decision in force along the course (proposed policy, run {run}), with the resulting wheelbase. Simulation, n = 1.')


def table(summary):
    rows = [('Task success rate', 'success_rate_percent', '\\%', 0), ('Mission time', 'mission_time_s', 's', 0), ('Cost of transport', 'cost_of_transport', '--', 2), ('Min. tip-over margin', 'min_stability_margin_deg', 'deg', 1),
            ('Peak winding temp.', 'peak_winding_C', '$^\\circ$C', 1), ('RMS long. slip', 'rms_longitudinal_slip_percent', '\\%', 1), ('RMS lat. slip', 'rms_lateral_slip_percent', '\\%', 1)]
    policies = [p for p in POLICIES if p in summary['policies']]; n = {p: summary['policies'][p]['runs'] for p in policies}
    lines = ['% Auto-generated by benchmarks/generate_paper_figures.py. Simulation results for an assumed-parameter model.', '\\begin{table}[t]', '\\centering',
             '\\caption{Ablation over the autonomous arena tour (mean $\\pm$ standard deviation; $n = ' + ', '.join(str(n[p]) for p in policies) + '$ runs). Zones a policy cannot pass are recorded as failed and the robot is repositioned past them, so mission time and cost of transport of failing policies cover less terrain.}',
             '\\label{tab:ablation}', '\\begin{tabular}{l' + 'c' * len(policies) + '}', '\\toprule', 'Metric & ' + ' & '.join(LABEL[p] for p in policies) + ' \\\\', '\\midrule']
    for label, key, unit, digits in rows:
        cells = []
        for p in policies:
            m = summary['policies'][p].get(key)
            cells.append('--' if not m else f"{m['mean']:.{digits}f} $\\pm$ {m['std']:.{digits}f}")
        lines.append(f"{label} [{unit}] & " + ' & '.join(cells) + ' \\\\')
    zones = list(next(iter(summary['policies'].values()))['zones'])
    lines.append('\\midrule'); lines.append('Zones passed [runs] & ' + ' & '.join(' ' for _ in policies) + ' \\\\')
    for zone in zones:
        lines.append(f"\\quad {zone} & " + ' & '.join(f"{summary['policies'][p]['zones'][zone]['passed']}/{summary['policies'][p]['zones'][zone]['runs']}" if zone in summary['policies'][p]['zones'] else '--' for p in policies) + ' \\\\')
    lines += ['\\bottomrule', '\\end{tabular}', '\\end{table}']
    (WS / 'reports/table1_ablation.tex').write_text('\n'.join(lines) + '\n'); print('wrote table1_ablation.tex')


def main():
    parser = argparse.ArgumentParser(); parser.add_argument('--prefix', default='v2abl'); parser.add_argument('--repeat', type=int, default=1); parser.add_argument('--proposed-run', default=None)
    args = parser.parse_args(); arena = json.loads((WS / 'src/morph_sim/config/tetra_arena.json').read_text())
    runs = {p: f'{args.prefix}_{p}_{args.repeat}' for p in POLICIES}; runs = {p: r for p, r in runs.items() if (WS / 'validation/performance' / r / 'raw.csv').exists()}
    proposed = args.proposed_run or runs.get('proposed')
    if proposed: fig_trajectory(arena, proposed); fig_cost(arena, proposed)
    if runs: fig_stability(arena, runs)
    fig_drift()
    summary = WS / 'benchmarks/results/ablation_summary.json'
    if summary.exists(): table(json.loads(summary.read_text()))


if __name__ == '__main__':
    main()
