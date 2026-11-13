#!/usr/bin/env python3
"""Derive Phase-0 baseline metrics and plots from a baseline_v2.py raw.json.

    python3 src/morph_sim/tools/baseline_v2_report.py RAW.json OUT_DIR

Differentiation: ground-truth odometry (50 Hz sim time) is resampled to a uniform 50 Hz
grid, smoothed with a centred moving average of WINDOW_S, then central-differenced.
Acceleration and jerk are therefore band-limited estimates; the window is reported with
every number. Unfiltered peaks are reported alongside to show the sensitivity.
"""
import json, math, sys
from pathlib import Path
import numpy as np

WINDOW_S = .10
RATE = 50.


def smooth(x, n):
    if n <= 1: return x
    k = np.ones(n) / n
    return np.convolve(np.pad(x, (n // 2, n - 1 - n // 2), mode='edge'), k, mode='valid')


def derive(t, v, window_s=WINDOW_S):
    grid = np.arange(t[0], t[-1], 1 / RATE)
    if len(grid) < 8: return None
    n = max(1, int(round(window_s * RATE)))
    vs = smooth(np.interp(grid, t, v), n)
    a = smooth(np.gradient(vs, grid), n)
    j = smooth(np.gradient(a, grid), n)
    return grid, vs, a, j


def main():
    raw = json.loads(Path(sys.argv[1]).read_text()); out = Path(sys.argv[2]); out.mkdir(parents=True, exist_ok=True)
    od = np.array(raw['samples']['odom'], dtype=float); st = raw['samples']['status']
    imu = np.array(raw['samples']['imu'], dtype=float)
    phases = {p['phase']: p for p in raw['phases']}
    # Odometry stamps may repeat when the bridge delivers bursts; keep strictly increasing samples.
    keep = np.concatenate([[True], np.diff(od[:, 0]) > 1e-6]); od = od[keep]
    t, x, y, yaw, vx, vy, wz = od[:, 0], od[:, 2], od[:, 3], od[:, 5], od[:, 6], od[:, 7], od[:, 9]
    speed = np.hypot(vx, vy)
    # Check the twist frame: body-frame vx should match the world displacement projected on heading.
    fd = (np.gradient(x, t) * np.cos(yaw) + np.gradient(y, t) * np.sin(yaw))
    frame_check = float(np.nanmedian(np.abs(smooth(fd, 5) - smooth(vx, 5))))
    st_t = np.array([r[0] for r in st]); st_cmd = np.array([r[1] if r[1] is not None else np.nan for r in st])
    st_stage = np.array([r[3] if r[3] is not None else np.nan for r in st]); st_rate = np.array([r[5] or 0. for r in st])
    st_align = np.array([r[6] if r[6] is not None else np.nan for r in st])

    def window(name):
        p = phases[name]; m = (t >= p['sim_start_s']) & (t <= p['sim_end_s']); return p, m

    def stop_metrics(name, threshold=.02, start=None):
        p, m = window(name); t0 = p['sim_start_s'] if start is None else start
        m = (t >= t0) & (t <= p['sim_end_s'])
        if not m.any(): return None
        below = np.where(speed[m] < threshold)[0]
        i0 = np.where(m)[0][0]
        if not len(below): return {'stopped': False, 'speed_at_start_m_s': float(speed[i0])}
        i1 = np.where(m)[0][below[0]]
        return {'stopped': True, 'speed_at_start_m_s': float(speed[i0]), 'stop_time_s': float(t[i1] - t[i0]),
                'stop_distance_m': float(math.hypot(x[i1] - x[i0], y[i1] - y[i0])), 'threshold_m_s': threshold}

    def motion(name, signal=None):
        p, m = window(name); sig = vx if signal is None else signal
        d = derive(t[m], sig[m]); raw_d = derive(t[m], sig[m], 0.)
        if d is None: return None
        g, vs, a, j = d
        tail = g > g[-1] - 1.
        return {'peak_abs_value': float(np.max(np.abs(sig[m]))), 'steady_mean_last_1s': float(np.mean(vs[tail])),
                'steady_std_last_1s': float(np.std(sig[m][t[m] > t[m][-1] - 1.])),
                'peak_abs_accel_filtered': float(np.max(np.abs(a))), 'peak_abs_jerk_filtered': float(np.max(np.abs(j))),
                'peak_abs_accel_unfiltered': float(np.max(np.abs(raw_d[2]))), 'peak_abs_jerk_unfiltered': float(np.max(np.abs(raw_d[3]))),
                'rise_time_to_90pct_s': (float(g[np.argmax(np.abs(vs) >= .9 * abs(np.mean(vs[tail])))] - g[0]) if abs(np.mean(vs[tail])) > .02 else None),
                'filter_window_s': WINDOW_S, 'rtf': p['rtf']}

    def command_level(name):
        p = phases[name]; m = (st_t >= p['sim_start_s']) & (st_t <= p['sim_end_s']) & np.isfinite(st_cmd)
        if m.sum() < 6: return None
        tt = st_t[m]; keep2 = np.concatenate([[True], np.diff(tt) > 1e-6]); tt = tt[keep2]; cc = st_cmd[m][keep2]
        a = np.gradient(cc, tt); j = np.gradient(a, tt)
        return {'peak_command_speed': float(np.max(cc)), 'peak_abs_command_accel': float(np.max(np.abs(a))),
                'peak_abs_command_jerk': float(np.max(np.abs(j))), 'status_rate_hz': float(1 / np.median(np.diff(tt))),
                'note': 'From /morph/status command_speed (unsigned planar magnitude, ~20 Hz); jerk is sampling-limited'}

    def morph(name):
        p = phases[name]; pre = phases.get(name + '_pre'); latch = phases.get(name + '_latch')
        t0 = latch['sim_start_s'] if latch else p['sim_start_s']; t1 = p['sim_end_s']
        m = (st_t >= t0) & (st_t <= t1) & np.isfinite(st_stage)
        if m.sum() < 3: return None
        s = st_stage[m]; ts = st_t[m]; moving = np.where(np.abs(st_rate[m]) > 0)[0]
        mo = (t >= t0) & (t <= t1)
        res = {'timed_out': p['timed_out'], 'event_to_settled_s': float(t1 - t0), 'stage_start_m': float(s[0]), 'stage_end_m': float(s[-1]),
               'wheelbase_start_mm': float(1000 * (.54 + 8 * s[0])), 'wheelbase_end_mm': float(1000 * (.54 + 8 * s[-1])),
               'event_to_morph_start_s': (float(ts[moving[0]] - t0) if len(moving) else None),
               'morph_active_s': (float(ts[moving[-1]] - ts[moving[0]]) if len(moving) else None),
               'mean_stage_rate_m_s': (float(abs(s[moving[-1]] - s[moving[0]]) / max(ts[moving[-1]] - ts[moving[0]], 1e-9)) if len(moving) > 1 else None),
               'body_speed_at_morph_start_m_s': (float(np.interp(ts[moving[0]], t, speed)) if len(moving) else None),
               'command_speed_at_morph_start_m_s': (float(st_cmd[m][moving[0]]) if len(moving) else None),
               'body_speed_min_m_s': float(np.min(speed[mo])) if mo.any() else None, 'body_speed_max_m_s': float(np.max(speed[mo])) if mo.any() else None,
               'body_displacement_m': float(math.hypot(x[mo][-1] - x[mo][0], y[mo][-1] - y[mo][0])) if mo.any() else None,
               'max_steering_alignment_error_rad': float(np.nanmax(st_align[m])), 'rtf': p['rtf']}
        d = derive(t[mo], vx[mo]) if mo.sum() > 10 else None
        if d: res['peak_abs_accel_filtered'] = float(np.max(np.abs(d[2]))); res['peak_abs_jerk_filtered'] = float(np.max(np.abs(d[3])))
        return res

    metrics = {'source': str(sys.argv[1]), 'protocol_error': raw.get('error'), 'mode': raw['mode'], 'sim_s': raw['sim_s'], 'wall_s': raw['wall_s'],
               'mean_rtf': raw['mean_rtf'], 'rtf_by_phase': {p['phase']: round(p['rtf'], 4) for p in raw['phases']},
               'odom_rate_hz': float(1 / np.median(np.diff(t))), 'imu_rate_hz': (float(1 / np.median(np.diff(imu[:, 0])[np.diff(imu[:, 0]) > 0])) if len(imu) > 2 else None),
               'twist_frame_check_median_abs_diff_m_s': frame_check, 'filter_window_s': WINDOW_S}
    if raw['mode'] == 'full':
        ev = {e['event']: e['sim_s'] for e in raw['events']}
        metrics.update({
            'forward_0p6': motion('fwd_0p6'), 'forward_0p6_command': command_level('fwd_0p6'), 'forward_0p6_stop': stop_metrics('fwd_0p6_stop'),
            'forward_0p6_stop_motion': motion('fwd_0p6_stop'), 'forward_0p6_stop_command': command_level('fwd_0p6_stop'),
            'forward_overcap_2p0': motion('fwd_overcap_2p0'), 'reverse_0p6': motion('rev_0p6'),
            'estop_from_0p6': stop_metrics('estop_hold', start=[e['sim_s'] for e in raw['events'] if e['event'] == 'estop'][0]) if 'estop' in ev else None,
            'estop_motion': motion('estop_hold'),
            'motor_limit_12rad_s': motion('motor_limit_12rad_s'), 'motor_limit_stop': stop_metrics('motor_limit_stop'),
            'spin_0p7_yaw_rate': motion('spin_0p7', wz), 'crab_0p3_vy': motion('crab_0p3', vy),
            'flip_fwd': motion('fwd_to_rev_flip'), 'flip_rev': motion('fwd_to_rev_flip_rev'),
        })
        p, m = window('spin_0p7')
        metrics['spin_0p7_position_drift_m'] = float(np.max(np.hypot(x[m] - x[m][0], y[m] - y[m][0])))
        metrics['spin_0p7_planar_speed_peak_m_s'] = float(np.max(speed[m]))
        p, m = window('crab_0p3'); metrics['crab_0p3_heading_drift_deg'] = float(np.degrees(np.max(np.abs(np.unwrap(yaw[m]) - yaw[m][0]))))
        metrics['crab_0p3_vx_leak_peak_m_s'] = float(np.max(np.abs(vx[m])))
        for name in ('morph_normal_to_extended', 'morph_extended_to_compact', 'morph_compact_to_extended', 'morph_extended_to_normal',
                     'morph_moving_normal_to_compact', 'morph_moving_compact_to_normal'):
            if name in phases: metrics[name] = morph(name)
        whole = derive(t, vx)
        metrics['whole_run_peak_abs_accel_filtered'] = float(np.max(np.abs(whole[2]))); metrics['whole_run_peak_abs_jerk_filtered'] = float(np.max(np.abs(whole[3])))
        if len(imu) > 10:
            metrics['imu_peak_abs_ax_filtered_0p1s'] = float(np.max(np.abs(smooth(imu[:, 1], 10))))
            metrics['imu_peak_abs_ax_unfiltered'] = float(np.max(np.abs(imu[:, 1])))
        try:
            import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
            for name, title in [('fwd_0p6', 'Forward step 0 -> 0.6 m/s (track mode)'), ('fwd_0p6_stop', 'Stop from 0.6 m/s (command zero)'),
                                ('estop_hold', 'E-stop from 0.6 m/s'), ('morph_moving_normal_to_compact', 'Morph requested while commanding 0.6 m/s')]:
                p, m = window(name); d = derive(t[m], vx[m]); cm = (st_t >= p['sim_start_s']) & (st_t <= p['sim_end_s'])
                if d is None: continue
                fig, ax = plt.subplots(3, 1, figsize=(8, 7), sharex=True)
                ax[0].plot(t[m] - p['sim_start_s'], vx[m], lw=.6, color='0.6', label='body vx (odom, raw)'); ax[0].plot(d[0] - p['sim_start_s'], d[1], color='C0', label=f'body vx ({WINDOW_S:.2f} s mean)')
                ax[0].plot(st_t[cm] - p['sim_start_s'], st_cmd[cm], color='C3', ls='--', label='controller |v| setpoint'); ax[0].set_ylabel('v [m/s]'); ax[0].legend(fontsize=7)
                ax[1].plot(d[0] - p['sim_start_s'], d[2], color='C1'); ax[1].set_ylabel('a [m/s²]')
                ax[2].plot(d[0] - p['sim_start_s'], d[3], color='C2'); ax[2].set_ylabel('jerk [m/s³]'); ax[2].set_xlabel('sim time in phase [s]')
                for a_ in ax: a_.grid(alpha=.3)
                fig.suptitle(f'BASELINE (before V2): {title}\nsimulated, assumed parameters; derivatives band-limited by {WINDOW_S:.2f} s window', fontsize=9)
                fig.tight_layout(); fig.savefig(out / f'baseline_{name}.png', dpi=110); plt.close(fig)
            fig, ax = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
            ax[0].plot(st_t - st_t[0], 1000 * (.54 + 8 * st_stage), color='C0'); ax[0].set_ylabel('wheelbase [mm]'); ax[0].axhline(237, color='0.7', ls=':'); ax[0].axhline(672, color='0.7', ls=':')
            ax[1].plot(t - st_t[0], speed, color='C1', lw=.7); ax[1].axhline(.22, color='C3', ls='--', lw=.8, label='0.22 m/s morph gate'); ax[1].set_ylabel('planar speed [m/s]'); ax[1].set_xlabel('sim time [s]'); ax[1].legend(fontsize=7)
            for a_ in ax: a_.grid(alpha=.3)
            fig.suptitle('BASELINE (before V2): wheelbase and speed over the whole protocol (simulated)', fontsize=9); fig.tight_layout(); fig.savefig(out / 'baseline_shape_and_speed.png', dpi=110); plt.close(fig)
        except Exception as exc:  # plots are optional evidence
            metrics['plot_error'] = repr(exc)
    (out / 'metrics.json').write_text(json.dumps(metrics, indent=2)); print(json.dumps(metrics, indent=2))


if __name__ == '__main__':
    main()
