#!/usr/bin/env python3
"""Compute metrics + graphs from one or more result folders.

  ros2 run nav_eval analyze --amcl results/amcl_XXXX --ekf results/ekf_XXXX --out report
"""
import argparse
import json
import math
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402


def wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


def wilson(k, n, z=1.96):
    if n == 0:
        return (float('nan'), float('nan'))
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (max(0.0, c - h), min(1.0, c + h))


def cross_track(points, path):
    """Distance from each point to the polyline `path` (Nx2)."""
    path = np.asarray(path, float)
    if len(path) < 2:
        return np.full(len(points), np.nan)
    a, b = path[:-1], path[1:]
    ab = b - a
    denom = np.maximum((ab ** 2).sum(1), 1e-12)
    out = []
    for p in points:
        t = np.clip(((p - a) * ab).sum(1) / denom, 0, 1)
        proj = a + t[:, None] * ab
        out.append(np.sqrt(((p - proj) ** 2).sum(1)).min())
    return np.array(out)


def trial_metrics(trials, samples, plans):
    rows = []
    for _, tr in trials.iterrows():
        s = samples[samples.trial_id == tr.trial_id]
        mv, st = s[s.phase == 'moving'], s[s.phase == 'stationary']
        r = {'trial_id': tr.trial_id, 'goal': tr.goal}
        if len(mv):
            e = np.hypot(mv.est_x - mv.gt_x, mv.est_y - mv.gt_y)
            ye = np.abs(wrap(mv.est_yaw - mv.gt_yaw))
            r.update(pos_rmse=float(np.sqrt((e ** 2).mean())), pos_mean=float(e.mean()),
                     pos_max=float(e.max()), yaw_rmse=float(np.sqrt((ye ** 2).mean())),
                     pos_var=float((mv.cov_xx + mv.cov_yy).mean()),
                     yaw_var=float(mv.cov_yaw.mean()))
            plan = plans.get(tr.trial_id)
            if plan:
                d = cross_track(mv[['gt_x', 'gt_y']].to_numpy(), plan)
                r.update(dev_mean=float(np.nanmean(d)), dev_max=float(np.nanmax(d)),
                         dev_rms=float(np.sqrt(np.nanmean(d ** 2))))
        if len(st) > 3:
            yaw = np.unwrap(st.est_yaw.to_numpy())
            ee = np.hypot(st.est_x - st.gt_x, st.est_y - st.gt_y)
            r.update(stat_pos_std=float(np.sqrt(st.est_x.var() + st.est_y.var())),
                     stat_yaw_std=float(np.std(yaw)),
                     stat_drift=float(math.hypot(st.est_x.iloc[-1] - st.est_x.iloc[0],
                                                 st.est_y.iloc[-1] - st.est_y.iloc[0])),
                     stat_err_mean=float(ee.mean()),
                     stat_cov_pos=float((st.cov_xx + st.cov_yy).mean()))
        rows.append(r)
    return pd.DataFrame(rows)


def load(path):
    trials = pd.read_csv(os.path.join(path, 'trials.csv'))
    samples = pd.read_csv(os.path.join(path, 'samples.csv'))
    with open(os.path.join(path, 'plans.json')) as f:
        plans = json.load(f)
    tm = trial_metrics(trials, samples, plans)
    return dict(trials=trials, samples=samples, plans=plans, metrics=tm)


METRICS = [
    ('pos_rmse', 'Pose accuracy: position RMSE [m]'),
    ('yaw_rmse', 'Pose accuracy: yaw RMSE [rad]'),
    ('pos_var', 'Pose variance: mean cov(x)+cov(y) [m^2]'),
    ('yaw_var', 'Pose variance: mean cov(yaw) [rad^2]'),
    ('dev_mean', 'Path deviation: mean cross-track [m]'),
    ('dev_max', 'Path deviation: max cross-track [m]'),
    ('stat_pos_std', 'Stability: stationary position std [m]'),
    ('stat_yaw_std', 'Stability: stationary yaw std [rad]'),
    ('stat_drift', 'Stability: stationary drift [m]'),
    ('stat_cov_pos', 'Stability: stationary pose variance cov(x)+cov(y) [m^2]'),
]
COLORS = {'amcl': '#1f77b4', 'ekf': '#d62728'}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--amcl', help='result folder of the AMCL run')
    ap.add_argument('--ekf', help='result folder of the EKF run')
    ap.add_argument('--out', default='report')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    data = {k: load(v) for k, v in (('amcl', a.amcl), ('ekf', a.ekf)) if v}
    if not data:
        ap.error('give at least --amcl or --ekf')

    # ---------- summary table
    summ, per_goal = [], []
    for m, d in data.items():
        t = d['trials']
        n, k, k2 = len(t), int(t.success.sum()), int(t.nav2_success.sum())
        lo, hi = wilson(k, n)
        row = {'mode': m, 'trials': n, 'verified_successes': k,
               'success_rate_%': 100 * k / n, 'ci95_low_%': 100 * lo, 'ci95_high_%': 100 * hi,
               'nav2_reported_rate_%': 100 * k2 / n,
               'mean_time_s(success)': t[t.success == 1].duration_s.mean(),
               'mean_recoveries': t.recoveries.mean()}
        for key, _ in METRICS:
            if key in d['metrics']:
                row[key + '_mean'] = d['metrics'][key].mean()
                row[key + '_std'] = d['metrics'][key].std()
        for s_, c in t.nav2_status.value_counts().items():
            row['n_' + s_] = int(c)
        summ.append(row)
        g = t.groupby('goal').agg(trials=('success', 'size'), successes=('success', 'sum'))
        g['rate_%'] = 100 * g.successes / g.trials
        g['mode'] = m
        per_goal.append(g.reset_index())
    S = pd.DataFrame(summ).set_index('mode').T
    G = pd.concat(per_goal)
    S.to_csv(os.path.join(a.out, 'summary.csv'))
    G.to_csv(os.path.join(a.out, 'success_per_goal.csv'), index=False)
    pd.concat([d['metrics'].assign(mode=m) for m, d in data.items()]).to_csv(
        os.path.join(a.out, 'per_trial_metrics.csv'), index=False)
    with open(os.path.join(a.out, 'report.md'), 'w') as f:
        f.write('# Evaluation summary\n\n' + S.to_string(float_format=lambda x: f'{x:.3f}')
                + '\n\n## Success per goal\n\n' + G.to_string(index=False) + '\n')
    print(S.to_string(float_format=lambda x: f'{x:.3f}'))

    # ---------- graph 1: goal success rate
    fig, ax = plt.subplots(1, 2, figsize=(11, 4), gridspec_kw={'width_ratios': [1, 2]})
    for i, (m, d) in enumerate(data.items()):
        t = d['trials']
        n, k = len(t), int(t.success.sum())
        lo, hi = wilson(k, n)
        r = 100 * k / n
        ax[0].bar(i, r, color=COLORS[m], yerr=[[r - 100 * lo], [100 * hi - r]], capsize=6)
        ax[0].text(i, r + 3, f'{k}/{n}', ha='center')
    ax[0].set_xticks(range(len(data)))
    ax[0].set_xticklabels([m.upper() for m in data])
    ax[0].set_ylim(0, 115)
    ax[0].set_ylabel('Goal success rate [%]')
    ax[0].set_title('Overall (95% Wilson CI)')
    goals = sorted(G.goal.unique())
    w = 0.8 / len(data)
    for i, (m, _) in enumerate(data.items()):
        gm = G[G['mode'] == m].set_index('goal').reindex(goals)
        ax[1].bar(np.arange(len(goals)) + i * w, gm['rate_%'], w, label=m.upper(), color=COLORS[m])
    ax[1].set_xticks(np.arange(len(goals)) + w * (len(data) - 1) / 2)
    ax[1].set_xticklabels(goals)
    ax[1].set_ylim(0, 115)
    ax[1].legend()
    ax[1].set_title('Per goal')
    fig.tight_layout()
    fig.savefig(os.path.join(a.out, 'goal_success_rate.png'), dpi=150)
    plt.close(fig)

    # ---------- graph 2: box plots of per-trial metrics
    avail = [(k, t) for k, t in METRICS if any(k in d['metrics'] for d in data.values())]
    cols = 3
    rows = math.ceil(len(avail) / cols)
    fig, axs = plt.subplots(rows, cols, figsize=(4.5 * cols, 3.6 * rows))
    axs = np.atleast_1d(axs).ravel()
    for ax_, (k, title) in zip(axs, avail):
        vals = [d['metrics'][k].dropna().to_numpy() if k in d['metrics'] else np.array([])
                for d in data.values()]
        bp = ax_.boxplot(vals, labels=[m.upper() for m in data], patch_artist=True)
        for patch, m in zip(bp['boxes'], data):
            patch.set_facecolor(COLORS[m])
            patch.set_alpha(0.6)
        ax_.set_title(title, fontsize=9)
    for ax_ in axs[len(avail):]:
        ax_.axis('off')
    fig.tight_layout()
    fig.savefig(os.path.join(a.out, 'metrics_boxplots.png'), dpi=150)
    plt.close(fig)

    # ---------- graph 3: trajectories per goal
    for g in goals:
        fig, ax = plt.subplots(figsize=(5.5, 5))
        for m, d in data.items():
            ids = d['trials'][d['trials'].goal == g].trial_id
            for j, tid in enumerate(ids):
                s = d['samples']
                s = s[(s.trial_id == tid) & (s.phase == 'moving')]
                ax.plot(s.gt_x, s.gt_y, color=COLORS[m], alpha=0.5, lw=1,
                        label=m.upper() if j == 0 else None)
            ref = next((d['plans'].get(t) for t in ids if d['plans'].get(t)), None)
            if ref is not None and m == list(data)[0]:
                r = np.array(ref)
                ax.plot(r[:, 0], r[:, 1], 'k--', lw=1.5, label='planned path')
            row = d['trials'][d['trials'].goal == g].iloc[0]
            ax.plot(row.goal_x, row.goal_y, 'g*', ms=14)
        ax.set_aspect('equal')
        ax.set_title(f'Ground-truth trajectories, goal {g}')
        ax.legend()
        ax.grid(alpha=0.3)
        fig.savefig(os.path.join(a.out, f'trajectories_{g}.png'), dpi=150)
        plt.close(fig)

    # ---------- graph 4: error over time (first trial of first goal)
    fig, ax = plt.subplots(figsize=(7, 3.5))
    for m, d in data.items():
        tid = d['trials'].trial_id.iloc[0]
        s = d['samples'][d['samples'].trial_id == tid]
        e = np.hypot(s.est_x - s.gt_x, s.est_y - s.gt_y)
        ax.plot(s.t - s.t.iloc[0], e, color=COLORS[m], label=f'{m.upper()} ({tid})')
    ax.set_xlabel('time [s]')
    ax.set_ylabel('position error [m]')
    ax.legend()
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(a.out, 'error_over_time.png'), dpi=150)

    # ---------- slide-ready data
    L = ['# PPT data (auto-generated)\n']
    L.append('## Slide: Goal success rate\n')
    for m, d in data.items():
        t = d['trials']; k = int(t.success.sum()); n = len(t); lo, hi = wilson(k, n)
        L.append(f'- {m.upper()}: {k}/{n} = {100*k/n:.1f}% (95% CI {100*lo:.1f}-{100*hi:.1f}%), '
                 f'Nav2-reported {100*t.nav2_success.mean():.1f}%, failures: '
                 f'{t[t.success == 0].nav2_status.value_counts().to_dict()}')
    L.append('\n## Slide: Comparison table (mean +- std over trials)\n')
    L.append('| Metric | ' + ' | '.join(m.upper() for m in data) + ' |')
    L.append('|---|' + '---|' * len(data))
    for key, title in METRICS:
        cells = []
        for d in data.values():
            v = d['metrics'][key].dropna() if key in d['metrics'] else []
            cells.append(f'{np.mean(v):.4f} +- {np.std(v):.4f}' if len(v) else 'n/a')
        L.append(f'| {title} | ' + ' | '.join(cells) + ' |')
    L.append('\n## Slide: Graphs (insert these images)\n')
    L += ['- goal_success_rate.png', '- metrics_boxplots.png', '- error_over_time.png']
    L += [f'- trajectories_{g}.png' for g in goals]
    L.append('\n## Slide: Per-goal success\n\n```\n' + G.to_string(index=False) + '\n```')
    with open(os.path.join(a.out, 'ppt_data.md'), 'w') as f:
        f.write('\n'.join(L) + '\n')
    print(f'\nReport written to {a.out}/')


if __name__ == '__main__':
    main()
