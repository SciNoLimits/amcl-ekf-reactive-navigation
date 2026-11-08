#!/usr/bin/env python3
"""Generate synthetic result folders so the analysis pipeline can be tested
before teammates finish.   ros2 run nav_eval fake_data --out /tmp/fake"""
import argparse
import csv
import json
import math
import os

import numpy as np

GOALS = [('g1', 1.0, 0.0), ('g2', 2.0, -1.5), ('g3', 0.0, 1.5)]
START = (-2.0, -0.5)


def gen(out, mode, repeats, rng, p_succ, noise, drift):
    os.makedirs(out, exist_ok=True)
    tcols = ['trial_id', 'mode', 'repeat', 'goal', 'goal_x', 'goal_y', 'goal_yaw', 'nav2_status',
             'nav2_success', 'gt_final_dist', 'est_final_dist', 'success', 'duration_s',
             'recoveries', 'start_err', 'missing_samples']
    scols = ['trial_id', 'phase', 't', 'est_x', 'est_y', 'est_yaw', 'gt_x', 'gt_y', 'gt_yaw',
             'cov_xx', 'cov_yy', 'cov_yaw']
    plans = {}
    with open(os.path.join(out, 'trials.csv'), 'w', newline='') as tf, \
            open(os.path.join(out, 'samples.csv'), 'w', newline='') as sf:
        tw, sw = csv.DictWriter(tf, tcols), csv.writer(sf)
        tw.writeheader()
        sw.writerow(scols)
        for r in range(repeats):
            for g, gx, gy in GOALS:
                tid = f'r{r}_{g}'
                ok = rng.random() < p_succ
                n = 150
                s = np.linspace(0, 1 if ok else 0.6, n)
                wob = rng.normal(0, 0.03, (n, 2))
                gt = np.c_[START[0] + (gx - START[0]) * s, START[1] + (gy - START[1]) * s] + wob
                plan = [[START[0] + (gx - START[0]) * u, START[1] + (gy - START[1]) * u]
                        for u in np.linspace(0, 1, 30)]
                plans[tid] = plan
                err = rng.normal(0, noise, (n, 2)) + np.outer(s, [drift, drift])
                est = gt + err
                t = 0.0
                for i in range(n):
                    t += 0.1
                    sw.writerow([tid, 'moving', t, *est[i], 0.0, *gt[i], 0.0,
                                 noise ** 2 * 2, noise ** 2 * 2, 0.02])
                for i in range(100):
                    t += 0.1
                    e = est[-1] + rng.normal(0, noise / 3, 2)
                    sw.writerow([tid, 'stationary', t, *e, 0.0, *gt[-1], 0.0,
                                 noise ** 2 * 2, noise ** 2 * 2, 0.02])
                d = float(np.hypot(gt[-1][0] - gx, gt[-1][1] - gy))
                tw.writerow(dict(trial_id=tid, mode=mode, repeat=r, goal=g, goal_x=gx, goal_y=gy,
                                 goal_yaw=0, nav2_status='SUCCEEDED' if ok else 'ABORTED',
                                 nav2_success=int(ok), gt_final_dist=round(d, 3),
                                 est_final_dist=round(d, 3), success=int(ok and d < 0.3),
                                 duration_s=round(n * 0.1, 1), recoveries=int(not ok),
                                 start_err=0.02, missing_samples=0))
    with open(os.path.join(out, 'plans.json'), 'w') as f:
        json.dump(plans, f)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='/tmp/fake_results')
    a = ap.parse_args()
    rng = np.random.default_rng(1)
    gen(os.path.join(a.out, 'amcl_fake'), 'amcl', 10, rng, 0.9, 0.04, 0.0)
    gen(os.path.join(a.out, 'ekf_fake'), 'ekf', 10, rng, 0.7, 0.05, 0.15)
    print(f'Fake data in {a.out}/amcl_fake and {a.out}/ekf_fake')


if __name__ == '__main__':
    main()
