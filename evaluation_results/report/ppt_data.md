# PPT data (auto-generated)

## Slide: Goal success rate

- AMCL: 9/9 = 100.0% (95% CI 70.1-100.0%), Nav2-reported 100.0%, failures: {}
- EKF: 8/9 = 88.9% (95% CI 56.5-98.0%), Nav2-reported 88.9%, failures: {'TIMEOUT': 1}

## Slide: Comparison table (mean +- std over trials)

| Metric | AMCL | EKF |
|---|---|---|
| Pose accuracy: position RMSE [m] | 0.0626 +- 0.0153 | 0.1000 +- 0.0291 |
| Pose accuracy: yaw RMSE [rad] | 0.0223 +- 0.0017 | 0.0390 +- 0.0114 |
| Pose variance: mean cov(x)+cov(y) [m^2] | 0.0447 +- 0.0083 | 0.0001 +- 0.0000 |
| Pose variance: mean cov(yaw) [rad^2] | 0.0148 +- 0.0012 | 0.0001 +- 0.0000 |
| Path deviation: mean cross-track [m] | 0.0741 +- 0.0164 | 0.1057 +- 0.0403 |
| Path deviation: max cross-track [m] | 0.1978 +- 0.0484 | 0.2796 +- 0.1162 |
| Stability: stationary position std [m] | 0.0001 +- 0.0003 | 0.0008 +- 0.0011 |
| Stability: stationary yaw std [rad] | 0.0000 +- 0.0000 | 0.0004 +- 0.0006 |
| Stability: stationary drift [m] | 0.0010 +- 0.0020 | 0.0066 +- 0.0093 |
| Stability: stationary pose variance cov(x)+cov(y) [m^2] | 0.0264 +- 0.0110 | 0.0001 +- 0.0000 |

## Slide: Graphs (insert these images)

- goal_success_rate.png
- metrics_boxplots.png
- error_over_time.png
- trajectories_g1.png
- trajectories_g2.png
- trajectories_g3.png

## Slide: Per-goal success

```
goal  trials  successes     rate_% mode
  g1       3          3 100.000000 amcl
  g2       3          3 100.000000 amcl
  g3       3          3 100.000000 amcl
  g1       3          3 100.000000  ekf
  g2       3          2  66.666667  ekf
  g3       3          3 100.000000  ekf
```
