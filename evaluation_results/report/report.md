# Evaluation summary

mode                    amcl    ekf
trials                 9.000  9.000
verified_successes     9.000  8.000
success_rate_%       100.000 88.889
ci95_low_%            70.085 56.499
ci95_high_%          100.000 98.011
nav2_reported_rate_% 100.000 88.889
mean_time_s(success)  17.261 17.541
mean_recoveries        0.000  0.667
pos_rmse_mean          0.063  0.100
pos_rmse_std           0.016  0.031
yaw_rmse_mean          0.022  0.039
yaw_rmse_std           0.002  0.012
pos_var_mean           0.045  0.000
pos_var_std            0.009  0.000
yaw_var_mean           0.015  0.000
yaw_var_std            0.001  0.000
dev_mean_mean          0.074  0.106
dev_mean_std           0.017  0.043
dev_max_mean           0.198  0.280
dev_max_std            0.051  0.123
stat_pos_std_mean      0.000  0.001
stat_pos_std_std       0.000  0.001
stat_yaw_std_mean      0.000  0.000
stat_yaw_std_std       0.000  0.001
stat_drift_mean        0.001  0.007
stat_drift_std         0.002  0.010
stat_cov_pos_mean      0.026  0.000
stat_cov_pos_std       0.012  0.000
n_SUCCEEDED            9.000  8.000
n_TIMEOUT                NaN  1.000

## Success per goal

goal  trials  successes     rate_% mode
  g1       3          3 100.000000 amcl
  g2       3          3 100.000000 amcl
  g3       3          3 100.000000 amcl
  g1       3          3 100.000000  ekf
  g2       3          2  66.666667  ekf
  g3       3          3 100.000000  ekf
