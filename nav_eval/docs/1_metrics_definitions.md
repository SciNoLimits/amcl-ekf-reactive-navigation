# Metrics definitions (slide: "Evaluation Metrics")

| Metric | What is measured | Formula / how computed | Data source | Better |
|---|---|---|---|---|
| Goal Success Rate | Goals reached out of attempts | successes / trials x 100. Success = Nav2 returns SUCCEEDED AND Gazebo ground-truth distance to goal <= 0.30 m. 95% Wilson confidence interval reported | Nav2 action result + Gazebo ground truth | higher |
| Pose Accuracy | Estimated pose vs ground truth | position error e = sqrt((x_est-x_gt)^2 + (y_est-y_gt)^2); reported as RMSE per trial, then mean/std over trials. Yaw error wrapped to [-pi, pi] | map->base_footprint TF vs Gazebo pose | lower |
| Pose Variance | Uncertainty claimed by the estimator | mean of cov(x)+cov(y) [m^2] and cov(yaw) [rad^2] over the trial | /amcl_pose or /odometry/filtered covariance | lower |
| Path Deviation | Actual vs reference trajectory | cross-track distance from each ground-truth point to the first Nav2 global plan (polyline); mean and max | /plan + Gazebo ground truth | lower |
| Stability | Consistency while stationary | 10 s recording after reaching goal: std of est. position [m], std of yaw [rad], drift (first-to-last position distance) | estimated pose time series | lower |

## Test protocol (slide: "Methodology")
- Same start pose for every trial (robot teleported in Gazebo, initial pose re-published, costmaps cleared).
- Same goal list (config/goals.yaml) for AMCL and EKF.
- N repeats per goal; timeout 120 s (sim time) -> counted as failure.
- Sampling 10 Hz; ground truth from Gazebo dynamic_pose.
- All parameters of a run are stored in results/<run>/params.json.
