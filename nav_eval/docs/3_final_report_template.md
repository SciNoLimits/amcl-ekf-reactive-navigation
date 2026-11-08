# Final Performance Evaluation: AMCL vs EKF Localization (TurtleBot3, ROS 2 Jazzy, Gazebo)

1. Introduction & objective
2. System architecture (map->odom->base_link; how EKF provides a global pose) [from Person 1-3]
3. Metrics (copy docs/1_metrics_definitions.md)
4. Test setup: world/map, goals, start pose, repeats, parameters (report/params from results/*/params.json)
5. Results (paste report/report.md tables + graphs)
   - Goal success rate: report/goal_success_rate.png
   - Pose accuracy / variance / path deviation / stability: report/metrics_boxplots.png
   - Trajectories: report/trajectories_<goal>.png
   - Error over time: report/error_over_time.png
6. Discussion: which is better per metric and why; failure cases (see n_ABORTED / n_TIMEOUT in summary.csv)
7. Limitations: simulation only, teleport reset, N trials, ground-truth frame offset
8. Conclusion
