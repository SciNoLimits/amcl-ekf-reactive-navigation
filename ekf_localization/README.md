# EKF Localisation

ROS 2 Jazzy package for TurtleBot3 wheel odometry and IMU fusion.

## Inputs
- `/odom`: differential-drive wheel odometry
- `/imu`: Gazebo IMU, adapted to `/imu/ekf`

## Outputs
- `/odometry/filtered`
- TF `odom -> base_footprint`

AMCL remains responsible for `map -> odom`.

## Build
From workspace root:
```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
