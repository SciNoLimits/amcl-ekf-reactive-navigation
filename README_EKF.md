# EKF Localization Evaluation Using ROS 2 and Gazebo

## 1. Project Overview

This project evaluates the performance of an Extended Kalman Filter (EKF) for robot localization using ROS 2 and Gazebo.

The objective is to compare the robot's estimated position and orientation, produced by the `robot_localization` EKF, against Gazebo's ground-truth pose.

The robot is controlled autonomously using Nav2's `NavigateToPose` action.

The experiment records both trajectories, calculates estimation errors, and generates plots and performance metrics.

### Evaluation objectives

- Evaluate positional estimation accuracy.
- Evaluate orientation (yaw) estimation accuracy.
- Measure stationary drift.
- Observe how estimation uncertainty changes during navigation.
- Compare the estimated trajectory against the ground-truth trajectory.

## 2. System Architecture

The experiment involves three main components.

### A. EKF Localization

The `robot_localization` package runs an Extended Kalman Filter.

It receives sensor measurements, including:
- Wheel odometry (`/odom`).
- IMU measurements (`/imu`).

It fuses these measurements and publishes the filtered odometry:

`/odometry/filtered`

Message type: `nav_msgs/msg/Odometry`

The output contains:
- Estimated position.
- Estimated orientation.
- Linear and angular velocity.
- Pose covariance.
- Twist covariance.

The EKF publishes its estimate in the `odom` frame.

### B. Gazebo Ground Truth

Gazebo provides the simulated robot's actual pose through:

`/world/default/dynamic_pose/info`

Message type: `gz.msgs.Pose_V`

A custom C++ ROS 2 node subscribes to this Gazebo Transport topic using `gz-transport` and extracts the pose belonging to the entity named `robot`.

It publishes the extracted pose to:

`/ground_truth/pose`

Message type: `geometry_msgs/msg/PoseStamped`

The ground-truth pose is expressed in the `world` frame.

This provides an independent reference for evaluating the EKF estimate.

### C. Experiment Recorder

A Python ROS 2 node performs the experiment.

It:
1. Subscribes to `/odometry/filtered`.
2. Subscribes to `/ground_truth/pose`.
3. Records both streams with timestamps.
4. Sends an autonomous navigation goal to Nav2.
5. Records the robot's movement.
6. Saves the measurements into CSV files.

The navigation goal is sent through:

`/navigate_to_pose`

Message type: `nav2_msgs/action/NavigateToPose`.

## 3. Experiment Workflow

The experiment consists of three phases.

| Phase | Duration | Description |
|---|---:|---|
| Stationary baseline | 30 seconds | Records the robot while stationary |
| Autonomous navigation | Until goal completion | Records the robot while navigating |
| Final stationary | 15 seconds | Records the robot after navigation |

Both the EKF and ground-truth measurements are recorded throughout the experiment.

The default navigation timeout is 240 seconds.

## 4. Repository Structure

The relevant packages and files are:

```text
workspace/
├── src/
│   └── amcl-ekf-reactive-navigation/
│       ├── ekf_ground_truth/
│       │   ├── src/
│       │   │   └── gazebo_ground_truth_node.cpp
│       │   ├── CMakeLists.txt
│       │   └── package.xml
│       │
│       └── ekf_localization/
│           ├── ekf_localization/
│           │   ├── run_ekf_experiment.py
│           │   └── analyze_ekf.py
│           ├── launch/
│           │   └── experiment.launch.py
│           ├── package.xml
│           └── setup.py
│
└── ekf_experiment/
    ├── ekf.csv
    ├── ground_truth.csv
    ├── aligned_errors.csv
    ├── metrics.csv
    ├── trajectory_comparison.png
    ├── position_error.png
    ├── yaw_error.png
    └── covariance.png
```

The experiment output directory is generated when the experiment and analysis scripts are executed.

## 5. Requirements

The current implementation uses:

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic (Gazebo Sim 8)
- `robot_localization`
- Nav2
- `ros_gz` / Gazebo Transport
- Python 3
- NumPy
- Matplotlib

The workspace must contain the robot simulation, navigation, EKF and experiment packages.

The custom ground-truth package also requires the compatible Gazebo Transport and Gazebo Messages development libraries.

## 6. Building the Workspace

Clone the repository and enter the workspace:

```bash
cd /workspace
```

Source ROS 2:

```bash
source /opt/ros/jazzy/setup.bash
```

Install any missing dependencies:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

Build the workspace:

```bash
colcon build --symlink-install
```

Source the workspace:

```bash
source install/setup.bash
```

## 7. Running the Experiment

### Step 1: Start the robot simulation

Launch the normal robot bringup, which starts Gazebo, the robot controllers, EKF and Nav2:

```bash
ros2 launch robot_bringup bringup.launch.py
```

Wait until the simulation and navigation stack are ready.

### Step 2: Verify the required topics

Check that the EKF is publishing:

```bash
ros2 topic echo /odometry/filtered --once
```

Check that Gazebo is publishing its ground-truth source topic:

```bash
gz topic -l | grep dynamic_pose
```

The experiment launch file starts the custom ground-truth publisher, so it is not necessary to start that node manually.

### Step 3: Launch the experiment

Open another terminal and source the workspace:

```bash
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
```

Run:

```bash
ros2 launch ekf_localization experiment.launch.py \
    goal_x:=1.0 goal_y:=1.0
```

The goal coordinates are expressed in the `map` frame.

Choose a goal that is reachable within the navigation map.

### Step 4: Check the recorded data

After the experiment finishes:

```bash
ls -lh ekf_experiment/
```

The recorder generates:

- `ekf.csv`
- `ground_truth.csv`

These contain the timestamped measurements from the EKF and Gazebo.

## 8. Analysing the Results

The analysis is performed using `analyze_ekf.py`.

Run:

```bash
python3 src/amcl-ekf-reactive-navigation/ekf_localization/ekf_localization/analyze_ekf.py \
    --input-dir /workspace/ekf_experiment
```

The script performs the following operations:

1. Loads the EKF and ground-truth CSV files.
2. Matches the trajectories using their timestamps.
3. Interpolates ground-truth measurements where necessary.
4. Aligns the initial poses of both trajectories.
5. Calculates position and orientation errors.
6. Calculates performance metrics.
7. Generates plots.

### Generated metrics

| Metric | Description |
|---|---|
| Position RMSE | Root mean square planar position error |
| Mean position error | Average planar position error |
| Maximum position error | Largest observed planar position error |
| Yaw RMSE | Root mean square heading error |
| Mean absolute yaw error | Average absolute heading error |
| Maximum absolute yaw error | Largest absolute heading error |
| Mean X/Y/Yaw variance | Average EKF-reported pose variance |
| Baseline drift | Position displacement during the initial stationary phase |
| Final stationary drift | Position displacement during the final stationary phase |

### Generated plots

**Trajectory comparison**

Compares the aligned EKF trajectory with the Gazebo ground-truth trajectory.

**Position error**

Shows how the planar position estimation error changes over time.

**Yaw error**

Shows the difference between the estimated and actual heading.

**Covariance**

Shows how the EKF's estimated uncertainty evolves during the experiment.

## 9. Initial Experimental Results

The first experiment completed successfully.

The recorded dataset contained:

- 2,024 overlapping EKF and ground-truth samples.
- Approximately 67.45 seconds of overlapping measurements.

The following metrics were obtained:

| Metric | Measured value |
|---|---:|
| Position RMSE | 0.00527 m |
| Mean position error | 0.00314 m |
| Maximum position error | 0.01668 m |
| Yaw RMSE | 0.000400 rad |
| Mean absolute yaw error | 0.000225 rad |
| Maximum absolute yaw error | 0.00518 rad |
| Baseline drift | Approximately 0 m |
| Final stationary drift | 0.00577 m |

These results indicate close agreement between the EKF trajectory and Gazebo ground truth during the recorded experiment.

However, these measurements should be treated as an initial baseline rather than a complete characterization of EKF performance.

## 10. Important Experimental Considerations

### Coordinate frame alignment

The EKF publishes its pose in the `odom` frame, while Gazebo publishes ground truth in the `world` frame.

These frames can have different origins.

The analysis therefore aligns the initial poses independently before comparing relative trajectories.

Consequently, the reported RMSE measures relative motion-estimation accuracy, not absolute global localization accuracy.

### Timestamp synchronization

The two data streams are published at different frequencies.

The analysis uses timestamps and interpolation to compare measurements at corresponding times.

Correct simulation-time synchronization is essential for meaningful results.

### Covariance interpretation

EKF covariance represents the filter's estimated uncertainty.

It is not the same as the actual estimation error.

A low covariance does not automatically imply accurate localization.

### Sensor independence

The ground-truth reference must remain independent of the measurements supplied to the EKF.

Gazebo ground truth is used only for evaluation, not as an EKF input.

## 11. Future Work

The initial experiment establishes a working evaluation pipeline.

Further experiments should investigate:

1. EKF performance during acceleration and deceleration.
2. Accuracy during sharp turns and rotational movements.
3. Long-duration stationary drift.
4. Performance under simulated sensor noise.
5. Performance with wheel odometry degradation.
6. Covariance consistency and uncertainty calibration.
7. Repeatability across multiple experimental runs.

The results should be compared across different configurations to understand the contribution of individual sensor measurements.

## 12. Summary

This project implements an automated EKF localization evaluation pipeline using ROS 2 Jazzy, Gazebo and Nav2.

It combines:
- Sensor fusion through `robot_localization`.
- Gazebo ground-truth extraction through Gazebo Transport.
- Autonomous robot navigation through Nav2.
- Automated trajectory recording and analysis using Python.

The resulting system provides a repeatable way to measure and visualize the difference between estimated robot motion and simulated ground truth.
```

## What remains before calling the project complete

I recommend only three additional tasks:

| Priority | Task | Purpose |
|---|---|---|
| 1 | Add the README documentation | Make the implementation reproducible |
| 2 | Commit the initial metrics and plots, if appropriate | Preserve the baseline experiment |
| 3 | Run two or three more trials with different navigation goals | Check repeatability |

You do not need to redesign the implementation now.

One important qualification: the current results are excellent, but they only establish performance for this particular simulation and trajectory. Don't claim that the EKF has been comprehensively validated.

**For another developer:** the essential workflow is `clone → install dependencies → build → launch robot bringup → launch experiment → run analysis`. The README above documents each stage.

If you share the GitHub repository URL, I can also review the actual repository structure and make sure the documentation matches the committed implementation exactly.