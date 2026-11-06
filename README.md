# AMCL EKF Reactive Navigation
ROS 2-based TurtleBot3 autonomous navigation using Nav2, featuring AMCL vs EKF localization comparison, LiDAR-based obstacle detection, reactive obstacle avoidance, and dynamic path replanning. The project combines:

- a reusable robot description in URDF/Xacro;
- Gazebo Sim (formerly Ignition Gazebo) simulation;
- `gz_ros2_control` wheel control;
- LiDAR, IMU, camera, clock, and odometry interfaces;
- Nav2 localization, planning, control, costmaps, recovery behaviors, and
  collision monitoring;
- AMCL localization against a supplied occupancy-grid map; and
- an RViz configuration for observing and commanding the robot.

The repository name refers to the intended AMCL/EKF and reactive-navigation
experiments. The current checked-in implementation launches AMCL and Nav2, but
does not yet include a `robot_localization` EKF configuration or a custom
reactive-avoidance node. Dynamic obstacle handling currently comes from the
Nav2 costmaps, MPPI controller, collision monitor, and recovery behaviors.

## Contents

- [Requirements](#requirements)
- [Workspace setup](#workspace-setup)
- [Run the complete simulation](#run-the-complete-simulation)
- [Run individual components](#run-individual-components)
- [Launch arguments](#launch-arguments)
- [How the system fits together](#how-the-system-fits-together)
- [Packages and important files](#packages-and-important-files)
- [Topics, frames, and interfaces](#topics-frames-and-interfaces)
- [Navigation configuration](#navigation-configuration)
- [Changing the world or map](#changing-the-world-or-map)
- [Useful commands](#useful-commands)
- [Troubleshooting](#troubleshooting)
- [Development notes](#development-notes)
- [License](#license)

## Requirements

The project targets a ROS 2 distribution with the following capabilities:

- ROS 2 launch, `ament_cmake`, `xacro`, `robot_state_publisher`, and RViz 2;
- Nav2 (`nav2_bringup`, AMCL, planner, controller, costmap, and behavior
  packages);
- Gazebo Sim and `ros_gz_sim`;
- `ros_gz_bridge`;
- `gz_ros2_control`, `controller_manager`,
  `joint_state_broadcaster`, and `diff_drive_controller`; and
- a Linux installation with a sourced ROS 2 environment.

The launch files use the newer `ros_gz_*` and `gz_*` interfaces, so install a
ROS 2/Gazebo combination that provides those packages. Jazzy with Gazebo
Harmonic is a suitable baseline. If package names differ in another ROS 2
distribution, install the distribution's equivalent packages.

## Workspace setup

Clone or place this repository in a ROS 2 workspace under `src/`. The commands
below assume the workspace layout used by this repository:

```text
rms_ws/
└── src/
    └── amcl-ekf-reactive-navigation/
```

From the workspace root:

```bash
cd <YOUR WORKSPACE>
source /opt/ros/<ros2-distro>/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Replace `<ros2-distro>` with the installed distribution, such as `jazzy`.
Source `install/setup.bash` again in every new terminal that uses the
workspace.

To confirm that the packages were installed into the overlay:

```bash
ros2 pkg list | grep -E 'robot_(bringup|description|gazebo|navigation)'
```

## Run the complete simulation

The recommended entry point starts Gazebo, spawns the robot, starts the
controllers, starts Nav2, and opens RViz with the project's navigation display:

```bash
cd <YOUR WORKSPACE>
source /opt/ros/<ros2-distro>/setup.bash
source install/setup.bash
ros2 launch robot_bringup bringup.launch.py
```

Startup is intentionally staggered:

1. Gazebo starts with `robot_gazebo/worlds/room.sdf`.
2. The robot description is published and the robot is spawned at
   `(-6.0, 2.0, 0.15)`.
3. The joint-state broadcaster and differential-drive controller are spawned.
4. Nav2 starts with the room map and `nav2_params.yaml`.
5. RViz opens with `robot_navigation/rviz/nav2_rviz_config.rviz`.

The exact startup timing is controlled by timer actions in
[`robot_bringup/launch/bringup.launch.py`](./robot_bringup/launch/bringup.launch.py).
If a machine is slow to start Gazebo, wait for the simulator and controllers
to become ready before sending a navigation goal.

### Sending a navigation goal

In RViz:

1. Set the robot's initial pose with **2D Pose Estimate**. The configured
   default AMCL pose is `(0, 0, 0)`, but the pose should match the robot's
   actual location in the map.
2. Use **Nav2 Goal** (or **2D Goal Pose**, depending on the RViz toolbar) to
   select a target pose.
3. Monitor the global path, local path, costmaps, and robot status in RViz.

From another sourced terminal, a goal can also be sent through Nav2's
`navigate_to_pose` action:

```bash
ros2 action send_goal /navigate_to_pose \
  nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {
    position: {x: 1.0, y: 1.0, z: 0.0},
    orientation: {w: 1.0}
  }}}"
```

Adjust the target coordinates to a free location in `room_map.pgm`.

## Run individual components

### Robot model in RViz

This launch file publishes the Xacro-generated robot description, starts the
joint-state publisher GUI, and opens RViz:

```bash
ros2 launch robot_description display.launch.py
```

This is a visualization-only workflow; it does not start Gazebo, Nav2, or the
simulation sensors.

### Gazebo simulation only

```bash
ros2 launch robot_gazebo gazebo.launch.py
```

The default world is `robot_gazebo/worlds/room.sdf`. This launch includes the
robot state publisher, Gazebo, the ROS-Gazebo bridge, and delayed controller
spawners.

### Nav2 only

```bash
ros2 launch robot_navigation navigation.launch.py
```

Run this after Gazebo and the robot controllers are available. The launch file
loads `robot_gazebo/maps/room_map.yaml`, loads
`robot_navigation/config/nav2_params.yaml`, enables simulated time, and
autostarts Nav2.

### RViz with a custom configuration

The full bringup launch uses the checked-in
[`nav2_rviz_config.rviz`](./robot_navigation/rviz/nav2_rviz_config.rviz).
To inspect or change the display layout, open that file from RViz or start
RViz separately:

```bash
rviz2 -d "$(ros2 pkg prefix robot_navigation)/share/robot_navigation/rviz/nav2_rviz_config.rviz"
```

## Launch arguments

`robot_gazebo gazebo.launch.py` declares these arguments:

| Argument | Default | Description |
| --- | --- | --- |
| `world` | `robot_gazebo/worlds/room.sdf` | SDF world loaded by Gazebo Sim |
| `use_sim_time` | `true` | Uses the `/clock` topic for ROS time |

For example, to run the empty world:

```bash
ros2 launch robot_gazebo gazebo.launch.py \
  world:="$(ros2 pkg prefix robot_gazebo)/share/robot_gazebo/worlds/empty.sdf"
```

The navigation launch currently selects the room map and Nav2 parameter file
internally. To use a different map or parameter file, update
[`robot_navigation/launch/navigation.launch.py`](./robot_navigation/launch/navigation.launch.py)
or provide a separate launch wrapper.

## How the system fits together

```mermaid
flowchart LR
    X[robot_description<br/>URDF/Xacro] --> RSP[robot_state_publisher]
    X --> S[ros_gz_sim create]
    S --> G[Gazebo Sim]
    G --> B[ros_gz_bridge]
    G --> C[gz_ros2_control]
    C --> D[diff_drive_controller]
    B --> L[/scan, /imu, /camera/image, /clock]
    D --> O[/odom and TF odom -> base_footprint]
    M[room_map.yaml + room_map.pgm] --> N[Nav2 bringup]
    P[nav2_params.yaml] --> N
    L --> N
    O --> N
    N --> V[RViz 2]
    N --> D
```

The robot's Xacro is included by
[`robot_gazebo.urdf.xacro`](./robot_gazebo/urdf/robot_gazebo.urdf.xacro),
which adds the Gazebo sensors and `gz_ros2_control` plugin to the base model
from [`robot_description/urdf`](./robot_description/urdf).

## Packages and important files

| Package | Responsibility | Key files |
| --- | --- | --- |
| `robot_description` | TurtleBot3 Waffle Pi geometry, materials, links, joints, and visualization launch | `urdf/turtlebot3_waffle_pi.urdf.xacro`, `meshes/`, `launch/display.launch.py` |
| `robot_gazebo` | Gazebo world, simulated sensors, ROS bridge, controllers, map, and robot simulation Xacro | `launch/gazebo.launch.py`, `urdf/robot_gazebo.urdf.xacro`, `config/`, `worlds/`, `maps/` |
| `robot_navigation` | Nav2 launch, parameters, and RViz navigation layout | `launch/navigation.launch.py`, `config/nav2_params.yaml`, `rviz/` |
| `robot_bringup` | Top-level orchestration of Gazebo, Nav2, and RViz | `launch/bringup.launch.py` |

The main assets are:

- [`room.sdf`](./robot_gazebo/worlds/room.sdf):
  default simulation world.
- [`opil_factory.world`](./robot_gazebo/worlds/opil_factory.world)
  and the `models/factory/` directory: warehouse-style simulation assets.
- [`room_map.yaml`](./robot_gazebo/maps/room_map.yaml)
  and `room_map.pgm`: the static map used by Nav2.
- [`controllers.yaml`](./robot_gazebo/config/controllers.yaml):
  wheel geometry and controller-manager configuration.
- [`bridge.yaml`](./robot_gazebo/config/bridge.yaml):
  Gazebo-to-ROS topic bridge.

## Topics, frames, and interfaces

### Main topics

| Topic | Type | Producer / consumer | Purpose |
| --- | --- | --- | --- |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo -> bridge | Simulation time |
| `/scan` | `sensor_msgs/msg/LaserScan` | Gazebo -> bridge -> Nav2 | 2D LiDAR observations |
| `/imu` | `sensor_msgs/msg/Imu` | Gazebo -> bridge | IMU data |
| `/camera/image` | `sensor_msgs/msg/Image` | Gazebo -> bridge | Simulated camera image |
| `/odom` | `nav_msgs/msg/Odometry` | Differential-drive controller | Wheel odometry |
| `/cmd_vel` | `geometry_msgs/msg/Twist` or stamped equivalent | Nav2 -> controller | Robot velocity command |
| `/map` | `nav_msgs/msg/OccupancyGrid` | Nav2 map server | Static environment map |

The controller launch remaps the differential-drive controller's odometry and
velocity interfaces to `/odom` and `/cmd_vel`. The exact command message
variant is governed by the installed Nav2 and controller versions and the
`enable_stamped_cmd_vel` settings in the Nav2 configuration.

### TF frames

The primary navigation chain is:

```text
map -> odom -> base_footprint -> base_link
                              ├─ base_scan
                              ├─ imu_link
                              └─ camera_link -> camera_rgb_optical_frame
```

AMCL publishes the `map -> odom` correction. The differential-drive controller
publishes odometry and the `odom -> base_footprint` transform. The robot state
publisher publishes the fixed and wheel-related model transforms.

### Sensor settings

The simulated robot currently configures:

- GPU LiDAR on `/scan`, 360 samples over `[-pi, pi]`, 10 Hz, and a range of
  `0.12` to `3.5` m;
- IMU on `/imu` at 100 Hz; and
- RGB camera images on `/camera/image` at 15 Hz, 640 x 480, with an 8 m far
  clip distance.

These settings are in
[`robot_gazebo/urdf/robot_gazebo.urdf.xacro`](./robot_gazebo/urdf/robot_gazebo.urdf.xacro).

## Navigation configuration

[`nav2_params.yaml`](./robot_navigation/config/nav2_params.yaml)
configures the following Nav2 components:

- **AMCL:** differential-drive motion model, `map` global frame,
  `base_footprint` robot frame, `/scan` input, 500-2000 particles, and an
  initial pose at the map origin.
- **Planner:** NavFn grid planner with unknown space allowed.
- **Controller:** MPPI differential-drive controller with a maximum forward
  velocity of 0.5 m/s and path/cost/goal critics enabled.
- **Local costmap:** 3 m x 3 m rolling voxel costmap using `/scan`, with a
  0.70 m inflation radius.
- **Global costmap:** static map plus LiDAR obstacle and inflation layers.
- **Behaviors:** spin, backup, drive-on-heading, assisted teleoperation, and
  wait.
- **Collision monitor:** footprint-approach monitoring based on `/scan`.
- **Velocity smoother:** acceleration and velocity limits for the simulated
  differential-drive base.

The map metadata declares a 0.05 m resolution and an origin of
`[-4.133, -3.561, 0]`. Keep the map, world geometry, robot spawn position,
initial AMCL pose, and frame conventions consistent when replacing assets.

### AMCL versus EKF

AMCL is the localization component currently launched by this repository. The
IMU topic is bridged for inspection and future sensor-fusion work, but there is
no `ekf_node`, `robot_localization` dependency, or EKF parameter file in the
current packages. Adding an EKF should include an explicit frame and
measurement design, then update Nav2's odometry/localization inputs rather than
silently enabling a second estimator.

## Changing the world or map

### Use another Gazebo world

Pass an absolute or package-resolved SDF path through `world`:

```bash
ros2 launch robot_gazebo gazebo.launch.py \
  world:="/absolute/path/to/custom_world.sdf"
```

If the world references local meshes or materials, ensure they are reachable
through `GZ_SIM_RESOURCE_PATH`. The Gazebo launch adds the package's
`robot_gazebo/models` directory and the workspace package root to that
environment variable.

### Use another Nav2 map

1. Add the map image and YAML metadata under
   `robot_gazebo/maps/` (or another installed package).
2. Update the `map_file` path in
   [`navigation.launch.py`](./robot_navigation/launch/navigation.launch.py).
3. Verify `resolution`, `origin`, `occupied_thresh`, and `free_thresh`.
4. Rebuild if the asset location or package installation rules changed.
5. Set the initial AMCL pose in RViz after launching.

## Useful commands

Inspect the runtime graph and key interfaces:

```bash
ros2 node list
ros2 topic list
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 topic hz /scan
ros2 run tf2_tools view_frames
```

Check that Nav2 actions and services are available:

```bash
ros2 action list | grep navigate
ros2 service list | grep -E 'amcl|costmap|map'
```

Inspect package installation paths:

```bash
ros2 pkg prefix robot_gazebo
ros2 pkg prefix robot_navigation
```

Validate Xacro expansion without launching the simulator:

```bash
xacro robot_description/urdf/turtlebot3_waffle_pi.urdf.xacro > /tmp/turtlebot3_waffle_pi.urdf
xacro robot_gazebo/urdf/robot_gazebo.urdf.xacro > /tmp/robot_gazebo.urdf
```

Run the package lint tests supplied by the package templates:

```bash
colcon test --packages-select \
  robot_bringup robot_description robot_gazebo robot_navigation
colcon test-result --verbose
```

## Troubleshooting

### `Package ... not found`

Source both the system ROS setup and the workspace overlay in the same
terminal:

```bash
source /opt/ros/<ros2-distro>/setup.bash
source <YOUR WORKSPACE>/install/setup.bash
```

If the package is still unavailable, rebuild with `colcon build --symlink-install`
and inspect the build output.

### Gazebo starts but the robot is not visible

Check the `robot_state_publisher` and `ros_gz_sim create` output. Confirm that
the Xacro expands successfully, that `GZ_SIM_RESOURCE_PATH` contains the
package assets, and that the spawn location is inside the selected world.

### Controllers do not become active

Check the controller manager:

```bash
ros2 control list_controllers
```

The launch file starts `joint_state_broadcaster` and `diff_drive_controller`
after a delay. Confirm that `gz_ros2_control` is loaded and that the wheel
joint names in `controllers.yaml` match the Xacro:
`wheel_left_joint` and `wheel_right_joint`.

### AMCL or Nav2 cannot localize

Confirm all of the following:

- `/clock`, `/scan`, `/odom`, `/tf`, and `/tf_static` are publishing;
- RViz's fixed frame is `map`;
- the robot is placed in a free part of the map;
- the initial pose is set with **2D Pose Estimate**; and
- the map origin and world geometry correspond.

### No obstacle appears in the costmap

Check `/scan` with `ros2 topic echo /scan --once`, then verify the LiDAR frame
and the `base_scan -> base_footprint` transform. The local and global costmaps
subscribe to `/scan` and limit obstacle marking to the configured ranges in
`nav2_params.yaml`.

### RViz opens with missing displays

The navigation RViz file assumes Nav2 topics and frames are available. Start
the full bringup or add the missing displays after the relevant nodes are
running. Use `robot_gazebo/rviz/config.rviz` when inspecting the Gazebo-only
robot view.

## Development notes

- Keep simulation time enabled consistently. Gazebo publishes `/clock`, and
  the launch files pass `use_sim_time: true` to simulation and Nav2 nodes.
- When changing a topic, frame, joint name, or controller interface, update the
  Xacro, bridge, controller configuration, Nav2 parameters, and RViz displays
  together.
- The package CMake files install launch files and runtime assets into the
  package share directories. New worlds, maps, meshes, or configuration files
  must be included in the relevant `install(DIRECTORY ...)` rule.
- Avoid depending on source-tree paths at runtime. Use package share paths and
  `FindPackageShare`/`get_package_share_directory`, as the existing launch
  files do.
- The repository currently has no dedicated unit or integration test suite.
  Build, Xacro expansion, lint, topic, TF, controller, and navigation checks
  are the practical validation steps.
