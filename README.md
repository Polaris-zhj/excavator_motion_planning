# Excavator Motion Planning

A ROS 2 Humble / C++17 framework for excavator kinematic simulation and replaceable joint-space planning interfaces.

The first milestone deliberately focuses on **kinematic motion only**. Hydraulic cylinder motion, hydraulic pressure/flow, rigid-body dynamics, soil interaction, and a runtime dependency on MoveIt 2 are out of scope.

## Design principles

- C++17 and Google C++ Style.
- Keep planning algorithms behind interfaces so RRT/RRT*/PRM or a custom planner can be swapped without changing the simulator.
- Keep model-dependent configuration separate from generic trajectory data.
- Use ROS 2 `sensor_msgs/msg/JointState` for simulated joint state; `robot_state_publisher` derives TF from the robot description.
- Use the model's four primary planning joints: `swing_joint`, `boom_joint`, `stick_joint`, and `bucket_joint`. The model also contains visual hydraulic-cylinder and linkage joints; these are not part of the first planning group.
- The source model includes a `bucket_tip` frame for Cartesian work targets. Validate the exact geometry and joint limits in RViz before relying on it for production planning.

## Repositories

- This code: https://github.com/Polaris-zhj/excavator_motion_planning
- Robot description and mesh assets: https://github.com/Polaris-zhj/ws_excavator

Clone both repositories into the same ROS 2 workspace so the model's `excavator_description` package and mesh resources are available:

```bash
mkdir -p ~/excavator_ws/src
cd ~/excavator_ws/src
git clone https://github.com/Polaris-zhj/excavator_motion_planning.git
git clone https://github.com/Polaris-zhj/ws_excavator.git
```

## Workspace layout

```text
excavator_motion_planning/
├── src/
│   ├── excavator_common/       # Generic joint path/trajectory data types and interfaces
│   └── excavator_sim_viz/      # Kinematic trajectory playback and JointState publishing
└── README.md
```

## Dependencies

- Ubuntu 22.04
- ROS 2 Humble
- `rclcpp`
- `sensor_msgs`
- `trajectory_msgs`
- `xacro`, `robot_state_publisher`, and `rviz2` for model visualization

Install the runtime dependencies:

```bash
sudo apt update
sudo apt install ros-humble-rclcpp ros-humble-sensor-msgs \
  ros-humble-trajectory-msgs ros-humble-xacro \
  ros-humble-robot-state-publisher ros-humble-rviz2
```

## Build

```bash
source /opt/ros/humble/setup.bash
cd ~/excavator_ws
colcon build --symlink-install
source install/setup.bash
```

## Run trajectory playback

Start the node:

```bash
ros2 launch excavator_sim_viz trajectory_player.launch.py
```

In another terminal, source the workspace and publish a small test trajectory. Joint order must match `joint_names`:

```bash
ros2 topic pub --once /joint_trajectory trajectory_msgs/msg/JointTrajectory \
"{joint_names: [swing_joint, boom_joint, stick_joint, bucket_joint], points: [
  {positions: [0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 0, nanosec: 0}},
  {positions: [0.4, -0.2, 0.3, -0.4], time_from_start: {sec: 3, nanosec: 0}},
  {positions: [0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 6, nanosec: 0}}
]}"
```

The player linearly interpolates between trajectory points and publishes `/joint_states` at 50 Hz by default. When a new trajectory arrives, it replaces the active trajectory. This is kinematic playback, not a dynamics simulation or a planner.

## Data and planner interfaces

- `excavator_common/joint_trajectory.hpp`: `JointPath`, `JointTrajectory`, validation helpers. Internal trajectory time is named `t`.
- `excavator_common/planner_interfaces.hpp`: replaceable `PathPlanner`, `TimeParameterizer`, and `TrajectoryInterpolator` abstract interfaces.
- ROS bridge: `trajectory_msgs/msg/JointTrajectory` input and `sensor_msgs/msg/JointState` output.

## Current implementation status

- [x] Initial repository and ROS package skeleton
- [x] Generic joint path and trajectory data structures and validation
- [x] Replaceable planner/time-parameterization/interpolation interfaces
- [x] Joint-space trajectory playback node publishing `/joint_states`
- [x] Inspected source model: primary joint names and `bucket_tip` frame identified
- [ ] Add model visualization launch that resolves the description package and starts RViz
- [ ] Add stateless FK/Jacobian/IK interfaces and a baseline implementation
- [ ] Add unit tests and CI build verification
- [ ] Validate joint limits and model motion in a live ROS 2 environment

The source repository contains the URDF/Xacro and STL mesh files, but this repository does not duplicate those assets. Keep the original model repository's license and attribution when redistributing model resources.
