# Excavator Motion Planning

A ROS 2 Humble / C++17 framework for excavator kinematic simulation and replaceable joint-space planning interfaces. The project follows Google C++ Style and does not require MoveIt 2 at runtime.

The first milestone is **kinematic simulation**, not hydraulic or rigid-body dynamics. Hydraulic cylinder actuation, hydraulic pressure/flow, soil contact, and dynamics are intentionally out of scope.

## Model included in this repository

The `excavator_description` package is included under `src/excavator_description/`, including the URDF/Xacro and mesh assets. The description declares the four primary planning joints:

- `swing_joint`
- `boom_joint`
- `stick_joint`
- `bucket_joint`

The Xacro also defines the `bucket_tip` frame for end-effector queries. The original model's package metadata declares BSD-3-Clause; see [model attribution and licensing](src/excavator_description/NOTICE.md). Most mesh assets were copied from [Polaris-zhj/ws_excavator](https://github.com/Polaris-zhj/ws_excavator). The source `base_link.STL` was too large for the available GitHub content transfer path, so the migrated package currently uses a small generated chassis/track approximation at that filename. Replace it with the original CAD mesh when transferring the full source file through a local Git clone.

## Packages

- `excavator_description`: URDF/Xacro and meshes.
- `excavator_interfaces`: ROS 2 messages and `PlanTrajectory` / `ExecuteTrajectory` action definitions.
- `excavator_common`: `JointPath`, `JointTrajectory`, robot state/configuration, joint-limit checks, validation helpers, and replaceable planner interfaces.
- `kinematics`: stateless URDF-backed FK, geometric Jacobian, and damped-least-squares IK.
- `excavator_planning`: baseline straight-line joint-space path planner, conservative segment time assignment, trajectory sampler, and `/plan_trajectory` action server.
- `excavator_sim_viz`: trajectory playback via `/joint_trajectory` and `/joint_states`, plus RViz launch configuration.

The straight-line planner is a baseline only: it does not perform collision avoidance. The time parameterizer is conservative segment timing, not a full time-optimal or jerk-limited algorithm. Validate model limits and behavior before real-world use.

## Requirements and build

- Ubuntu 22.04
- ROS 2 Humble
- C++17, Eigen, URDF, colcon

Install dependencies and build:

```bash
sudo apt update
sudo apt install ros-humble-desktop ros-humble-urdf ros-humble-xacro \
  ros-humble-robot-state-publisher ros-humble-rviz2 libeigen3-dev
source /opt/ros/humble/setup.bash
mkdir -p ~/excavator_ws/src
cd ~/excavator_ws/src
git clone https://github.com/Polaris-zhj/excavator_motion_planning.git
cd ~/excavator_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Plan a joint-space trajectory\n\nStart the planning action server in one terminal:\n\n```bash\nros2 launch excavator_planning planning_server.launch.py\n```\n\nSend a baseline request from the model home pose to a nearby joint configuration (the response explicitly states that collision avoidance is not implemented):\n\n```bash\nros2 action send_goal /plan_trajectory excavator_interfaces/action/PlanTrajectory \\\n"{start_state: {name: [swing_joint, boom_joint, stick_joint, bucket_joint], position: [0.0, -0.35, 0.45, -0.4]}, goal_state: {name: [swing_joint, boom_joint, stick_joint, bucket_joint], position: [0.2, -0.3, 0.4, -0.3]}, target_link: bucket_tip, allowed_planning_time: {sec: 1, nanosec: 0}, planning_scene: {is_diff: false, objects: []}}"\n```\n\nThe action rejects requests with obstacle objects because no FCL collision backend is implemented yet. A successful baseline result is not a collision-safety guarantee. The generated trajectory can be sent to `/joint_trajectory` for playback.\n\n## Visualize and play a trajectory

Start the model, RViz, and trajectory player:

```bash
ros2 launch excavator_sim_viz model_view.launch.py
```

In a second terminal, publish a sample trajectory. Joint order must match `joint_names`:

```bash
ros2 topic pub --once /joint_trajectory trajectory_msgs/msg/JointTrajectory \
"{joint_names: [swing_joint, boom_joint, stick_joint, bucket_joint], points: [
  {positions: [0.0, -0.35, 0.45, -0.4], time_from_start: {sec: 0, nanosec: 0}},
  {positions: [0.4, -0.2, 0.3, -0.4], time_from_start: {sec: 3, nanosec: 0}},
  {positions: [0.0, -0.35, 0.45, -0.4], time_from_start: {sec: 6, nanosec: 0}}
]}"
```

The playback node linearly interpolates trajectory positions and publishes `/joint_states` at 50 Hz. A new trajectory replaces the active trajectory. `robot_state_publisher` derives TF from the single JointState stream.

To validate the Xacro separately:

```bash
xacro src/excavator_description/urdf/excavator.urdf.xacro > /tmp/excavator.urdf
```

## Kinematics API\n\nThe `kinematics` package loads expanded URDF XML or a URDF file, computes stateless FK relative to `base_link` when that link exists, computes a geometric Jacobian, exposes an SVD singularity metric, and provides damped-least-squares IK. The `planning_server` uses the Xacro-expanded robot description and extracts joint position/velocity limits from the model.

After converting Xacro to URDF (or using a generated URDF file), the API can load it without holding a mutable current joint state:

```cpp
#include "kinematics/urdf_kinematics_model.hpp"

tramcar::planning::UrdfKinematicsModel kinematics;
const auto load = kinematics.InitializeFromFile("/tmp/excavator.urdf");
if (!load.ok) {
  // Handle load.message.
}

tramcar::planning::Pose3 pose;
const auto fk = kinematics.GetLinkPose(
    {0.0, -0.35, 0.45, -0.4}, "bucket_tip", &pose);
```

The default joint ordering is swing, boom, stick, bucket. Use the actual model limits and verify the coordinate frames before using results for planning or control.

## Tests and current limitations

Run unit tests with:

```bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

CI builds the ROS 2 workspace, runs unit tests, and validates Xacro. Remaining validation includes running the workspace on Ubuntu 22.04 + ROS 2 Humble, checking the rendered model/TF in RViz, replacing the base-link fallback with the exact upstream CAD mesh, and extending path/time validation with collision checking, interpolation deviation checks, and full acceleration/jerk constraints.
