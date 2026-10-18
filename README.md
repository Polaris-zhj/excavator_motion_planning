# Excavator Motion Planning

A ROS 2 Humble / C++17 framework for excavator kinematic simulation and replaceable joint-space planning interfaces.

The first milestone deliberately focuses on **kinematic motion only**. Hydraulic cylinders, hydraulic pressure/flow, rigid-body dynamics, soil interaction, and a runtime dependency on MoveIt 2 are out of scope.

## Design principles

- C++17 and Google C++ Style.
- Keep planning algorithms behind interfaces so RRT/RRT*/PRM or a custom planner can be swapped without changing the simulator.
- Keep model-dependent configuration separate from generic trajectory data.
- Use ROS 2 `sensor_msgs/msg/JointState` for simulated joint state; `robot_state_publisher` derives TF from the robot description.
- Do not assume excavator joint names or link names until the model repository is populated and inspected.

## Workspace layout

```text
excavator_motion_planning/
├── src/
│   ├── excavator_common/       # Generic joint path/trajectory data types
│   └── excavator_sim_viz/      # Kinematic trajectory playback and JointState publishing
├── test/
└── README.md
```

## Dependencies

- Ubuntu 22.04
- ROS 2 Humble
- `rclcpp`
- `sensor_msgs`
- `trajectory_msgs`

Install the ROS dependencies:

```bash
sudo apt update
sudo apt install ros-humble-rclcpp ros-humble-sensor-msgs ros-humble-trajectory-msgs
```

## Build

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/excavator_ws/src
# Clone this repository into ~/excavator_ws/src/excavator_motion_planning
cd ~/excavator_ws
colcon build --symlink-install
source install/setup.bash
```

## Current implementation status

- [x] Initial repository and ROS package skeleton
- [x] Generic joint path and trajectory data structures
- [x] Joint-space trajectory playback node publishing `/joint_states`
- [ ] Inspect and integrate the actual excavator URDF/mesh assets
- [ ] Validate joint order, limits, and end-effector frame against the actual model
- [ ] Add unit tests and RViz launch configuration
- [ ] Add kinematics interfaces and a baseline FK implementation

## Trajectory input

The playback node subscribes to `/joint_trajectory` (`trajectory_msgs/msg/JointTrajectory`) and publishes `/joint_states` (`sensor_msgs/msg/JointState`). The incoming trajectory's `joint_names` defines the joint ordering. Each point's `time_from_start` is treated as elapsed time from trajectory start.

This is a first usable simulation scaffold, not yet a complete planner or a validated excavator model integration.
