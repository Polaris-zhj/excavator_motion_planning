# Collision Detection Module

This directory contains the replaceable collision implementation used by the
\`excavator_collision\` ROS 2 package.

## Directory structure

\`\`\`text
collision_detection/
├── README.md
├── include/excavator_collision/internal/
│   └── robot_collision_model.hpp
├── src/
│   ├── collision_checker.cpp
│   └── robot_collision_model.cpp
└── third_library/
    └── fcl/
        ├── fcl_collision_model.hpp
        └── fcl_collision_model.cpp
\`\`\`

## Responsibility boundaries

- \`RobotCollisionModel\` is a project-owned description of collision shapes
  attached to robot links. It stores each geometry's fixed local transform and
  has no FCL dependency.
- \`KinematicsModel\` remains independent. At query time the checker requests
  link poses from the configured FK implementation.
- \`FclCollisionModel\` is the only layer that includes FCL. It converts the
  project geometry records to FCL geometry, creates query objects and performs
  collision/distance queries.
- \`UrdfFclCollisionChecker\` implements the stable \`CollisionChecker\`
  interface, coordinates FK, scene validation and path sampling.

The transform used for each robot collision geometry is:

\`\`\`text
T_world_collision = T_world_link * T_link_collision
\`\`\`

Here \`T_world_link\` is returned by \`KinematicsModel::GetLinkPose\`, while
\`T_link_collision\` comes from the URDF collision element origin. FK is not
recomputed by the FCL adapter.

FCL collision objects are constructed per query from immutable geometry
resources and the supplied FK poses. This avoids sharing mutable collision
objects between concurrent checks.

## Implemented query contract

- \`CheckState\`: self-collision and environment-collision check.
- \`CheckSelfCollision\`: robot self-collision only.
- \`CheckEnvironmentCollision\`: environment-collision only.
- \`CheckPath\`: linearly samples joint-space path segments and checks both
  collision scopes at every sampled configuration.
- \`CheckPathCollisionWithObstacle\`: samples a path against one selected
  environment obstacle.
- \`CheckTrajectory\`: validates a trajectory and checks its sampled states.
- \`ComputeDistanceToObstacle\`: minimum robot-to-obstacle distance at one
  joint configuration.

Self-collision ignores geometries on the same link, direct parent-child link
pairs, and user-configured allowed-collision pairs.

## Supported input and current limitations

Robot collision geometry is loaded from URDF:
- Box, sphere and cylinder primitives.
- Binary or ASCII STL triangle meshes referenced by URDF mesh elements.
- \`package://\` mesh URIs and direct filesystem paths.

Environment objects currently use the \`PlanningScene\` message's
\`shape_msgs/SolidPrimitive\` representation:
- Box, sphere and cylinder.

The current environment adapter does not yet ingest raw point clouds, OctoMap,
voxel grids, triangle-mesh obstacles or moving-obstacle prediction. Those need
a project-owned scene conversion path or another backend implementation.
Unsupported geometry fails the query rather than being silently ignored.

The scene \`header.frame_id\` must be empty or equal to the URDF root frame.
No TF transform is applied in this module. The caller must provide obstacle
poses in that frame.

Path checking is discrete sampling controlled by \`max_joint_step\`; it does
not constitute a continuous-collision guarantee. A smaller step increases
checking cost and reduces (but does not eliminate) the chance of missing a
collision between samples.

## Backend replacement

The planner depends only on \`CollisionChecker\`. A future collision algorithm
can implement this interface and be registered in
\`CollisionCheckerFactory\`. The planner and controller do not need to include
FCL or branch on FCL-specific types.
