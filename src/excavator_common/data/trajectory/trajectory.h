/******************************************************************************
 * Copyright 2026 The CIDI Authors. All Rights Reserved.
 *****************************************************************************/
#pragma once
#include "common/data/trajectory/cartesian_trajectory.h"
#include "common/data/trajectory/joint_trajectory.h"
#include <mutex>

namespace excavator {
namespace planning {
class Trajectory {
public:
  Trajectory() = default;
  virtual ~Trajectory() {}

  data::JointTrajectory swing_joint_traj() const;

  data::JointTrajectory boom_joint_traj() const;

  data::JointTrajectory arm_joint_traj() const;

  data::JointTrajectory bucket_joint_traj() const;

  data::CartesianTrajectoryInfo boom_carte_traj_info() const;

  data::CartesianTrajectoryInfo arm_carte_traj_info() const;

  void set_swing_traj(const data::JointTrajectory &joint_taj);

  void set_boom_traj(const data::JointTrajectory &joint_taj);
  void set_boom_traj(const data::CartesianTrajectoryInfo &carte_traj);

  void set_arm_traj(const data::JointTrajectory &joint_taj);
  void set_arm_traj(const data::CartesianTrajectoryInfo &carte_traj);

  void set_bucket_traj(const data::JointTrajectory &joint_taj);

private:
  data::JointTrajectory swing_joint_traj_, boom_joint_traj_, arm_joint_traj_,
      bucket_joint_traj_;
  data::CartesianTrajectoryInfo boom_carte_traj_info_, arm_carte_traj_info_;

  mutable std::mutex swing_mutex_, boom_mutex_, arm_mutex_, bucket_mutex_;
};
} // namespace planning
} // namespace excavator