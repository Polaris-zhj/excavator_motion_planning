/******************************************************************************
 * Copyright 2026 The CIDI Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/data/trajectory/trajectory.h"

namespace excavator {
namespace planning {
data::JointTrajectory Trajectory::swing_joint_traj() const {
  std::lock_guard<std::mutex> lock(swing_mutex_);
  return swing_joint_traj_;
}

data::JointTrajectory Trajectory::boom_joint_traj() const {
  std::lock_guard<std::mutex> lock(boom_mutex_);
  return boom_joint_traj_;
}

data::JointTrajectory Trajectory::arm_joint_traj() const {
  std::lock_guard<std::mutex> lock(arm_mutex_);
  return arm_joint_traj_;
}

data::JointTrajectory Trajectory::bucket_joint_traj() const {
  std::lock_guard<std::mutex> lock(bucket_mutex_);
  return bucket_joint_traj_;
}

data::CartesianTrajectoryInfo Trajectory::boom_carte_traj_info() const {
  std::lock_guard<std::mutex> lock(boom_mutex_);
  return boom_carte_traj_info_;
}

data::CartesianTrajectoryInfo Trajectory::arm_carte_traj_info() const {
  std::lock_guard<std::mutex> lock(arm_mutex_);
  return arm_carte_traj_info_;
}

void Trajectory::set_swing_traj(const data::JointTrajectory &joint_taj) {
  std::lock_guard<std::mutex> lock(swing_mutex_);
  swing_joint_traj_ = joint_taj;
}

void Trajectory::set_boom_traj(const data::JointTrajectory &joint_taj) {
  std::lock_guard<std::mutex> lock(boom_mutex_);
  boom_joint_traj_ = joint_taj;
  // TODO 需要同步更新笛卡尔轨迹
}
void Trajectory::set_boom_traj(
    const data::CartesianTrajectoryInfo &carte_traj) {
  std::lock_guard<std::mutex> lock(boom_mutex_);
  boom_carte_traj_info_ = carte_traj;
  // TODO 需要同步更新关节轨迹
}

void Trajectory::set_arm_traj(const data::JointTrajectory &joint_taj) {
  std::lock_guard<std::mutex> lock(arm_mutex_);
  arm_joint_traj_ = joint_taj;
}
void Trajectory::set_arm_traj(const data::CartesianTrajectoryInfo &carte_traj) {
  std::lock_guard<std::mutex> lock(arm_mutex_);
  arm_carte_traj_info_ = carte_traj;
}

void Trajectory::set_bucket_traj(const data::JointTrajectory &joint_taj) {
  std::lock_guard<std::mutex> lock(bucket_mutex_);
  bucket_joint_traj_ = joint_taj;
}

} // namespace planning
} // namespace excavator