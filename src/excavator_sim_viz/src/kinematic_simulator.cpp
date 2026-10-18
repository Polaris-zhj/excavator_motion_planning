#include "excavator_sim_viz/kinematic_simulator.hpp"

#include <cmath>
#include <utility>

namespace excavator_sim_viz {
namespace planning = tramcar::planning;

KinematicSimulator::KinematicSimulator(
    std::vector<std::string> joint_names,
    planning::JointPositions initial_positions)
    : joint_names_(std::move(joint_names)),
      initial_positions_(std::move(initial_positions)) {}

planning::ValidationResult KinematicSimulator::SubmitTrajectory(
    const planning::JointTrajectory& trajectory) {
  const planning::ValidationResult validation =
      planning::ValidateJointTrajectory(trajectory);
  if (!validation.ok) {
    return validation;
  }
  if (trajectory.joint_names != joint_names_) {
    return {false, "trajectory joint order does not match simulator configuration"};
  }
  if (trajectory.points.front().t != 0.0) {
    return {false, "trajectory must start at t = 0"};
  }

  planning::JointPositions current_positions = initial_positions_;
  if (has_trajectory_) {
    planning::JointTrajectoryPoint current_sample;
    const auto sample_result =
        interpolator_.Sample(trajectory_, elapsed_time_, &current_sample);
    if (!sample_result.ok) {
      return sample_result;
    }
    current_positions = std::move(current_sample.positions);
  }
  constexpr double kMaxStartPositionError = 0.05;
  for (std::size_t joint = 0; joint < joint_names_.size(); ++joint) {
    if (std::abs(trajectory.points.front().positions[joint] -
                 current_positions[joint]) > kMaxStartPositionError) {
      return {false, "trajectory start is discontinuous from current state"};
    }
  }

  trajectory_ = trajectory;
  elapsed_time_ = 0.0;
  has_trajectory_ = true;
  trajectory_finished_ = false;
  return {true, ""};
}

void KinematicSimulator::CancelTrajectory() {
  if (has_trajectory_) {
    planning::JointTrajectoryPoint current_sample;
    if (interpolator_.Sample(trajectory_, elapsed_time_, &current_sample).ok) {
      initial_positions_ = std::move(current_sample.positions);
    }
  }
  has_trajectory_ = false;
  trajectory_finished_ = false;
  elapsed_time_ = 0.0;
}

planning::ValidationResult KinematicSimulator::Step(
    double dt, SimulationState* state) {
  if (state == nullptr) {
    return {false, "simulation state output pointer is null"};
  }
  if (!std::isfinite(dt) || dt < 0.0) {
    return {false, "simulation dt must be finite and non-negative"};
  }

  state->joint_names = joint_names_;
  state->elapsed_time = elapsed_time_;
  state->trajectory_active = has_trajectory_ && !trajectory_finished_;
  state->trajectory_finished = trajectory_finished_;

  if (!has_trajectory_) {
    state->positions = initial_positions_;
    state->velocities.assign(joint_names_.size(), 0.0);
    return {true, ""};
  }

  if (!trajectory_finished_) {
    elapsed_time_ += dt;
    const double final_time = trajectory_.points.back().t;
    if (elapsed_time_ >= final_time) {
      elapsed_time_ = final_time;
      trajectory_finished_ = true;
    }
  }

  planning::JointTrajectoryPoint sample;
  const planning::ValidationResult sample_result =
      interpolator_.Sample(trajectory_, elapsed_time_, &sample);
  if (!sample_result.ok) {
    return sample_result;
  }
  state->elapsed_time = elapsed_time_;
  state->positions = std::move(sample.positions);
  state->velocities = trajectory_finished_
                          ? planning::JointPositions(joint_names_.size(), 0.0)
                          : std::move(sample.velocities);
  state->trajectory_active = !trajectory_finished_;
  state->trajectory_finished = trajectory_finished_;
  return {true, ""};
}

}  // namespace excavator_sim_viz
