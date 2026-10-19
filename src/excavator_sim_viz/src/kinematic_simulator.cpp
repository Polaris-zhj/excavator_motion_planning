#include "excavator_sim_viz/kinematic_simulator.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace tramcar {
namespace planning {
namespace {

bool IsFinite(const JointPositions& values) {
  return std::all_of(values.begin(), values.end(),
                     [](double value) { return std::isfinite(value); });
}

}  // namespace

KinematicSimulator::KinematicSimulator(std::vector<std::string> joint_names,
                                       JointPositions initial_positions,
                                       SimulatorLimits limits)
    : joint_names_(std::move(joint_names)),
      initial_positions_(std::move(initial_positions)),
      current_positions_(initial_positions_),
      limits_(std::move(limits)) {
  current_velocities_.assign(joint_names_.size(), 0.0);
}

ValidationResult KinematicSimulator::SubmitTrajectory(
    const JointTrajectory& trajectory) {
  const ValidationResult validation = ValidateJointTrajectory(trajectory);
  if (!validation.ok) {
    return validation;
  }
  if (trajectory.joint_names != joint_names_) {
    return {false, "trajectory joint order does not match simulator configuration"};
  }
  if (trajectory.points.front().t != 0.0) {
    return {false, "trajectory must start at t = 0"};
  }
  if (safety_fault_) {
    return {false, "simulator is in a safety fault; reset the state first"};
  }

  JointPositions current_positions = current_positions_;
  if (has_trajectory_) {
    JointTrajectoryPoint current_sample;
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
  current_positions_ = std::move(current_positions);
  current_velocities_.assign(joint_names_.size(), 0.0);
  return {true, ""};
}

void KinematicSimulator::CancelTrajectory() {
  if (has_trajectory_) {
    JointTrajectoryPoint current_sample;
    if (interpolator_.Sample(trajectory_, elapsed_time_, &current_sample).ok) {
      current_positions_ = std::move(current_sample.positions);
    }
  }
  current_velocities_.assign(joint_names_.size(), 0.0);
  has_trajectory_ = false;
  trajectory_finished_ = false;
  elapsed_time_ = 0.0;
}

ValidationResult KinematicSimulator::SetState(const RobotState& state) {
  if (state.positions.size() != joint_names_.size() ||
      !IsFinite(state.positions)) {
    return {false, "simulator state positions must match the joint count"};
  }
  if (!state.velocities.empty() &&
      (state.velocities.size() != joint_names_.size() ||
       !IsFinite(state.velocities))) {
    return {false, "simulator state velocities must match the joint count"};
  }
  current_positions_ = state.positions;
  current_velocities_ = state.velocities.empty()
                            ? JointPositions(joint_names_.size(), 0.0)
                            : state.velocities;
  initial_positions_ = current_positions_;
  has_trajectory_ = false;
  trajectory_finished_ = false;
  safety_fault_ = false;
  elapsed_time_ = 0.0;
  return {true, ""};
}

ValidationResult KinematicSimulator::Step(double dt, SimulationState* state) {
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
  state->safety_fault = safety_fault_;

  if (!has_trajectory_) {
    state->positions = current_positions_;
    state->velocities = current_velocities_;
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

  JointTrajectoryPoint sample;
  const ValidationResult sample_result =
      interpolator_.Sample(trajectory_, elapsed_time_, &sample);
  if (!sample_result.ok) {
    return sample_result;
  }
  state->elapsed_time = elapsed_time_;
  current_positions_ = sample.positions;
  current_velocities_ =
      trajectory_finished_
          ? JointPositions(joint_names_.size(), 0.0)
          : sample.velocities;
  state->positions = current_positions_;
  state->velocities = current_velocities_;
  state->trajectory_active = !trajectory_finished_;
  state->trajectory_finished = trajectory_finished_;
  return {true, ""};
}

ValidationResult KinematicSimulator::CheckCommandLimits(
    const ControlCommand& command, double dt, ControlCommand* limited_command,
    bool* limited) const {
  *limited = false;
  if (command.velocities.size() != joint_names_.size() ||
      !IsFinite(command.velocities)) {
    return {false, "command velocity must be finite and match the joint count"};
  }
  limited_command->positions.clear();
  limited_command->has_positions = false;
  limited_command->velocities = command.velocities;

  for (std::size_t joint = 0; joint < joint_names_.size(); ++joint) {
    double velocity = limited_command->velocities[joint];
    if (limits_.max_acceleration.size() == joint_names_.size()) {
      const double max_delta = limits_.max_acceleration[joint] * dt;
      const double delta = velocity - current_velocities_[joint];
      const double clamped_delta =
          std::clamp(delta, -max_delta, max_delta);
      if (clamped_delta != delta) {
        *limited = true;
      }
      velocity = current_velocities_[joint] + clamped_delta;
    }
    if (limits_.max_velocity.size() == joint_names_.size()) {
      const double max_velocity = limits_.max_velocity[joint];
      const double clamped_velocity =
          std::clamp(velocity, -max_velocity, max_velocity);
      if (clamped_velocity != velocity) {
        *limited = true;
      }
      velocity = clamped_velocity;
    }
    limited_command->velocities[joint] = velocity;
  }
  return {true, ""};
}

ValidationResult KinematicSimulator::ApplyCommand(
    const ControlCommand& command, double dt, SimulationState* state) {
  if (state == nullptr) {
    return {false, "simulation state output pointer is null"};
  }
  if (!std::isfinite(dt) || dt <= 0.0) {
    return {false, "simulation command dt must be finite and positive"};
  }
  if (safety_fault_) {
    return {false, "simulator is in a safety fault; reset the state first"};
  }
  if (has_trajectory_) {
    return {false,
            "command mode cannot be used while a direct trajectory is active"};
  }
  if (command.has_positions) {
    return {false, "position commands are not supported by the kinematic simulator"};
  }

  ControlCommand limited_command;
  bool limited = false;
  const ValidationResult limit_check =
      CheckCommandLimits(command, dt, &limited_command, &limited);
  if (!limit_check.ok) {
    return limit_check;
  }

  JointPositions next_positions = current_positions_;
  for (std::size_t joint = 0; joint < joint_names_.size(); ++joint) {
    next_positions[joint] += limited_command.velocities[joint] * dt;
    if (limits_.lower_position_limits.size() == joint_names_.size() &&
        (next_positions[joint] < limits_.lower_position_limits[joint] ||
         next_positions[joint] > limits_.upper_position_limits[joint])) {
      safety_fault_ = true;
      current_velocities_.assign(joint_names_.size(), 0.0);
      state->joint_names = joint_names_;
      state->elapsed_time = elapsed_time_;
      state->positions = current_positions_;
      state->velocities = current_velocities_;
      state->trajectory_active = false;
      state->trajectory_finished = false;
      state->safety_fault = true;
      return {false,
              "command would exceed the position limit of joint " +
                  joint_names_[joint] + "; safety stop engaged"};
    }
  }

  current_positions_ = std::move(next_positions);
  current_velocities_ = limited_command.velocities;
  elapsed_time_ += dt;

  state->joint_names = joint_names_;
  state->elapsed_time = elapsed_time_;
  state->positions = current_positions_;
  state->velocities = current_velocities_;
  state->trajectory_active = false;
  state->trajectory_finished = false;
  state->safety_fault = false;
  return {true, limited ? "command applied with limiting" : "command applied"};
}

const JointPositions& KinematicSimulator::positions() const {
  return current_positions_;
}

const JointPositions& KinematicSimulator::velocities() const {
  return current_velocities_;
}

bool KinematicSimulator::HasSafetyFault() const { return safety_fault_; }

}  // namespace planning
}  // namespace tramcar
