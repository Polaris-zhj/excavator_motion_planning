#include "excavator_sim_viz/trajectory_controller.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "excavator_planning/baseline_planners.hpp"

namespace tramcar {
namespace planning {
namespace {

bool IsFinite(const JointPositions& values) {
  return std::all_of(values.begin(), values.end(),
                     [](double value) { return std::isfinite(value); });
}

double MaxDifference(const JointPositions& first,
                     const JointPositions& second) {
  double maximum = 0.0;
  for (std::size_t index = 0; index < first.size(); ++index) {
    maximum = std::max(maximum, std::abs(first[index] - second[index]));
  }
  return maximum;
}

bool IsTerminalState(ExecutionState state) {
  return state == ExecutionState::kCompleted ||
         state == ExecutionState::kCancelled ||
         state == ExecutionState::kSafetyStop ||
         state == ExecutionState::kError;
}

}  // namespace

TrajectoryController::TrajectoryController(
    TrajectoryControllerConfig config,
    std::unique_ptr<ControllerInterface> controller,
    std::unique_ptr<TrajectoryInterpolator> interpolator)
    : config_(std::move(config)),
      controller_(std::move(controller)),
      interpolator_(std::move(interpolator)) {
  if (!interpolator_) {
    interpolator_ = std::make_unique<LinearTrajectoryInterpolator>();
  }
  status_.state = ExecutionState::kIdle;
  status_.message = "idle";
}

double TrajectoryController::Duration() const {
  return active_.points.empty() ? 0.0 : active_.points.back().t;
}

void TrajectoryController::SetState(ExecutionState state,
                                    const std::string& message) {
  status_.state = state;
  status_.message = message;
  status_.duration = Duration();
  status_.elapsed_time = elapsed_time_;
  status_.progress =
      status_.duration > 0.0
          ? std::clamp(status_.elapsed_time / status_.duration, 0.0, 1.0)
          : 0.0;
}

ValidationResult TrajectoryController::ValidateTrajectory(
    const JointTrajectory& trajectory, const RobotState& current,
    bool require_start_continuity) const {
  const std::size_t joint_count = config_.joint_names.size();
  if (joint_count == 0 || controller_ == nullptr || !interpolator_) {
    return {false, "trajectory controller is not configured"};
  }
  if (trajectory.joint_names != config_.joint_names) {
    return {false, "trajectory joint order does not match controller configuration"};
  }
  const ValidationResult structure =
      ValidateJointTrajectoryLimits(trajectory, config_.max_velocity,
                                    config_.max_acceleration);
  if (!structure.ok) {
    return structure;
  }
  if (trajectory.points.size() < 2) {
    return {false, "trajectory must contain at least two points"};
  }
  if (trajectory.points.back().t <= 0.0) {
    return {false, "trajectory duration must be positive"};
  }
  if (config_.lower_position_limits.size() != joint_count ||
      config_.upper_position_limits.size() != joint_count) {
    return {false, "controller position limits must match the joint count"};
  }

  for (std::size_t point_index = 0; point_index < trajectory.points.size();
       ++point_index) {
    const JointPositions& positions = trajectory.points[point_index].positions;
    for (std::size_t joint = 0; joint < joint_count; ++joint) {
      if (positions[joint] < config_.lower_position_limits[joint] ||
          positions[joint] > config_.upper_position_limits[joint]) {
        return {false, "trajectory point " + std::to_string(point_index) +
                           " violates the position limit of joint " +
                           config_.joint_names[joint]};
      }
    }
  }

  if (require_start_continuity) {
    if (current.positions.size() != joint_count ||
        !IsFinite(current.positions)) {
      return {false, "current state must match the controller joint count"};
    }
    const double start_error =
        MaxDifference(trajectory.points.front().positions, current.positions);
    if (start_error > config_.start_continuity_tolerance) {
      return {false,
              "trajectory start is not continuous with the current state"};
    }
  }
  return {true, ""};
}

ValidationResult TrajectoryController::SubmitTrajectory(
    const JointTrajectory& trajectory, const RobotState& current) {
  std::lock_guard<std::mutex> lock(mutex_);
  const ValidationResult validation =
      ValidateTrajectory(trajectory, current, true);
  if (!validation.ok) {
    status_.message = "candidate rejected: " + validation.message;
    return validation;
  }
  candidate_ = trajectory;
  has_candidate_ = true;
  pending_update_ = false;
  if (status_.state != ExecutionState::kRunning &&
      status_.state != ExecutionState::kPaused) {
    elapsed_time_ = 0.0;
    SetState(ExecutionState::kReady,
             "trajectory accepted as candidate; execution not started");
  } else {
    status_.message = "candidate trajectory accepted while execution continues";
  }
  return {true, ""};
}

ValidationResult TrajectoryController::UpdateTrajectory(
    const JointTrajectory& trajectory, const RobotState& current) {
  std::lock_guard<std::mutex> lock(mutex_);
  const ValidationResult validation =
      ValidateTrajectory(trajectory, current, true);
  if (!validation.ok) {
    status_.message = "trajectory update rejected: " + validation.message;
    return validation;
  }
  candidate_ = trajectory;
  has_candidate_ = true;
  if (status_.state == ExecutionState::kRunning ||
      status_.state == ExecutionState::kPaused) {
    pending_update_ = true;
    status_.message = "trajectory update accepted; waiting for cycle boundary";
  } else {
    pending_update_ = false;
    elapsed_time_ = 0.0;
    SetState(ExecutionState::kReady,
             "trajectory update accepted as candidate; execution not started");
  }
  return {true, ""};
}

ValidationResult TrajectoryController::ExecuteTrajectory(
    const JointTrajectory& trajectory, const RobotState& current) {
  std::lock_guard<std::mutex> lock(mutex_);
  const ExecutionState previous_state = status_.state;
  SetState(ExecutionState::kValidating, "validating execution request");
  const ValidationResult validation =
      ValidateTrajectory(trajectory, current, true);
  if (!validation.ok) {
    SetState(previous_state, "execution rejected: " + validation.message);
    return validation;
  }
  candidate_ = trajectory;
  active_ = trajectory;
  has_candidate_ = true;
  has_active_ = true;
  pending_update_ = false;
  elapsed_time_ = 0.0;
  controller_->Reset();
  status_.tracking_error_norm = 0.0;
  SetState(ExecutionState::kRunning, "execution started");
  return {true, ""};
}

ValidationResult TrajectoryController::Pause() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (status_.state != ExecutionState::kRunning) {
    return {false, "pause requires a running execution"};
  }
  SetState(ExecutionState::kPaused, "execution paused");
  return {true, ""};
}

ValidationResult TrajectoryController::Resume() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (status_.state != ExecutionState::kPaused) {
    return {false, "resume requires a paused execution"};
  }
  SetState(ExecutionState::kRunning, "execution resumed");
  return {true, ""};
}

ValidationResult TrajectoryController::Cancel() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_active_ || IsTerminalState(status_.state)) {
    return {false, "no active execution to cancel"};
  }
  pending_update_ = false;
  has_candidate_ = false;
  SetState(ExecutionState::kCancelled, "execution cancelled by request");
  return {true, ""};
}

ValidationResult TrajectoryController::Stop() {
  std::lock_guard<std::mutex> lock(mutex_);
  const bool had_active = has_active_;
  has_active_ = false;
  has_candidate_ = false;
  pending_update_ = false;
  active_ = JointTrajectory{};
  elapsed_time_ = 0.0;
  status_.tracking_error_norm = 0.0;
  SetState(ExecutionState::kIdle,
           had_active ? "execution stopped immediately" : "already idle");
  return {true, ""};
}

void TrajectoryController::ZeroCommand(TrajectoryStepResult* result) const {
  result->has_command = true;
  result->command.velocities.assign(config_.joint_names.size(), 0.0);
}

bool TrajectoryController::ApplyPendingUpdate(const RobotState& current,
                                               std::string* message) {
  if (!pending_update_ || !has_candidate_) {
    return false;
  }
  pending_update_ = false;
  if (current.positions.size() != config_.joint_names.size()) {
    *message = "trajectory update skipped: measured state has wrong dimension";
    return false;
  }
  const double measured_error = MaxDifference(
      candidate_.points.front().positions, current.positions);
  if (measured_error > config_.update_position_continuity_tolerance) {
    *message = "trajectory update skipped: start is discontinuous with the "
               "measured state";
    return false;
  }
  const JointPositions& candidate_velocities =
      candidate_.points.front().velocities;
  if (candidate_velocities.size() == current.velocities.size() &&
      current.velocities.size() == config_.joint_names.size()) {
    const double velocity_error =
        MaxDifference(candidate_velocities, current.velocities);
    if (velocity_error > config_.update_velocity_continuity_tolerance) {
      *message = "trajectory update skipped: start velocity is discontinuous "
                 "with the measured state";
      return false;
    }
  }
  active_ = candidate_;
  has_active_ = true;
  elapsed_time_ = 0.0;
  controller_->Reset();
  *message = "active trajectory replaced at the control-cycle boundary";
  return true;
}

TrajectoryStepResult TrajectoryController::Update(const RobotState& current,
                                                  double dt) {
  std::lock_guard<std::mutex> lock(mutex_);
  TrajectoryStepResult result;
  result.status = status_;

  if (status_.state == ExecutionState::kRunning) {
    std::string update_message;
    if (ApplyPendingUpdate(current, &update_message)) {
      status_.message = update_message;
    } else if (!update_message.empty()) {
      status_.message = update_message;
    }
  }

  if (status_.state == ExecutionState::kIdle ||
      status_.state == ExecutionState::kReady) {
    status_.elapsed_time = elapsed_time_;
    result.ok = true;
    result.status = status_;
    return result;
  }

  if (status_.state == ExecutionState::kPaused) {
    ZeroCommand(&result);
    result.ok = true;
    result.status = status_;
    return result;
  }

  if (IsTerminalState(status_.state)) {
    ZeroCommand(&result);
    result.ok = true;
    result.status = status_;
    return result;
  }

  if (status_.state != ExecutionState::kRunning) {
    SetState(ExecutionState::kError, "controller is in an unexpected state");
    ZeroCommand(&result);
    result.ok = false;
    result.message = status_.message;
    result.status = status_;
    return result;
  }

  if (!std::isfinite(dt) || dt <= 0.0) {
    SetState(ExecutionState::kError, "control dt must be finite and positive");
    ZeroCommand(&result);
    result.ok = false;
    result.message = status_.message;
    result.status = status_;
    return result;
  }
  if (!active_.points.empty() && current.positions.size() !=
                                     config_.joint_names.size()) {
    SetState(ExecutionState::kError,
             "measured state dimension does not match controller configuration");
    ZeroCommand(&result);
    result.ok = false;
    result.message = status_.message;
    result.status = status_;
    return result;
  }

  elapsed_time_ += dt;
  const double duration = Duration();
  if (elapsed_time_ >= duration) {
    elapsed_time_ = duration;
  }

  JointTrajectoryPoint reference;
  const ValidationResult sample =
      interpolator_->Sample(active_, elapsed_time_, &reference);
  if (!sample.ok) {
    SetState(ExecutionState::kError, "trajectory sampling failed: " +
                                         sample.message);
    ZeroCommand(&result);
    result.ok = false;
    result.message = status_.message;
    result.status = status_;
    return result;
  }

  ControlCommand command;
  const ControllerOutput controller_output =
      controller_->ComputeCommand(current, reference, dt, &command);
  if (!controller_output.ok) {
    SetState(ExecutionState::kError,
             "control algorithm failed: " + controller_output.message);
    ZeroCommand(&result);
    result.ok = false;
    result.message = status_.message;
    result.status = status_;
    return result;
  }

  status_.tracking_error_norm = controller_output.tracking_error_norm;
  status_.elapsed_time = elapsed_time_;
  status_.duration = duration;
  status_.progress =
      duration > 0.0 ? std::clamp(elapsed_time_ / duration, 0.0, 1.0) : 0.0;

  const bool time_reached = elapsed_time_ >= duration;
  if (controller_output.tracking_error_norm > config_.max_tracking_error) {
    SetState(ExecutionState::kSafetyStop,
             "tracking error exceeds the safety threshold");
    ZeroCommand(&result);
    result.ok = true;
    result.message = status_.message;
    result.status = status_;
    return result;
  }
  if (time_reached &&
      controller_output.tracking_error_norm <=
          config_.completion_position_tolerance) {
    SetState(ExecutionState::kCompleted,
             "trajectory completed within the position tolerance");
    ZeroCommand(&result);
    result.ok = true;
    result.message = status_.message;
    result.status = status_;
    return result;
  }
  if (elapsed_time_ > duration + config_.execution_timeout_margin) {
    SetState(ExecutionState::kError,
             "execution timed out before reaching the completion tolerance");
    ZeroCommand(&result);
    result.ok = false;
    result.message = status_.message;
    result.status = status_;
    return result;
  }

  status_.message = controller_output.limited
                        ? "running; control command limited"
                        : "running";
  result.ok = true;
  result.has_command = true;
  result.command = std::move(command);
  result.status = status_;
  return result;
}

ExecutionStatusSnapshot TrajectoryController::GetStatus() const {
  std::lock_guard<std::mutex> lock(mutex_);
  ExecutionStatusSnapshot status = status_;
  status.elapsed_time = elapsed_time_;
  status.duration = Duration();
  status.progress =
      status.duration > 0.0
          ? std::clamp(status.elapsed_time / status.duration, 0.0, 1.0)
          : 0.0;
  return status;
}

bool TrajectoryController::HasActiveTrajectory() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return has_active_;
}

}  // namespace planning
}  // namespace tramcar
