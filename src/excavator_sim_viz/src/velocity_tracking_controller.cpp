#include "excavator_sim_viz/velocity_tracking_controller.hpp"

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

VelocityTrackingController::VelocityTrackingController(
    VelocityControllerConfig config)
    : config_(std::move(config)) {}

void VelocityTrackingController::Reset() {}

ControllerOutput VelocityTrackingController::ComputeCommand(
    const RobotState& current, const JointTrajectoryPoint& reference, double dt,
    ControlCommand* command) {
  ControllerOutput output;
  if (command == nullptr) {
    output.message = "control command output pointer is null";
    return output;
  }
  command->positions.clear();
  command->velocities.clear();
  command->has_positions = false;

  const std::size_t joint_count = config_.max_velocity.size();
  if (joint_count == 0 ||
      config_.max_acceleration.size() != joint_count ||
      config_.lower_position_limits.size() != joint_count ||
      config_.upper_position_limits.size() != joint_count) {
    output.message = "controller limits must be non-empty and aligned";
    return output;
  }
  if (current.positions.size() != joint_count ||
      reference.positions.size() != joint_count) {
    output.message = "current state and reference must match the joint count";
    return output;
  }
  if (!IsFinite(current.positions) || !IsFinite(reference.positions) ||
      !IsFinite(config_.max_velocity) || !IsFinite(config_.max_acceleration)) {
    output.message = "current state, reference or limits contain non-finite values";
    return output;
  }
  if (!std::isfinite(dt) || dt <= 0.0) {
    output.message = "control dt must be finite and positive";
    return output;
  }
  for (std::size_t joint = 0; joint < joint_count; ++joint) {
    if (config_.max_velocity[joint] <= 0.0 ||
        config_.max_acceleration[joint] <= 0.0 ||
        config_.lower_position_limits[joint] >
            config_.upper_position_limits[joint]) {
      output.message = "controller limits are invalid";
      return output;
    }
  }

  const bool has_feed_forward =
      reference.velocities.size() == joint_count &&
      IsFinite(reference.velocities);
  JointPositions measured_velocities = current.velocities;
  if (measured_velocities.size() != joint_count) {
    measured_velocities.assign(joint_count, 0.0);
  }
  if (!IsFinite(measured_velocities)) {
    output.message = "current velocity contains non-finite values";
    return output;
  }

  const double margin = std::max(0.0, config_.soft_limit_margin);
  command->velocities.resize(joint_count, 0.0);
  double squared_error = 0.0;
  for (std::size_t joint = 0; joint < joint_count; ++joint) {
    const double error = reference.positions[joint] - current.positions[joint];
    squared_error += error * error;
    double desired =
        (has_feed_forward ? reference.velocities[joint] : 0.0) +
        config_.position_gain * error;

    const double max_velocity = config_.max_velocity[joint];
    if (std::abs(desired) > max_velocity) {
      desired = std::clamp(desired, -max_velocity, max_velocity);
      output.limited = true;
    }

    const double max_delta =
        config_.max_acceleration[joint] * dt;
    const double limited_delta =
        std::clamp(desired - measured_velocities[joint], -max_delta, max_delta);
    if (limited_delta != desired - measured_velocities[joint]) {
      output.limited = true;
    }
    desired = measured_velocities[joint] + limited_delta;

    if (config_.enforce_soft_limits) {
      if (current.positions[joint] >
              config_.upper_position_limits[joint] - margin &&
          desired > 0.0) {
        desired = 0.0;
        output.limited = true;
      }
      if (current.positions[joint] <
              config_.lower_position_limits[joint] + margin &&
          desired < 0.0) {
        desired = 0.0;
        output.limited = true;
      }
    }

    command->velocities[joint] =
        std::clamp(desired, -max_velocity, max_velocity);
  }

  output.tracking_error_norm = std::sqrt(squared_error);
  output.ok = true;
  output.message = output.limited ? "control command limited"
                                  : "control command computed";
  return output;
}

}  // namespace planning
}  // namespace tramcar
