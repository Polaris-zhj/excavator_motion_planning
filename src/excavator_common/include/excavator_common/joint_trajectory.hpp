#ifndef EXCAVATOR_COMMON__JOINT_TRAJECTORY_HPP_
#define EXCAVATOR_COMMON__JOINT_TRAJECTORY_HPP_

#include <cmath>
#include <cstddef>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace tramcar {
namespace planning {

using JointPositions = std::vector<double>;

struct JointPath {
  std::string path_id;
  std::vector<std::string> joint_names;
  std::vector<JointPositions> points;
};

struct JointTrajectoryPoint {
  double t{0.0};
  JointPositions positions;
  JointPositions velocities;
  JointPositions accelerations;
};

struct JointTrajectory {
  std::string path_id;
  std::vector<std::string> joint_names;
  std::vector<JointTrajectoryPoint> points;
};

struct ValidationResult {
  bool ok{false};
  std::string message;
};

inline ValidationResult ValidateJointNames(
    const std::vector<std::string>& joint_names) {
  if (joint_names.empty()) {
    return {false, "joint_names must not be empty"};
  }
  std::unordered_set<std::string> unique_names;
  for (const std::string& name : joint_names) {
    if (name.empty()) {
      return {false, "joint_names must not contain empty names"};
    }
    if (!unique_names.insert(name).second) {
      return {false, "joint_names must not contain duplicates"};
    }
  }
  return {true, ""};
}

inline ValidationResult ValidateJointPath(const JointPath& path) {
  const ValidationResult names_validation = ValidateJointNames(path.joint_names);
  if (!names_validation.ok) {
    return names_validation;
  }
  if (path.points.empty()) {
    return {false, "path must contain at least one point"};
  }
  for (std::size_t point_index = 0; point_index < path.points.size();
       ++point_index) {
    const JointPositions& positions = path.points[point_index];
    if (positions.size() != path.joint_names.size()) {
      return {false, "path point joint count does not match joint_names"};
    }
    for (double position : positions) {
      if (!std::isfinite(position)) {
        return {false, "path contains a non-finite joint position"};
      }
    }
  }
  return {true, ""};
}

// Checks finite-difference motion limits on the piecewise-linear position
// trajectory. This does not guarantee continuous acceleration or jerk limits.
inline ValidationResult ValidateJointTrajectoryLimits(
    const JointTrajectory& trajectory, const JointPositions& max_velocity,
    const JointPositions& max_acceleration, double tolerance = 1.0e-9) {
  const ValidationResult trajectory_validation =
      ValidateJointTrajectory(trajectory);
  if (!trajectory_validation.ok) {
    return trajectory_validation;
  }
  const std::size_t joint_count = trajectory.joint_names.size();
  if (max_velocity.size() != joint_count ||
      max_acceleration.size() != joint_count || !std::isfinite(tolerance) ||
      tolerance < 0.0) {
    return {false, "motion limits must match joint count and tolerance must be non-negative"};
  }
  for (std::size_t joint = 0; joint < joint_count; ++joint) {
    if (!std::isfinite(max_velocity[joint]) || max_velocity[joint] <= 0.0 ||
        !std::isfinite(max_acceleration[joint]) ||
        max_acceleration[joint] <= 0.0) {
      return {false, "motion limits must be finite and positive"};
    }
  }

  const std::size_t point_count = trajectory.points.size();
  for (std::size_t index = 1; index < point_count; ++index) {
    const double dt =
        trajectory.points[index].t - trajectory.points[index - 1].t;
    for (std::size_t joint = 0; joint < joint_count; ++joint) {
      const double velocity =
          (trajectory.points[index].positions[joint] -
           trajectory.points[index - 1].positions[joint]) / dt;
      if (std::abs(velocity) > max_velocity[joint] + tolerance) {
        return {false, "trajectory segment exceeds a joint velocity limit"};
      }
    }
  }

  for (std::size_t index = 1; index + 1 < point_count; ++index) {
    const double previous_dt =
        trajectory.points[index].t - trajectory.points[index - 1].t;
    const double next_dt =
        trajectory.points[index + 1].t - trajectory.points[index].t;
    const double total_dt = previous_dt + next_dt;
    for (std::size_t joint = 0; joint < joint_count; ++joint) {
      const double previous_velocity =
          (trajectory.points[index].positions[joint] -
           trajectory.points[index - 1].positions[joint]) / previous_dt;
      const double next_velocity =
          (trajectory.points[index + 1].positions[joint] -
           trajectory.points[index].positions[joint]) / next_dt;
      const double acceleration =
          2.0 * (next_velocity - previous_velocity) / total_dt;
      if (std::abs(acceleration) > max_acceleration[joint] + tolerance) {
        return {false, "trajectory waypoint exceeds a joint acceleration limit"};
      }
    }
  }
  return {true, ""};
}

inline ValidationResult ValidateJointTrajectory(
    const JointTrajectory& trajectory) {
  const ValidationResult names_validation =
      ValidateJointNames(trajectory.joint_names);
  if (!names_validation.ok) {
    return names_validation;
  }
  if (trajectory.points.empty()) {
    return {false, "trajectory must contain at least one point"};
  }

  if (trajectory.points.front().t != 0.0) {
    return {false, "first trajectory point time t must be zero"};
  }

  double previous_t = -1.0;
  for (std::size_t point_index = 0; point_index < trajectory.points.size();
       ++point_index) {
    const JointTrajectoryPoint& point = trajectory.points[point_index];
    if (!std::isfinite(point.t) || point.t < 0.0) {
      return {false, "point time t must be finite and non-negative"};
    }
    if (point_index > 0 && point.t <= previous_t) {
      return {false, "point time t must be strictly increasing"};
    }
    if (point.positions.size() != trajectory.joint_names.size()) {
      return {false, "point position count does not match joint_names"};
    }
    if (!point.velocities.empty() &&
        point.velocities.size() != trajectory.joint_names.size()) {
      return {false, "point velocity count does not match joint_names"};
    }
    if (!point.accelerations.empty() &&
        point.accelerations.size() != trajectory.joint_names.size()) {
      return {false, "point acceleration count does not match joint_names"};
    }
    for (double position : point.positions) {
      if (!std::isfinite(position)) {
        return {false, "trajectory contains a non-finite joint position"};
      }
    }
    for (double velocity : point.velocities) {
      if (!std::isfinite(velocity)) {
        return {false, "trajectory contains a non-finite joint velocity"};
      }
    }
    for (double acceleration : point.accelerations) {
      if (!std::isfinite(acceleration)) {
        return {false, "trajectory contains a non-finite joint acceleration"};
      }
    }
    previous_t = point.t;
  }
  return {true, ""};
}

}  // namespace planning
}  // namespace tramcar

#endif  // EXCAVATOR_COMMON__JOINT_TRAJECTORY_HPP_
