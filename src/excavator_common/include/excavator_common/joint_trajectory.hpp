#ifndef EXCAVATOR_COMMON__JOINT_TRAJECTORY_HPP_
#define EXCAVATOR_COMMON__JOINT_TRAJECTORY_HPP_

#include <cmath>
#include <cstddef>
#include <string>
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

inline ValidationResult ValidateJointPath(const JointPath& path) {
  if (path.joint_names.empty()) {
    return {false, "joint_names must not be empty"};
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

inline ValidationResult ValidateJointTrajectory(
    const JointTrajectory& trajectory) {
  if (trajectory.joint_names.empty()) {
    return {false, "joint_names must not be empty"};
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
