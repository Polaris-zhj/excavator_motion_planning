#ifndef EXCAVATOR_COMMON__JOINT_LIMITS_HPP_
#define EXCAVATOR_COMMON__JOINT_LIMITS_HPP_

#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include "excavator_common/joint_trajectory.hpp"

namespace tramcar {
namespace planning {

struct JointLimit {
  double lower{-INFINITY};
  double upper{INFINITY};
  double max_velocity{INFINITY};
  double max_acceleration{INFINITY};
  bool continuous{false};
};

struct LimitCheckResult {
  bool ok{false};
  std::size_t joint_index{0};
  std::string message;
};

inline LimitCheckResult CheckJointPositionLimits(
    const JointPositions& positions, const std::vector<JointLimit>& limits,
    double tolerance = 1.0e-9) {
  if (positions.size() != limits.size() || !std::isfinite(tolerance) ||
      tolerance < 0.0) {
    return {false, 0, "joint positions and limits must have matching sizes"};
  }
  for (std::size_t index = 0; index < positions.size(); ++index) {
    if (!std::isfinite(positions[index])) {
      return {false, index, "joint position is not finite"};
    }
    if (!limits[index].continuous &&
        (positions[index] < limits[index].lower - tolerance ||
         positions[index] > limits[index].upper + tolerance)) {
      return {false, index, "joint position violates position limits"};
    }
  }
  return {true, 0, ""};
}

inline LimitCheckResult CheckJointVelocityLimits(
    const JointPositions& velocities, const std::vector<JointLimit>& limits,
    double tolerance = 1.0e-9) {
  if (velocities.size() != limits.size() || !std::isfinite(tolerance) ||
      tolerance < 0.0) {
    return {false, 0, "joint velocities and limits must have matching sizes"};
  }
  for (std::size_t index = 0; index < velocities.size(); ++index) {
    if (!std::isfinite(velocities[index])) {
      return {false, index, "joint velocity is not finite"};
    }
    if (std::isfinite(limits[index].max_velocity) &&
        std::abs(velocities[index]) > limits[index].max_velocity + tolerance) {
      return {false, index, "joint velocity exceeds its limit"};
    }
  }
  return {true, 0, ""};
}

}  // namespace planning
}  // namespace tramcar

#endif  // EXCAVATOR_COMMON__JOINT_LIMITS_HPP_
