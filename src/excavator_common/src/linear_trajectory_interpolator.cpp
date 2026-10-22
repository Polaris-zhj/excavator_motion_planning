#include "excavator_common/linear_trajectory_interpolator.hpp"

#include <cmath>
#include <cstddef>

namespace tramcar {
namespace planning {

ValidationResult LinearTrajectoryInterpolator::Sample(
    const JointTrajectory& trajectory, double t,
    JointTrajectoryPoint* point) const {
  const ValidationResult validation = ValidateJointTrajectory(trajectory);
  if (!validation.ok) {
    return validation;
  }
  if (point == nullptr || !std::isfinite(t)) {
    return {false, "sample output pointer and time must be valid"};
  }
  if (t < trajectory.points.front().t || t > trajectory.points.back().t) {
    return {false, "sample time is outside the trajectory interval"};
  }
  if (t == trajectory.points.front().t) {
    *point = trajectory.points.front();
  } else if (t == trajectory.points.back().t) {
    *point = trajectory.points.back();
  } else {
    bool found = false;
    for (std::size_t index = 1; index < trajectory.points.size(); ++index) {
      const auto& first = trajectory.points[index - 1];
      const auto& second = trajectory.points[index];
      if (t <= second.t) {
        const double delta_t = second.t - first.t;
        const double ratio = (t - first.t) / delta_t;
        const std::size_t joint_count = first.positions.size();
        const bool has_velocities =
            first.velocities.size() == joint_count &&
            second.velocities.size() == joint_count;
        const bool has_accelerations =
            first.accelerations.size() == joint_count &&
            second.accelerations.size() == joint_count;
        point->t = t;
        point->positions.resize(joint_count);
        point->velocities.resize(joint_count, 0.0);
        point->accelerations.resize(joint_count, 0.0);
        for (std::size_t joint = 0; joint < joint_count; ++joint) {
          const double delta =
              second.positions[joint] - first.positions[joint];
          point->positions[joint] = first.positions[joint] + ratio * delta;
          point->velocities[joint] =
              has_velocities
                  ? first.velocities[joint] +
                        ratio * (second.velocities[joint] -
                                 first.velocities[joint])
                  : delta / delta_t;
          point->accelerations[joint] =
              has_accelerations
                  ? first.accelerations[joint] +
                        ratio * (second.accelerations[joint] -
                                 first.accelerations[joint])
                  : 0.0;
        }
        found = true;
        break;
      }
    }
    if (!found) {
      return {false, "failed to locate trajectory segment"};
    }
  }

  const std::size_t joint_count = point->positions.size();
  if (point->velocities.size() != joint_count) {
    point->velocities.assign(joint_count, 0.0);
  }
  if (point->accelerations.size() != joint_count) {
    point->accelerations.assign(joint_count, 0.0);
  }
  return {true, ""};
}

}  // namespace planning
}  // namespace tramcar
