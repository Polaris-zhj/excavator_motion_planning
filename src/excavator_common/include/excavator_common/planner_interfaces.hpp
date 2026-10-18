#ifndef EXCAVATOR_COMMON__PLANNER_INTERFACES_HPP_
#define EXCAVATOR_COMMON__PLANNER_INTERFACES_HPP_

#include <string>

#include "excavator_common/joint_trajectory.hpp"

namespace tramcar {
namespace planning {

struct PlanningRequest {
  JointPositions start;
  JointPositions goal;
  std::string group_name;
  double timeout_seconds{1.0};
};

struct PlanningResponse {
  bool success{false};
  JointPath path;
  std::string message;
};

class PathPlanner {
 public:
  virtual ~PathPlanner() = default;

  virtual PlanningResponse Plan(const PlanningRequest& request) = 0;
};

struct TimeParameterizationRequest {
  JointPath path;
  JointPositions max_velocity;
  JointPositions max_acceleration;
};

struct TimeParameterizationResponse {
  bool success{false};
  JointTrajectory trajectory;
  std::string message;
};

class TimeParameterizer {
 public:
  virtual ~TimeParameterizer() = default;

  virtual TimeParameterizationResponse Parameterize(
      const TimeParameterizationRequest& request) = 0;
};

class TrajectoryInterpolator {
 public:
  virtual ~TrajectoryInterpolator() = default;

  virtual ValidationResult Sample(const JointTrajectory& trajectory, double t,
                                  JointTrajectoryPoint* point) const = 0;
};

}  // namespace planning
}  // namespace tramcar

#endif  // EXCAVATOR_COMMON__PLANNER_INTERFACES_HPP_
