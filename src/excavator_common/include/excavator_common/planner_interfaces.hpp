#ifndef EXCAVATOR_COMMON__PLANNER_INTERFACES_HPP_
#define EXCAVATOR_COMMON__PLANNER_INTERFACES_HPP_

#include <functional>
#include <string>

#include "excavator_common/joint_trajectory.hpp"

namespace tramcar {
namespace planning {

// Optional collision/validity query supplied by the planning pipeline. It
// returns true when the joint state is valid. The callback must be cheap
// enough for the planner's finite detour search and must not perform planning.
using StateValidityCheck =
    std::function<bool(const JointPositions&, std::string*)>;

// Optional cancellation/timeout hook checked inside planner loops.
using ContinueCheck = std::function<bool()>;

struct PlanningRequest {
  JointPositions start;
  JointPositions goal;
  std::string group_name;
  double timeout_seconds{1.0};
  StateValidityCheck state_validity_check;
  ContinueCheck should_continue;
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
