#ifndef EXCAVATOR_PLANNING__BASELINE_PLANNERS_HPP_
#define EXCAVATOR_PLANNING__BASELINE_PLANNERS_HPP_

#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include "excavator_common/planner_interfaces.hpp"

namespace tramcar {
namespace planning {

// Deterministic joint-space baseline. It first tries the straight line and,
// when PlanningRequest::state_validity_check is provided, a finite number of
// single-joint intermediate detour candidates. It is not a complete search
// and is not a replacement for RRT/RRT*/PRM.
class LinearPathPlanner final : public PathPlanner {
 public:
  LinearPathPlanner(std::vector<std::string> joint_names,
                    std::vector<std::pair<double, double>> joint_limits,
                    double max_joint_step = 0.05,
                    std::size_t max_detour_attempts = 12);

  PlanningResponse Plan(const PlanningRequest& request) override;

 private:
  std::vector<std::string> joint_names_;
  std::vector<std::pair<double, double>> joint_limits_;
  double max_joint_step_;
  std::size_t max_detour_attempts_;
};

class SimpleTimeParameterizer final : public TimeParameterizer {
 public:
  TimeParameterizationResponse Parameterize(
      const TimeParameterizationRequest& request) override;
};

class LinearTrajectoryInterpolator final : public TrajectoryInterpolator {
 public:
  ValidationResult Sample(const JointTrajectory& trajectory, double t,
                          JointTrajectoryPoint* point) const override;
};

}  // namespace planning
}  // namespace tramcar

#endif  // EXCAVATOR_PLANNING__BASELINE_PLANNERS_HPP_
