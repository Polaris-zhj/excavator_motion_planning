#ifndef EXCAVATOR_PLANNING__BASELINE_PLANNERS_HPP_
#define EXCAVATOR_PLANNING__BASELINE_PLANNERS_HPP_

#include <string>
#include <utility>
#include <vector>

#include "excavator_common/planner_interfaces.hpp"

namespace tramcar {
namespace planning {

// A deterministic straight-line joint-space baseline. It does not perform
// obstacle avoidance; use it as a replaceable reference implementation.
class LinearPathPlanner final : public PathPlanner {
 public:
  LinearPathPlanner(std::vector<std::string> joint_names,
                    std::vector<std::pair<double, double>> joint_limits,
                    double max_joint_step = 0.05);

  PlanningResponse Plan(const PlanningRequest& request) override;

 private:
  std::vector<std::string> joint_names_;
  std::vector<std::pair<double, double>> joint_limits_;
  double max_joint_step_;
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
