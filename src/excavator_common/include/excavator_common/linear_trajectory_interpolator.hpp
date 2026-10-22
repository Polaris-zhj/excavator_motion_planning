#ifndef EXCAVATOR_COMMON__LINEAR_TRAJECTORY_INTERPOLATOR_HPP_
#define EXCAVATOR_COMMON__LINEAR_TRAJECTORY_INTERPOLATOR_HPP_

#include "excavator_common/planner_interfaces.hpp"

namespace tramcar {
namespace planning {

// Piecewise-linear interpolation with linear velocity/acceleration blending.
// This is a shared execution primitive: the controller, simulator and planning
// pipeline all need to sample the same trajectory representation.
class LinearTrajectoryInterpolator final : public TrajectoryInterpolator {
 public:
  ValidationResult Sample(const JointTrajectory& trajectory, double t,
                          JointTrajectoryPoint* point) const override;
};

}  // namespace planning
}  // namespace tramcar

#endif  // EXCAVATOR_COMMON__LINEAR_TRAJECTORY_INTERPOLATOR_HPP_
