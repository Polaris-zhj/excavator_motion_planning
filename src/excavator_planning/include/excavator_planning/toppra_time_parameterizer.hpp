#ifndef EXCAVATOR_PLANNING__TOPPRA_TIME_PARAMETERIZER_HPP_
#define EXCAVATOR_PLANNING__TOPPRA_TIME_PARAMETERIZER_HPP_

#include <cstddef>

#include "excavator_common/planner_interfaces.hpp"

namespace tramcar {
namespace planning {

// Time-optimal rest-to-rest parameterization backed by TOPP-RA.
//
// The geometric JointPath is interpolated with a clamped cubic spline in
// joint space (chord-length path parameter), then TOPP-RA computes the
// fastest path timing that satisfies the velocity and acceleration limits.
class ToppraTimeParameterizer final : public TimeParameterizer {
 public:
  // grid_points is the number of discretization intervals N; TOPP-RA produces
  // N + 1 trajectory samples.
  explicit ToppraTimeParameterizer(std::size_t grid_points = 200);

  TimeParameterizationResponse Parameterize(
      const TimeParameterizationRequest& request) override;

 private:
  std::size_t grid_points_;
};

}  // namespace planning
}  // namespace tramcar

#endif  // EXCAVATOR_PLANNING__TOPPRA_TIME_PARAMETERIZER_HPP_
