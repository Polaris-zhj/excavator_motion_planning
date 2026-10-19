#ifndef EXCAVATOR_SIM_VIZ__VELOCITY_TRACKING_CONTROLLER_HPP_
#define EXCAVATOR_SIM_VIZ__VELOCITY_TRACKING_CONTROLLER_HPP_

#include <vector>

#include "excavator_sim_viz/controller_interface.hpp"

namespace tramcar {
namespace planning {

struct VelocityControllerConfig {
  JointPositions max_velocity;
  JointPositions max_acceleration;
  // Hard position limits are used to stop motion that would leave the
  // workspace. Soft limits additionally suppress outward motion near them.
  JointPositions lower_position_limits;
  JointPositions upper_position_limits;
  double position_gain{4.0};
  double soft_limit_margin{0.02};
  bool enforce_soft_limits{true};
};

// Feed-forward reference velocity plus proportional position feedback. The
// output is limited by joint velocity, joint acceleration and soft position
// limits. Every modification is reported through ControllerOutput::limited.
class VelocityTrackingController final : public ControllerInterface {
 public:
  explicit VelocityTrackingController(VelocityControllerConfig config);

  ControllerOutput ComputeCommand(const RobotState& current,
                                  const JointTrajectoryPoint& reference,
                                  double dt,
                                  ControlCommand* command) override;

  void Reset() override;

 private:
  VelocityControllerConfig config_;
};

}  // namespace planning
}  // namespace tramcar

#endif  // EXCAVATOR_SIM_VIZ__VELOCITY_TRACKING_CONTROLLER_HPP_
