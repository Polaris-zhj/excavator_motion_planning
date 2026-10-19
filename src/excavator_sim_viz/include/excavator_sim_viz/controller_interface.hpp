#ifndef EXCAVATOR_SIM_VIZ__CONTROLLER_INTERFACE_HPP_
#define EXCAVATOR_SIM_VIZ__CONTROLLER_INTERFACE_HPP_

#include <string>

#include "excavator_common/joint_trajectory.hpp"
#include "excavator_common/robot_state.hpp"

namespace tramcar {
namespace planning {

// Control output produced by a ControllerInterface implementation. Velocities
// are joint velocities in the RobotModel joint ordering (rad/s or m/s).
struct ControlCommand {
  JointPositions positions;
  JointPositions velocities;
  bool has_positions{false};
};

struct ControllerOutput {
  bool ok{false};
  bool limited{false};
  double tracking_error_norm{0.0};
  std::string message;
};

// Computes a bounded joint command from the measured state and one reference
// trajectory point. Implementations must not advance the simulated/real state,
// cache trajectories or publish ROS data.
class ControllerInterface {
 public:
  virtual ~ControllerInterface() = default;

  virtual ControllerOutput ComputeCommand(
      const RobotState& current, const JointTrajectoryPoint& reference,
      double dt, ControlCommand* command) = 0;

  // Called when a new trajectory starts so that any internal state is cleared.
  virtual void Reset() = 0;
};

}  // namespace planning
}  // namespace tramcar

#endif  // EXCAVATOR_SIM_VIZ__CONTROLLER_INTERFACE_HPP_
