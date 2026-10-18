#ifndef EXCAVATOR_COMMON__ROBOT_STATE_HPP_
#define EXCAVATOR_COMMON__ROBOT_STATE_HPP_

#include "excavator_common/joint_trajectory.hpp"

namespace tramcar {
namespace planning {

struct RobotState {
  double t{0.0};
  JointPositions positions;
  JointPositions velocities;
  JointPositions accelerations;
};

enum class ExecutionState {
  kIdle,
  kValidating,
  kRunning,
  kSucceeded,
  kCanceled,
  kRejected,
  kFailed
};

}  // namespace planning
}  // namespace tramcar

#endif  // EXCAVATOR_COMMON__ROBOT_STATE_HPP_
