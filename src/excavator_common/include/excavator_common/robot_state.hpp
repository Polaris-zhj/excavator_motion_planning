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

// Execution states shared by the controller, ROS adapters and visualization.
// Values must stay aligned with excavator_interfaces/msg/ExecutionStatus.msg.
enum class ExecutionState {
  kIdle = 0,
  kValidating = 1,
  kReady = 2,
  kRunning = 3,
  kPaused = 4,
  kCompleted = 5,
  kCancelled = 6,
  kSafetyStop = 7,
  kError = 8
};

inline const char* ToString(ExecutionState state) {
  switch (state) {
    case ExecutionState::kIdle:
      return "IDLE";
    case ExecutionState::kValidating:
      return "VALIDATING";
    case ExecutionState::kReady:
      return "READY";
    case ExecutionState::kRunning:
      return "RUNNING";
    case ExecutionState::kPaused:
      return "PAUSED";
    case ExecutionState::kCompleted:
      return "COMPLETED";
    case ExecutionState::kCancelled:
      return "CANCELLED";
    case ExecutionState::kSafetyStop:
      return "SAFETY_STOP";
    case ExecutionState::kError:
      return "ERROR";
  }
  return "UNKNOWN";
}

}  // namespace planning
}  // namespace tramcar

#endif  // EXCAVATOR_COMMON__ROBOT_STATE_HPP_
