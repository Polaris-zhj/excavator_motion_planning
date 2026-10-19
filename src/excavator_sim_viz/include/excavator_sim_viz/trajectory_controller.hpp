#ifndef EXCAVATOR_SIM_VIZ__TRAJECTORY_CONTROLLER_HPP_
#define EXCAVATOR_SIM_VIZ__TRAJECTORY_CONTROLLER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "excavator_common/planner_interfaces.hpp"
#include "excavator_common/robot_state.hpp"
#include "excavator_sim_viz/controller_interface.hpp"

namespace tramcar {
namespace planning {

struct ExecutionStatusSnapshot {
  ExecutionState state{ExecutionState::kIdle};
  std::string message{"idle"};
  double elapsed_time{0.0};
  double duration{0.0};
  double progress{0.0};
  double tracking_error_norm{0.0};
};

struct TrajectoryControllerConfig {
  std::vector<std::string> joint_names;
  JointPositions max_velocity;
  JointPositions max_acceleration;
  JointPositions lower_position_limits;
  JointPositions upper_position_limits;
  // Abort with SAFETY_STOP when the position error norm exceeds this value.
  double max_tracking_error{0.5};
  // Finish when the trajectory time is reached and this error is below the
  // tolerance; otherwise keep tracking until the timeout margin elapses.
  double completion_position_tolerance{0.02};
  double execution_timeout_margin{3.0};
  double start_continuity_tolerance{0.05};
  // Online replacement must be continuous with both the measured state and the
  // currently executing reference.
  double update_position_continuity_tolerance{0.05};
  double update_velocity_continuity_tolerance{1.0};
};

struct TrajectoryStepResult {
  bool ok{false};
  bool has_command{false};
  ControlCommand command;
  ExecutionStatusSnapshot status;
  std::string message;
};

// Owns the candidate/active trajectory, execution state and control-loop time
// progression. The class performs no ROS I/O and no planning; callers provide
// the measured state and apply the returned command through a simulator or
// hardware adapter.
class TrajectoryController final {
 public:
  TrajectoryController(TrajectoryControllerConfig config,
                       std::unique_ptr<ControllerInterface> controller,
                       std::unique_ptr<TrajectoryInterpolator> interpolator =
                           nullptr);

  // Validates a complete trajectory and stores it as the candidate. Never
  // starts execution; a running active trajectory is not disturbed.
  ValidationResult SubmitTrajectory(const JointTrajectory& trajectory,
                                    const RobotState& current);

  // Validates an online replacement candidate. It is applied at the next
  // control-cycle boundary if it is continuous with the measured state.
  ValidationResult UpdateTrajectory(const JointTrajectory& trajectory,
                                    const RobotState& current);

  // Atomic execution entry point: validates against the measured state and
  // starts execution only when every check passes.
  ValidationResult ExecuteTrajectory(const JointTrajectory& trajectory,
                                     const RobotState& current);

  ValidationResult Pause();
  ValidationResult Resume();
  ValidationResult Cancel();
  ValidationResult Stop();

  // Advances execution by dt and computes the next command for the measured
  // state. Call once per fixed control cycle.
  TrajectoryStepResult Update(const RobotState& current, double dt);

  ExecutionStatusSnapshot GetStatus() const;
  bool HasActiveTrajectory() const;

 private:
  ValidationResult ValidateTrajectory(const JointTrajectory& trajectory,
                                      const RobotState& current,
                                      bool require_start_continuity) const;
  double Duration() const;
  bool ApplyPendingUpdate(const RobotState& current, std::string* message);
  void ZeroCommand(TrajectoryStepResult* result) const;
  void SetState(ExecutionState state, const std::string& message);

  mutable std::mutex mutex_;
  TrajectoryControllerConfig config_;
  std::unique_ptr<ControllerInterface> controller_;
  std::unique_ptr<TrajectoryInterpolator> interpolator_;
  JointTrajectory candidate_;
  JointTrajectory active_;
  bool has_candidate_{false};
  bool has_active_{false};
  bool pending_update_{false};
  double elapsed_time_{0.0};
  ExecutionStatusSnapshot status_;
};

}  // namespace planning
}  // namespace tramcar

#endif  // EXCAVATOR_SIM_VIZ__TRAJECTORY_CONTROLLER_HPP_
