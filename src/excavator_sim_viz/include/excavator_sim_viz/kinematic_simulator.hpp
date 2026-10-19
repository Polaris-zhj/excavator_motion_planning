#ifndef EXCAVATOR_SIM_VIZ__KINEMATIC_SIMULATOR_HPP_
#define EXCAVATOR_SIM_VIZ__KINEMATIC_SIMULATOR_HPP_

#include <string>
#include <vector>

#include "excavator_common/joint_trajectory.hpp"
#include "excavator_common/robot_state.hpp"
#include "excavator_planning/baseline_planners.hpp"
#include "excavator_sim_viz/controller_interface.hpp"

namespace tramcar {
namespace planning {

struct SimulationState {
  double elapsed_time{0.0};
  std::vector<std::string> joint_names;
  JointPositions positions;
  JointPositions velocities;
  bool trajectory_active{false};
  bool trajectory_finished{false};
  bool safety_fault{false};
};

struct SimulatorLimits {
  // Empty vectors disable the corresponding limit.
  JointPositions max_velocity;
  JointPositions max_acceleration;
  JointPositions lower_position_limits;
  JointPositions upper_position_limits;
};

class SimulatorInterface {
 public:
  virtual ~SimulatorInterface() = default;
  virtual ValidationResult SubmitTrajectory(
      const JointTrajectory& trajectory) = 0;
  virtual ValidationResult Step(double dt, SimulationState* state) = 0;
  virtual ValidationResult ApplyCommand(const ControlCommand& command,
                                        double dt, SimulationState* state) = 0;
};

// Purely kinematic simulator. It supports two mutually exclusive modes:
//  - direct trajectory stepping (SubmitTrajectory + Step), kept for simple
//    playback and tests;
//  - closed-loop command stepping (ApplyCommand), where velocities are
//    integrated and checked against configured limits.
// It never models hydraulic pressure, load, gravity or soil contact.
class KinematicSimulator final : public SimulatorInterface {
 public:
  KinematicSimulator(std::vector<std::string> joint_names,
                     JointPositions initial_positions,
                     SimulatorLimits limits = {});

  ValidationResult SubmitTrajectory(
      const JointTrajectory& trajectory) override;
  ValidationResult Step(double dt, SimulationState* state) override;
  ValidationResult ApplyCommand(const ControlCommand& command, double dt,
                                SimulationState* state) override;

  void CancelTrajectory();

  // Resets the simulated state and clears both the active trajectory and any
  // safety fault.
  ValidationResult SetState(const RobotState& state);

  const JointPositions& positions() const;
  const JointPositions& velocities() const;
  bool HasSafetyFault() const;

 private:
  ValidationResult CheckCommandLimits(const ControlCommand& command, double dt,
                                      ControlCommand* limited_command,
                                      bool* limited) const;

  std::vector<std::string> joint_names_;
  JointPositions initial_positions_;
  JointPositions current_positions_;
  JointPositions current_velocities_;
  SimulatorLimits limits_;
  JointTrajectory trajectory_;
  LinearTrajectoryInterpolator interpolator_;
  double elapsed_time_{0.0};
  bool has_trajectory_{false};
  bool trajectory_finished_{false};
  bool safety_fault_{false};
};

}  // namespace planning
}  // namespace tramcar

#endif  // EXCAVATOR_SIM_VIZ__KINEMATIC_SIMULATOR_HPP_
