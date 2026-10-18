#ifndef EXCAVATOR_SIM_VIZ__KINEMATIC_SIMULATOR_HPP_
#define EXCAVATOR_SIM_VIZ__KINEMATIC_SIMULATOR_HPP_

#include <string>
#include <vector>

#include "excavator_common/joint_trajectory.hpp"
#include "excavator_planning/baseline_planners.hpp"

namespace excavator_sim_viz {

struct SimulationState {
  double elapsed_time{0.0};
  std::vector<std::string> joint_names;
  tramcar::planning::JointPositions positions;
  tramcar::planning::JointPositions velocities;
  bool trajectory_active{false};
  bool trajectory_finished{false};
};

class SimulatorInterface {
 public:
  virtual ~SimulatorInterface() = default;
  virtual tramcar::planning::ValidationResult SubmitTrajectory(
      const tramcar::planning::JointTrajectory& trajectory) = 0;
  virtual tramcar::planning::ValidationResult Step(
      double dt, SimulationState* state) = 0;
};

class KinematicSimulator final : public SimulatorInterface {
 public:
  KinematicSimulator(std::vector<std::string> joint_names,
                     tramcar::planning::JointPositions initial_positions);

  tramcar::planning::ValidationResult SubmitTrajectory(
      const tramcar::planning::JointTrajectory& trajectory) override;
  tramcar::planning::ValidationResult Step(
      double dt, SimulationState* state) override;
  void CancelTrajectory();

 private:
  std::vector<std::string> joint_names_;
  tramcar::planning::JointPositions initial_positions_;
  tramcar::planning::JointTrajectory trajectory_;
  tramcar::planning::LinearTrajectoryInterpolator interpolator_;
  double elapsed_time_{0.0};
  bool has_trajectory_{false};
  bool trajectory_finished_{false};
};

}  // namespace excavator_sim_viz

#endif  // EXCAVATOR_SIM_VIZ__KINEMATIC_SIMULATOR_HPP_
