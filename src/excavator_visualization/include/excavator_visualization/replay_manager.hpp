#ifndef EXCAVATOR_VISUALIZATION__REPLAY_MANAGER_HPP_
#define EXCAVATOR_VISUALIZATION__REPLAY_MANAGER_HPP_

#include <string>
#include <vector>

#include "excavator_common/joint_trajectory.hpp"
#include "excavator_common/robot_state.hpp"

namespace tramcar {
namespace planning {
namespace visualization {

// CSV persistence for offline replay. Files are plain text:
//   joint_names,<name1>,<name2>,...
//   t,<positions...>,<velocities...>,<accelerations...>
// The joint_names line is mandatory; each data row must have 1 + 3 * N
// columns. State logs use the same layout with velocity/acceleration columns.
class ReplayManager {
 public:
  bool SaveTrajectory(const std::string& path,
                      const JointTrajectory& trajectory,
                      std::string* error) const;
  bool LoadTrajectory(const std::string& path, JointTrajectory* trajectory,
                      std::string* error) const;

  bool SaveStates(const std::string& path,
                  const std::vector<std::string>& joint_names,
                  const std::vector<RobotState>& states,
                  std::string* error) const;
  bool LoadStates(const std::string& path,
                  std::vector<std::string>* joint_names,
                  std::vector<RobotState>* states, std::string* error) const;
};

}  // namespace visualization
}  // namespace planning
}  // namespace tramcar

#endif  // EXCAVATOR_VISUALIZATION__REPLAY_MANAGER_HPP_
