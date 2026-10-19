#ifndef EXCAVATOR_VISUALIZATION__TRAJECTORY_VISUALIZER_HPP_
#define EXCAVATOR_VISUALIZATION__TRAJECTORY_VISUALIZER_HPP_

#include <string>

#include "excavator_common/joint_trajectory.hpp"
#include "kinematics/kinematics_interface.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace tramcar {
namespace planning {
namespace visualization {

// Converts joint trajectories to RViz markers and end-effector paths using
// forward kinematics. Markers use stable namespaces/IDs and begin with a
// DELETEALL entry so replaced trajectories do not leave stale geometry.
class TrajectoryVisualizer {
 public:
  TrajectoryVisualizer(const KinematicsModel& kinematics,
                       std::string frame_id = "base_link",
                       std::string tip_link = "bucket_tip");

  ValidationResult BuildMarkers(
      const JointTrajectory& trajectory,
      visualization_msgs::msg::MarkerArray* markers) const;

  ValidationResult BuildEndEffectorPath(const JointTrajectory& trajectory,
                                        nav_msgs::msg::Path* path) const;

  void SetFrameId(const std::string& frame_id);
  void SetTipLink(const std::string& tip_link);

 private:
  ValidationResult ComputeTipPoses(
      const JointTrajectory& trajectory,
      std::vector<Pose3>* poses) const;

  const KinematicsModel& kinematics_;
  std::string frame_id_;
  std::string tip_link_;
};

}  // namespace visualization
}  // namespace planning
}  // namespace tramcar

#endif  // EXCAVATOR_VISUALIZATION__TRAJECTORY_VISUALIZER_HPP_
