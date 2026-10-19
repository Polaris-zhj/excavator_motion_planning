#ifndef EXCAVATOR_VISUALIZATION__TRAJECTORY_OCCUPANCY_VISUALIZER_HPP_
#define EXCAVATOR_VISUALIZATION__TRAJECTORY_OCCUPANCY_VISUALIZER_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include "excavator_common/joint_trajectory.hpp"
#include "kinematics/kinematics_interface.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace tramcar {
namespace planning {
namespace visualization {

// Visual geometry of one URDF link, expressed in the link frame. Multiple
// sampled trajectory states are rendered as semi-transparent mesh markers.
struct LinkVisual {
  std::string link_name;
  std::string mesh_resource;
  Vector3 scale{1.0, 1.0, 1.0};
  Pose3 origin;
  float red{0.8F};
  float green{0.8F};
  float blue{0.9F};
  float alpha{0.25F};
};

// Multi-pose occupancy preview built from the actual URDF visual meshes. It is
// an approximate visualization of sampled states only: markers do not form a
// continuous swept volume and must never be used as a collision decision.
class TrajectoryOccupancyVisualizer {
 public:
  TrajectoryOccupancyVisualizer(const KinematicsModel& kinematics,
                                std::vector<LinkVisual> link_visuals,
                                std::string frame_id = "base_link",
                                std::size_t max_samples = 8);

  ValidationResult Build(const JointTrajectory& trajectory,
                         visualization_msgs::msg::MarkerArray* markers) const;

 private:
  const KinematicsModel& kinematics_;
  std::vector<LinkVisual> link_visuals_;
  std::string frame_id_;
  std::size_t max_samples_;
};

}  // namespace visualization
}  // namespace planning
}  // namespace tramcar

#endif  // EXCAVATOR_VISUALIZATION__TRAJECTORY_OCCUPANCY_VISUALIZER_HPP_
