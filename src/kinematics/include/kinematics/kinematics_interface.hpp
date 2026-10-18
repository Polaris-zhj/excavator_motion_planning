#ifndef KINEMATICS__KINEMATICS_INTERFACE_HPP_
#define KINEMATICS__KINEMATICS_INTERFACE_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include "excavator_common/joint_trajectory.hpp"

namespace tramcar {
namespace planning {

struct Vector3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Quaternion {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
};

struct Pose3 {
  Vector3 position;
  Quaternion orientation;
};

struct Jacobian {
  // Six rows: linear velocity x/y/z, then angular velocity x/y/z.
  // Each row has one value per joint in the supplied joint ordering.
  std::vector<std::vector<double>> rows;
};

struct InverseKinematicsOptions {
  std::size_t max_iterations{100};
  double position_tolerance{1.0e-4};
  double orientation_tolerance{1.0e-4};
  double damping{1.0e-3};
};

struct InverseKinematicsResult {
  bool success{false};
  JointPositions joint_positions;
  std::size_t iterations{0};
  std::string message;
};

class KinematicsModel {
 public:
  virtual ~KinematicsModel() = default;

  // Stateless forward kinematics: joint positions are inputs, not mutable
  // model state. link_name is a frame name from the loaded robot description.
  virtual ValidationResult GetLinkPose(const JointPositions& joint_positions,
                                       const std::string& link_name,
                                       Pose3* pose) const = 0;

  virtual ValidationResult GetJacobian(const JointPositions& joint_positions,
                                       const std::string& link_name,
                                       Jacobian* jacobian) const = 0;

  virtual InverseKinematicsResult SolveIK(
      const Pose3& target_pose, const std::string& link_name,
      const JointPositions& seed,
      const InverseKinematicsOptions& options) const = 0;
};

}  // namespace planning
}  // namespace tramcar

#endif  // KINEMATICS__KINEMATICS_INTERFACE_HPP_
