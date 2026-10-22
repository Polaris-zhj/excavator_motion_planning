#ifndef KINEMATICS__DH_PARAMETERS_HPP_
#define KINEMATICS__DH_PARAMETERS_HPP_

#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>

#include "excavator_common/joint_trajectory.hpp"

namespace tramcar {
namespace planning {

struct DhJoint {
  std::string name;
  std::string link;
  // Rotation about the DH Z axis is theta_sign * q + theta_offset.
  // Use -1 when the physical joint axis is opposite to the DH Z axis.
  double theta_sign{1.0};
  double alpha_prev{0.0};
  double a_prev{0.0};
  double d{0.0};
  double theta_offset{0.0};
  double lower{0.0};
  double upper{0.0};
  double velocity{1.0};
};

// Modified DH parameters (Craig):
//   A_i = Rx(alpha_prev) Tx(a_prev) Rz(theta + theta_offset) Tz(d)
//
// `link_offsets` maps a physical URDF link frame to the fixed transform from
// the corresponding post-joint DH frame. This lets the DH model report the
// same link/tool poses as the URDF model while planning and IK use DH.
struct DhParameters {
  Eigen::Isometry3d base{Eigen::Isometry3d::Identity()};
  std::vector<DhJoint> joints;
  std::unordered_map<std::string, Eigen::Isometry3d> link_offsets;

  ValidationResult LoadFromFile(const std::string& path);
  ValidationResult LoadFromString(const std::string& yaml);

  std::vector<std::string> JointNames() const;
  bool IsLoaded() const { return !joints.empty(); }
};

}  // namespace planning
}  // namespace tramcar

#endif  // KINEMATICS__DH_PARAMETERS_HPP_
