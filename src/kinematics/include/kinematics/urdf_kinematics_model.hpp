#ifndef KINEMATICS__URDF_KINEMATICS_MODEL_HPP_
#define KINEMATICS__URDF_KINEMATICS_MODEL_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "kinematics/kinematics_interface.hpp"

namespace tramcar {
namespace planning {

// URDF-backed, stateless kinematics for a configured ordered joint group.
// The model owns no mutable "current joint state"; every query receives q.
class UrdfKinematicsModel final : public KinematicsModel {
 public:
  explicit UrdfKinematicsModel(std::vector<std::string> joint_names = {
      "swing_joint", "boom_joint", "stick_joint", "bucket_joint"});
  ~UrdfKinematicsModel() override;

  UrdfKinematicsModel(const UrdfKinematicsModel&) = delete;
  UrdfKinematicsModel& operator=(const UrdfKinematicsModel&) = delete;

  ValidationResult InitializeFromFile(const std::string& urdf_path);
  ValidationResult InitializeFromString(const std::string& urdf_xml);
  std::vector<std::pair<double, double>> GetPositionLimits() const;
  std::vector<double> GetVelocityLimits() const;
  const std::vector<std::string>& joint_names() const;

  ValidationResult GetLinkPose(const JointPositions& joint_positions,
                               const std::string& link_name,
                               Pose3* pose) const override;
  ValidationResult GetJacobian(const JointPositions& joint_positions,
                               const std::string& link_name,
                               Jacobian* jacobian) const override;
  InverseKinematicsResult SolveIK(
      const Pose3& target_pose, const std::string& link_name,
      const JointPositions& seed,
      const InverseKinematicsOptions& options) const override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  std::vector<std::string> joint_names_;
};

}  // namespace planning
}  // namespace tramcar

#endif  // KINEMATICS__URDF_KINEMATICS_MODEL_HPP_
