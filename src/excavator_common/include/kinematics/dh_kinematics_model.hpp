#ifndef KINEMATICS__DH_KINEMATICS_MODEL_HPP_
#define KINEMATICS__DH_KINEMATICS_MODEL_HPP_

#include <string>
#include <vector>

#include "kinematics/dh_parameters.hpp"
#include "kinematics/kinematics_interface.hpp"

namespace excavator {
namespace planning {

// Modified-DH kinematics backend. It does not parse URDF; it is configured by
// a DH YAML file and reports link/tool poses in base_link.
class DhKinematicsModel final : public KinematicsModel {
 public:
  explicit DhKinematicsModel(std::vector<std::string> joint_names = {
      "swing_joint", "boom_joint", "stick_joint", "bucket_joint"});
  ~DhKinematicsModel() override;

  DhKinematicsModel(const DhKinematicsModel&) = delete;
  DhKinematicsModel& operator=(const DhKinematicsModel&) = delete;

  ValidationResult InitializeFromFile(const std::string& path);
  ValidationResult InitializeFromString(const std::string& yaml);

  const std::vector<std::string>& joint_names() const override;
  std::vector<std::pair<double, double>> GetPositionLimits() const override;
  std::vector<double> GetVelocityLimits() const override;

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
  int LinkJointCount(const std::string& link_name) const;
  ValidationResult BuildFrames(
      const JointPositions& positions,
      std::vector<Eigen::Isometry3d>* frames) const;
  ValidationResult LinkPose(const JointPositions& positions,
                            const std::string& link_name,
                            Eigen::Isometry3d* transform) const;

  DhParameters parameters_;
  std::vector<std::string> joint_names_;
  bool initialized_{false};
};

}  // namespace planning
}  // namespace excavator

#endif  // KINEMATICS__DH_KINEMATICS_MODEL_HPP_
