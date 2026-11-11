#ifndef EXCAVATOR_KINEMATICS__KINEMATICS_SOLVER_HPP_
#define EXCAVATOR_KINEMATICS__KINEMATICS_SOLVER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "excavator_kinematics/kinematics_model.hpp"

namespace tramcar::planning {

class KinematicsSolver {
 public:
  virtual ~KinematicsSolver() = default;

  virtual KinematicsResult Initialize(
      std::shared_ptr<const KinematicsModel> model) = 0;

  // Pose is expressed in the model root_link coordinate frame.
  virtual KinematicsResult GetLinkPose(const JointVector& q,
                                       const std::string& link_name,
                                       Pose* pose) const = 0;

  // poses[i] corresponds to link_names[i].
  virtual KinematicsResult GetLinkPoses(
      const JointVector& q, const std::vector<std::string>& link_names,
      std::vector<Pose>* poses) const = 0;

  // Pose of target_link expressed in reference_link coordinates.
  virtual KinematicsResult GetRelativePose(
      const JointVector& q, const std::string& reference_link,
      const std::string& target_link, Pose* pose) const = 0;

  // 6xN geometric Jacobian expressed in root_link coordinates.
  virtual KinematicsResult GetJacobian(const JointVector& q,
                                       const std::string& link_name,
                                       JacobianMatrix* jacobian) const = 0;

  virtual IkResult InverseKinematics(const IkRequest& request) const = 0;
};

}  // namespace tramcar::planning

#endif  // EXCAVATOR_KINEMATICS__KINEMATICS_SOLVER_HPP_
