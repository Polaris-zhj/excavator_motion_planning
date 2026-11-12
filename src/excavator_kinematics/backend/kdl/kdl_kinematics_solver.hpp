#ifndef EXCAVATOR_KINEMATICS__KDL_KINEMATICS_SOLVER_HPP_
#define EXCAVATOR_KINEMATICS__KDL_KINEMATICS_SOLVER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "excavator_kinematics/backend/kdl/kdl_model_converter.hpp"
#include "excavator_kinematics/backend/kdl/kdl_solver_bundle.hpp"
#include "excavator_kinematics/kinematics_solver.hpp"

namespace excavator {
namespace planning {

class KdlKinematicsSolver final : public KinematicsSolver {
 public:
  explicit KdlKinematicsSolver(KinematicsSolverConfig config = {});
  ~KdlKinematicsSolver() override = default;
  KinematicsResult Initialize(
      std::shared_ptr<const KinematicsModel> model) override;
  KinematicsResult GetLinkPose(const JointVector& q,
                               const std::string& link_name,
                               Pose* pose) const override;
  KinematicsResult GetLinkPoses(const JointVector& q,
                                const std::vector<std::string>& link_names,
                                std::vector<Pose>* poses) const override;
  KinematicsResult GetRelativePose(const JointVector& q,
                                   const std::string& reference_link,
                                   const std::string& target_link,
                                   Pose* pose) const override;
  KinematicsResult GetJacobian(const JointVector& q,
                               const std::string& link_name,
                               JacobianMatrix* jacobian) const override;
  IkResult InverseKinematics(const IkRequest& request) const override;

 private:
  KinematicsResult ValidateJointVector(const JointVector& q) const;
  KinematicsResult GetOrCreateBundleLocked(const std::string& base_link,
                                           const std::string& tip_link,
                                           KdlSolverBundle** bundle) const;
  KinematicsResult GetLinkPoseLocked(const JointVector& q,
                                     const std::string& link_name,
                                     Pose* pose) const;
  std::string MakeCacheKey(const std::string& base_link,
                           const std::string& tip_link) const;

  KinematicsSolverConfig config_;
  std::shared_ptr<const KinematicsModel> model_;
  KdlModelConverter model_converter_;
  mutable std::mutex mutex_;
  mutable std::unordered_map<std::string, std::unique_ptr<KdlSolverBundle>>
      bundles_;
};

}  // namespace planning
}  // namespace excavator

#endif  // EXCAVATOR_KINEMATICS__KDL_KINEMATICS_SOLVER_HPP_
