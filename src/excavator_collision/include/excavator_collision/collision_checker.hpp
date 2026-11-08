#ifndef EXCAVATOR_COLLISION__COLLISION_CHECKER_HPP_
#define EXCAVATOR_COLLISION__COLLISION_CHECKER_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "excavator_collision/collision_checker_interface.hpp"

namespace tramcar {
namespace planning {

// YAML collision geometry with local FCL queries. FCL types are hidden behind
// the private adapter boundary. FK is supplied by the selected KinematicsModel
// (normally DhKinematicsModel); this backend never parses URDF.
class FclCollisionChecker final : public CollisionChecker {
 public:
  FclCollisionChecker();
  ~FclCollisionChecker() override;

  FclCollisionChecker(const FclCollisionChecker&) = delete;
  FclCollisionChecker& operator=(const FclCollisionChecker&) = delete;

  ValidationResult Initialize(
      const CollisionModelConfig& config,
      const KinematicsModel* kinematics_model) override;

  void SetAllowedCollisionPairs(
      const std::vector<std::pair<std::string, std::string>>& pairs) override;

  CollisionCheckResult CheckState(
      const JointPositions& positions,
      const excavator_interfaces::msg::PlanningScene& scene) const override;

  CollisionCheckResult CheckSelfCollision(
      const JointPositions& positions) const override;

  CollisionCheckResult CheckEnvironmentCollision(
      const JointPositions& positions,
      const excavator_interfaces::msg::PlanningScene& scene) const override;

  CollisionCheckResult CheckPath(
      const JointPath& path,
      const excavator_interfaces::msg::PlanningScene& scene,
      double max_joint_step) const override;

  CollisionCheckResult CheckPathCollisionWithObstacle(
      const JointPath& path,
      const excavator_interfaces::msg::PlanningScene& scene,
      const std::string& obstacle_id, double max_joint_step) const override;

  CollisionCheckResult CheckTrajectory(
      const JointTrajectory& trajectory,
      const excavator_interfaces::msg::PlanningScene& scene,
      double max_joint_step) const override;

  CollisionDistanceResult ComputeDistanceToObstacle(
      const JointPositions& positions,
      const excavator_interfaces::msg::PlanningScene& scene,
      const std::string& obstacle_id) const override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace planning
}  // namespace tramcar

#endif  // EXCAVATOR_COLLISION__COLLISION_CHECKER_HPP_
