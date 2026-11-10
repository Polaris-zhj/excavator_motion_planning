#ifndef EXCAVATOR_COLLISION__FCL_COLLISION_CHECKER_HPP_
#define EXCAVATOR_COLLISION__FCL_COLLISION_CHECKER_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "excavator_collision/collision_checker_interface.hpp"

namespace excavator {
namespace planning {

struct FclEnvironment;
class FclCollisionModelBuilder;

// YAML collision geometry with local FCL queries. FCL types are hidden behind
// the private adapter boundary.
class FclCollisionChecker final : public CollisionChecker {
 public:
  FclCollisionChecker();
  ~FclCollisionChecker() override;

  FclCollisionChecker(const FclCollisionChecker&) = delete;
  FclCollisionChecker& operator=(const FclCollisionChecker&) = delete;

  ValidationResult Initialize(const CollisionModelConfig& config,
                              const KinematicsModel* kinematics_model) override;

  void SetAllowedCollision(const std::string& link_a, const std::string& link_b,
                           bool allowed) override;

  CollisionCheckResult CheckCollision(
      const JointPositions& positions,
      const std::vector<data::Object>& obstacles,
      const CollisionQueryOptions& options = {}) const override;

  CollisionCheckResult CheckSelfCollision(
      const JointPositions& positions) const override;

  CollisionCheckResult CheckEnvironmentCollision(
      const JointPositions& positions,
      const std::vector<data::Object>& obstacles,
      const CollisionQueryOptions& options = {}) const override;

  CollisionCheckResult CheckPathCollision(
      const JointPath& path, const std::vector<data::Object>& obstacles,
      double max_joint_step,
      const CollisionQueryOptions& options = {}) const override;

  CollisionCheckResult CheckPathCollision(
      const JointPath& path, const data::Object& obstacle,
      double max_joint_step,
      const CollisionQueryOptions& options = {}) const override;

  CollisionCheckResult CheckTrajectory(
      const JointTrajectory& trajectory,
      const std::vector<data::Object>& obstacles, double max_joint_step,
      const CollisionQueryOptions& options = {}) const override;

  CollisionDistanceResult ComputeDistance(
      const JointPositions& positions, const data::Object& obstacle,
      const CollisionQueryOptions& options = {}) const override;

 private:
  ValidationResult BuildEnvironment(const std::vector<data::Object>& obstacles,
                                    const CollisionQueryOptions& options,
                                    FclEnvironment* environment) const;

  CollisionCheckResult CheckPositions(
      const JointPositions& positions,
      const std::vector<data::Object>& obstacles,
      const CollisionQueryOptions& options, bool check_self_collision,
      bool check_environment_collision) const;

  CollisionCheckResult CheckPathCore(const JointPath& path,
                                     const std::vector<data::Object>& obstacles,
                                     double max_joint_step,
                                     const CollisionQueryOptions& options,
                                     bool check_self_collision,
                                     bool check_environment_collision) const;

  ValidationResult ValidateSingleObstacle(const data::Object& obstacle) const;

  // Owns the YAML robot model and the kinematics model, and only resolves
  // joint positions into link collision information.
  std::unique_ptr<FclCollisionModelBuilder> fcl_model_;
  std::vector<std::pair<std::string, std::string>> allowed_pairs_;
  bool initialized_{false};
};

}  // namespace planning
}  // namespace excavator

#endif  // EXCAVATOR_COLLISION__FCL_COLLISION_CHECKER_HPP_
