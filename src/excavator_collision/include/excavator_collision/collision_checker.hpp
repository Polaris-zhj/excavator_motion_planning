#ifndef EXCAVATOR_COLLISION__COLLISION_CHECKER_HPP_
#define EXCAVATOR_COLLISION__COLLISION_CHECKER_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "excavator_collision/collision_checker_interface.hpp"
#include "excavator_common/joint_trajectory.hpp"

namespace tramcar {
namespace planning {

// URDF collision geometry checker backed by FCL. Unsupported or malformed
// geometry fails closed: it is never silently omitted from collision checks.
class UrdfFclCollisionChecker final : public CollisionChecker {
 public:
  UrdfFclCollisionChecker();
  ~UrdfFclCollisionChecker() override;

  UrdfFclCollisionChecker(const UrdfFclCollisionChecker&) = delete;
  UrdfFclCollisionChecker& operator=(const UrdfFclCollisionChecker&) = delete;

  // Existing convenience overload, kept for tests and direct users.
  ValidationResult Initialize(const std::string& urdf_xml,
                              std::vector<std::string> joint_names);

  ValidationResult Initialize(
      const CollisionModelConfig& config) override;

  // Additional link pairs excluded from self-collision checking (on top of
  // direct parent-child pairs). Order of names in a pair does not matter.
  void SetAllowedCollisionPairs(
      const std::vector<std::pair<std::string, std::string>>& pairs) override;

  CollisionCheckResult CheckState(
      const JointPositions& positions,
      const excavator_interfaces::msg::PlanningScene& scene) const override;

  CollisionCheckResult CheckPath(
      const JointPath& path,
      const excavator_interfaces::msg::PlanningScene& scene,
      double max_joint_step) const override;

  CollisionCheckResult CheckTrajectory(
      const JointTrajectory& trajectory,
      const excavator_interfaces::msg::PlanningScene& scene,
      double max_joint_step) const override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace planning
}  // namespace tramcar

#endif  // EXCAVATOR_COLLISION__COLLISION_CHECKER_HPP_
