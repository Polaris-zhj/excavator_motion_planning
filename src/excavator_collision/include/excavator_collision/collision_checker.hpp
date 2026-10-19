#ifndef EXCAVATOR_COLLISION__COLLISION_CHECKER_HPP_
#define EXCAVATOR_COLLISION__COLLISION_CHECKER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "excavator_common/joint_trajectory.hpp"
#include "excavator_interfaces/msg/collision_object.hpp"
#include "excavator_interfaces/msg/planning_scene.hpp"

namespace tramcar {
namespace planning {

struct CollisionCheckResult {
  bool valid{false};
  bool collision{false};
  std::string message;
  std::string first_link;
  std::string second_object;
  // Joint state at which the first collision was detected. Empty when no
  // collision was found. Allows RViz collision highlight markers.
  JointPositions collision_positions;
  double checked_states{0.0};
};

// URDF collision geometry checker backed by FCL. Unsupported or malformed
// geometry fails closed: it is never silently omitted from collision checks.
class UrdfFclCollisionChecker final {
 public:
  UrdfFclCollisionChecker();
  ~UrdfFclCollisionChecker();

  UrdfFclCollisionChecker(const UrdfFclCollisionChecker&) = delete;
  UrdfFclCollisionChecker& operator=(const UrdfFclCollisionChecker&) = delete;

  ValidationResult Initialize(const std::string& urdf_xml,
                             std::vector<std::string> joint_names);

  // Additional link pairs excluded from self-collision checking (on top of
  // direct parent-child pairs). Order of names in a pair does not matter.
  void SetAllowedCollisionPairs(
      const std::vector<std::pair<std::string, std::string>>& pairs);

  CollisionCheckResult CheckState(
      const JointPositions& positions,
      const excavator_interfaces::msg::PlanningScene& scene) const;

  CollisionCheckResult CheckPath(
      const JointPath& path,
      const excavator_interfaces::msg::PlanningScene& scene,
      double max_joint_step) const;

  CollisionCheckResult CheckTrajectory(
      const JointTrajectory& trajectory,
      const excavator_interfaces::msg::PlanningScene& scene,
      double max_joint_step) const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace planning
}  // namespace tramcar

#endif  // EXCAVATOR_COLLISION__COLLISION_CHECKER_HPP_
