#ifndef EXCAVATOR_COLLISION__COLLISION_CHECKER_INTERFACE_HPP_
#define EXCAVATOR_COLLISION__COLLISION_CHECKER_INTERFACE_HPP_

#include <string>
#include <utility>
#include <vector>

#include "excavator_common/joint_trajectory.hpp"
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

// Backend-independent collision model description.
//
// urdf_fcl:
//   model_format = "urdf", model_source = URDF XML string
// dh_fcl (future):
//   model_format = "dh", model_source = path to the DH parameter file
struct CollisionModelConfig {
  std::vector<std::string> joint_names;
  std::string model_source;
  std::string model_format{"urdf"};
};

// Abstract collision checker. Planning and control depend only on this
// interface, so a new collision algorithm can be selected by type without
// changing the planning or controller logic.
class CollisionChecker {
 public:
  virtual ~CollisionChecker() = default;

  virtual ValidationResult Initialize(
      const CollisionModelConfig& config) = 0;

  // Additional link pairs excluded from self-collision checking (on top of
  // direct parent-child pairs). Order of names in a pair does not matter.
  virtual void SetAllowedCollisionPairs(
      const std::vector<std::pair<std::string, std::string>>& pairs) = 0;

  virtual CollisionCheckResult CheckState(
      const JointPositions& positions,
      const excavator_interfaces::msg::PlanningScene& scene) const = 0;

  virtual CollisionCheckResult CheckPath(
      const JointPath& path,
      const excavator_interfaces::msg::PlanningScene& scene,
      double max_joint_step) const = 0;

  virtual CollisionCheckResult CheckTrajectory(
      const JointTrajectory& trajectory,
      const excavator_interfaces::msg::PlanningScene& scene,
      double max_joint_step) const = 0;
};

}  // namespace planning
}  // namespace tramcar

#endif  // EXCAVATOR_COLLISION__COLLISION_CHECKER_INTERFACE_HPP_
