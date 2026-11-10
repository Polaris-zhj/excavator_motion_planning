#ifndef EXCAVATOR_COLLISION__OBSTACLE_ADAPTER_HPP_
#define EXCAVATOR_COLLISION__OBSTACLE_ADAPTER_HPP_

#include <string>
#include <vector>

#include "data/object/object.h"
#include "excavator_interfaces/msg/collision_object.hpp"
#include "excavator_interfaces/msg/planning_scene.hpp"
#include "kinematics/kinematics_interface.hpp"

namespace excavator {
namespace planning {

// Bridges message or legacy obstacle representations into the project-owned
// data::Object model used by collision backends. Primitive obstacles are
// converted to an enclosing axis-aligned bounding box, so orientation is
// conservatively expanded instead of being dropped.
class ObstacleAdapter {
 public:
  ObstacleAdapter() = delete;

  static ValidationResult Convert(
      const excavator_interfaces::msg::CollisionObject& source,
      data::Object* object);

  static ValidationResult Convert(
      const std::vector<excavator_interfaces::msg::CollisionObject>& sources,
      std::vector<data::Object>* objects);

  static ValidationResult Convert(
      const excavator_interfaces::msg::PlanningScene& scene,
      std::vector<data::Object>* objects);

  // Stable integer id assigned to a message obstacle id so collision results
  // and converted obstacles can be correlated. Numeric ids are preserved;
  // other ids are folded with FNV-1a.
  static int ObjectId(const std::string& id);
};

}  // namespace planning
}  // namespace excavator

#endif  // EXCAVATOR_COLLISION__OBSTACLE_ADAPTER_HPP_
