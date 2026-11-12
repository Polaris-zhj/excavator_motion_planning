#ifndef EXCAVATOR_KINEMATICS__URDF_MODEL_CONVERTER_HPP_
#define EXCAVATOR_KINEMATICS__URDF_MODEL_CONVERTER_HPP_

#include <string>
#include <vector>

#include "excavator_kinematics/kinematics_types.hpp"

namespace excavator {
namespace planning {

class UrdfModelConverter {
 public:
  // If active_joint_order is empty, all movable URDF joints are selected in
  // tree traversal order. Otherwise only the named joints are independent DOF.
  // Other movable joints remain in the topology and are locked at q = 0.
  static KinematicsResult FromFile(
      const std::string& path, ModelDescription* description,
      const std::vector<std::string>& active_joint_order = {});
  static KinematicsResult FromString(
      const std::string& xml, ModelDescription* description,
      const std::vector<std::string>& active_joint_order = {});
};

}  // namespace planning
}  // namespace excavator

#endif  // EXCAVATOR_KINEMATICS__URDF_MODEL_CONVERTER_HPP_
