#ifndef EXCAVATOR_KINEMATICS__URDF_MODEL_CONVERTER_HPP_
#define EXCAVATOR_KINEMATICS__URDF_MODEL_CONVERTER_HPP_

#include <string>

#include "excavator_kinematics/kinematics_types.hpp"

namespace tramcar::planning {

class UrdfModelConverter {
 public:
  static KinematicsResult FromFile(const std::string& path,
                                   ModelDescription* description);
  static KinematicsResult FromString(const std::string& xml,
                                     ModelDescription* description);
};

}  // namespace tramcar::planning

#endif  // EXCAVATOR_KINEMATICS__URDF_MODEL_CONVERTER_HPP_
